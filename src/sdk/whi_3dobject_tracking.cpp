/******************************************************************
3D object tracking to get the 6DOF pose under ROS 1

Features:
- 6DOF pose
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_3dobject_tracking/whi_3dobject_tracking.h"
#include "whi_interfaces/WhiTcpPose.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <m3t/tracker.h>
#include <m3t/renderer_geometry.h>
#include <m3t/realsense_camera.h>
#include <m3t/renderer_geometry.h>
#include <m3t/normal_viewer.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/region_model.h>
#include <m3t/depth_model.h>
#include <m3t/region_modality.h>
#include <m3t/texture_modality.h>
#include <m3t/depth_modality.h>
#include <m3t/link.h>
#include <m3t/static_detector.h>

#include <boost/endian/conversion.hpp>

template <typename T>
static int signOf(T Val)
{
    return T(0) == Val ? 1 : (T(0) < Val) - (Val < T(0));
}

namespace whi_3DObjectTracking
{
    TriDObjectTracking::TriDObjectTracking(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    TriDObjectTracking::~TriDObjectTracking()
    {
        if (th_tracking_.joinable())
        {
            th_tracking_.join();
        }
    }

    void TriDObjectTracking::init()
    {
        // params
        bool viewColor = true;
        bool viewDepth = true;
        bool visualizePoseResult = true;
        bool useRegionModality = true;
        bool useDepthModality = true;
        bool useTextureModality = false;
        bool measureOcclusions = false;
        bool modelOcclusions = false;
        node_handle_->param("view_color", viewColor, true);
        node_handle_->param("view_depth", viewDepth, true);
        node_handle_->param("visualize_pose_result", visualizePoseResult, true);
        node_handle_->param("use_region_modality", useRegionModality, true);
        node_handle_->param("use_depth_modality", useDepthModality, true);
        node_handle_->param("use_texture_modality", useTextureModality, false);
        node_handle_->param("measure_occlusions", measureOcclusions, false);
        node_handle_->param("model_occlusions", modelOcclusions, false);
        std::vector<std::string> bodyNames;
        node_handle_->getParam("bodies", bodyNames);

        // pose frame
        node_handle_->param("pose_frame", pose_frame_, std::string("world"));
        std::vector<double> trans;
        if (node_handle_->getParam("transform_to_tcp", trans))
        {
            tf2::Quaternion q;
            if (trans.size() == 6)
            {
                q.setRPY(trans[3], trans[4], trans[5]);
            }
            else
            {
                q.setRPY(0.0, 0.0, 0.0);
            }
            
            if (trans.size() >= 3)
            {
                tf2::Stamped<tf2::Transform> stamped;
                stamped.setData(tf2::Transform(q, tf2::Vector3(trans[0], trans[1], trans[2])));
                transform_to_tcp_ = std::make_shared<geometry_msgs::TransformStamped>(tf2::toMsg(stamped));
            }
        }
        std::vector<double> transformedRef;
        node_handle_->getParam("transformed_reference", transformedRef);
        for (size_t i = 0; i < transformedRef.size(); ++i)
        {
            transformed_reference_[i] = transformedRef[i];
        }
        
        // publisher
        std::string poseTopic;
        node_handle_->param("pose_topic", poseTopic, std::string());
        if (!poseTopic.empty())
        {
            pub_pose_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<whi_interfaces::WhiTcpPose>(poseTopic, 1));
        }
        image_transport_ = std::make_unique<image_transport::ImageTransport>(*node_handle_);
        std::string colorTopic;
        node_handle_->param("color_topic", colorTopic, std::string());
        if (!colorTopic.empty())
        {
            pub_color_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise(colorTopic, 1));
        }
        std::string depthTopic;
        node_handle_->param("depth_topic", depthTopic, std::string());
        if (!depthTopic.empty())
        {
            pub_depth_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise(depthTopic, 1));
        }

        /// M3T
        // setup tracker and renderer geometry
        auto tracker{ std::make_shared<m3t::Tracker>("tracker") };
        auto rendererGeometry{ std::make_shared<m3t::RendererGeometry>("renderer geometry") };
        // create cameras
        auto colorCamera{ std::make_shared<m3t::RealSenseColorCamera>("realsense_color") };
        auto depthCamera{ std::make_shared<m3t::RealSenseDepthCamera>("realsense_depth") };
        // setup viewers
        if (viewColor)
        {
            auto colorViewer{ std::make_shared<m3t::NormalColorViewer>("color_viewer",
                colorCamera, rendererGeometry) };
            colorViewer->registerViewImageCallback(std::bind(&TriDObjectTracking::colorImageCallback,
                this, std::placeholders::_1, std::placeholders::_2));
            //if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
            tracker->AddViewer(colorViewer);
        }
        if (viewDepth)
        {
            auto depthViewer{ std::make_shared<m3t::NormalDepthViewer>("depth_viewer",
                depthCamera, rendererGeometry, 0.3f, 1.0f)};
            depthViewer->registerViewImageCallback(std::bind(&TriDObjectTracking::depthImageCallback,
                this, std::placeholders::_1, std::placeholders::_2));
            //if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
            tracker->AddViewer(depthViewer);
        }
        // setup depth renderer
        auto colorDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("color_depth_renderer",
            rendererGeometry, colorCamera) };
        auto depthDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("depth_depth_renderer",
            rendererGeometry, depthCamera) } ;
        // set up silhouette renderer
        auto colorSilhouetteRenderer{ std::make_shared<m3t::FocusedSilhouetteRenderer>("color_silhouette_renderer",
            rendererGeometry, colorCamera) };
        // setup bodies
        for (const auto bodyName : bodyNames)
        {
            std::string dirPath;
            node_handle_->param("directory", dirPath, std::string());
            const std::filesystem::path directory{ dirPath };

            // setup body
            std::filesystem::path metafilePath{ directory / (bodyName + ".yaml") };
            auto body{ std::make_shared<m3t::Body>(bodyName, metafilePath) };
            rendererGeometry->AddBody(body);
            colorDepthRenderer->AddReferencedBody(body);
            depthDepthRenderer->AddReferencedBody(body);
            colorSilhouetteRenderer->AddReferencedBody(body);
            // setup models
            auto regionModel{ std::make_shared<m3t::RegionModel>(bodyName + "_region_model", body,
                directory / (bodyName + "_region_model.bin")) };
            auto depthModel{ std::make_shared<m3t::DepthModel>(bodyName + "_depth_model", body,
                directory / (bodyName + "_depth_model.bin")) };
            // setup modalities
            auto regionModality{ std::make_shared<m3t::RegionModality>(bodyName + "_region_modality",
                body, colorCamera, regionModel) };
            regionModality->registerPoseResultCallback(std::bind(&TriDObjectTracking::poseCallback,
                this, std::placeholders::_1, std::placeholders::_2));
            auto textureModality{ std::make_shared<m3t::TextureModality>(bodyName + "_texture_modality",
                body, colorCamera, colorSilhouetteRenderer) };
            auto depthModality{ std::make_shared<m3t::DepthModality>(bodyName + "_depth_modality",
                body, depthCamera, depthModel)};
            if (visualizePoseResult)
            {
                regionModality->set_visualize_pose_result(true);
            }
            if (measureOcclusions)
            {
                regionModality->MeasureOcclusions(depthCamera);
                textureModality->MeasureOcclusions(depthCamera);
                depthModality->MeasureOcclusions();
            }
            if (modelOcclusions)
            {
                regionModality->ModelOcclusions(colorDepthRenderer);
                textureModality->ModelOcclusions(colorDepthRenderer);
                depthModality->ModelOcclusions(depthDepthRenderer);
            }
            // setup link
            auto link{ std::make_shared<m3t::Link>(bodyName + "_link", body) };
            if (useRegionModality)
            {
                link->AddModality(regionModality);
            }
            if (useTextureModality)
            {
                link->AddModality(textureModality);
            }
            if (useDepthModality)
            {
                link->AddModality(depthModality);
            }
            // setup optimizer
            auto optimizer{ std::make_shared<m3t::Optimizer>(bodyName + "_optimizer", link) };
            tracker->AddOptimizer(optimizer);
            // setup detector
            std::filesystem::path detectorPath{ directory / (bodyName + "_static_detector.yaml") };
            auto detector{std::make_shared<m3t::StaticDetector>(bodyName + "_detector", detectorPath, optimizer)};
            tracker->AddDetector(detector);
        }
        // start tracking
        if (tracker->SetUp())
        {
            th_tracking_ = std::thread
            {
                [this, tracker]() -> void
                {
                    bool trackingOnStart = false;
                    this->node_handle_->param("tracking_on_start", trackingOnStart, false);
                    tracker->RunTrackerProcess(true, trackingOnStart);
                }
            };
        }
    }

    void TriDObjectTracking::poseCallback(const std::string& Object, const Eigen::Isometry3d& Pose)
    {
        if (pub_pose_)
        {
            whi_interfaces::WhiTcpPose msg;
            msg.tcp_pose.header.frame_id = pose_frame_;
            msg.tcp_pose.header.stamp = ros::Time::now();

            // convert left-hand to right-hand frame
            Eigen::Isometry3d rightPose(Pose);
            rightPose.matrix()(0, 1) = Pose.matrix()(0, 2);
            rightPose.matrix()(0, 2) = Pose.matrix()(0, 1);
            rightPose.matrix()(1, 0) = Pose.matrix()(2, 0);
            rightPose.matrix()(1, 1) = Pose.matrix()(2, 2);
            rightPose.matrix()(1, 2) = Pose.matrix()(2, 1);
            rightPose.matrix()(2, 0) = Pose.matrix()(1, 0);
            rightPose.matrix()(2, 1) = Pose.matrix()(1, 2);
            rightPose.matrix()(2, 2) = Pose.matrix()(1, 1);
            rightPose.matrix()(1, 3) = Pose.matrix()(2, 3);
            rightPose.matrix()(2, 3) = Pose.matrix()(1, 3);
            rightPose.matrix()(3, 0) = 0.0;
            rightPose.matrix()(3, 1) = 0.0;
            rightPose.matrix()(3, 2) = 0.0;
            rightPose.matrix()(3, 3) = 1.0;
#ifdef DEBUG
            std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
                << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
            std::cout << "rightPose = " << std::endl;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    std::cout << rightPose.matrix()(i, j) << ", ";
                }
                std::cout << std::endl;
            }
#endif

            Eigen::Isometry3d transformed(rightPose);
            if (transform_to_tcp_)
            {
                tf2::doTransform(rightPose, transformed, *transform_to_tcp_);
                for (size_t i = 0; i < 3; ++i)
                {
                    transformed.matrix()(i, 3) =
                        signOf(transformed_reference_[i]) * (transformed.matrix()(i, 3) - transformed_reference_[i]);
                }
            }
            msg.tcp_pose.pose = Eigen::toMsg(transformed);
            pub_pose_->publish(msg);
        }
    }

    void TriDObjectTracking::colorImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        if (pub_color_)
        {
            sensor_msgs::Image imgMsg;
            imgMsg.header.stamp = ros::Time::now();
            imgMsg.header.frame_id = Name;
            toImageMsg(Image, sensor_msgs::image_encodings::BGR8, imgMsg);
            pub_color_->publish(imgMsg);
        }
    }

    void TriDObjectTracking::depthImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        if (pub_depth_)
        {
            sensor_msgs::Image imgMsg;
            imgMsg.header.stamp = ros::Time::now();
            imgMsg.header.frame_id = Name;
            toImageMsg(Image, sensor_msgs::image_encodings::BGR8, imgMsg);
            pub_depth_->publish(imgMsg);
        }
    }

    void TriDObjectTracking::toImageMsg(const cv::Mat& SrcImg, const std::string& SrcEncoding,
        sensor_msgs::Image& RosImage)
    {
        RosImage.height = SrcImg.rows;
        RosImage.width = SrcImg.cols;
        RosImage.encoding = SrcEncoding;
        RosImage.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
        RosImage.step = SrcImg.cols * SrcImg.elemSize();
        size_t size = RosImage.step * SrcImg.rows;
        RosImage.data.resize(size);

        if (SrcImg.isContinuous())
        {
            memcpy((char*)(&RosImage.data[0]), SrcImg.data, size);
        }
        else
        {
            // Copy by row by row
            uchar* rosDataPtr = (uchar*)(&RosImage.data[0]);
            uchar* dataPtr = SrcImg.data;
            for (int i = 0; i < SrcImg.rows; ++i)
            {
                memcpy(rosDataPtr, dataPtr, RosImage.step);
                rosDataPtr += RosImage.step;
                dataPtr += SrcImg.step;
            }
        }
    }
} // namespace whi_3DObjectTracking
