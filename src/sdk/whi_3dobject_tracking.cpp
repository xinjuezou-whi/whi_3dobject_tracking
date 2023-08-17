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
#include "whi_interfaces/WhiSrvTcpPose.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
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
        /// init infrastructure
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
        // service client
        std::string poseService;
        node_handle_->param("pose_service", poseService, std::string());
        if (!poseService.empty())
        {
            client_pose_ = std::make_unique<ros::ServiceClient>(
                node_handle_->serviceClient<whi_interfaces::WhiSrvTcpPose>(poseService));
        }

        /// init M3T
        initM3t();
    }

    void TriDObjectTracking::initM3t()
    {
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
        for (const auto& bodyName : bodyNames)
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
            auto regionModality{ std::make_shared<m3t::RegionModality>(bodyName,
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
            auto detector{std::make_shared<m3t::StaticDetector>(bodyName, detectorPath, optimizer)};
            tracker->AddDetector(detector);
        }
        // start tracking
        if (tracker->SetUp())
        {
            // acquire the link2world poses
            for (const auto& it : tracker->detector_ptrs())
            {
                Eigen::Isometry3f pose;
                pose.translation() = ((m3t::StaticDetector*)it.get())->link2world_pose().translation();
                pose.linear() = ((m3t::StaticDetector*)it.get())->link2world_pose().rotation();
                link_2_world_pose_map_[it->name()] = pose.cast<double>();
            }

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
        // convert left-hand to right-hand frame
        Eigen::Isometry3d rightPose;
        toggleRightAndLeftHand(Pose, rightPose);
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

        // transform to tcp frame if there is 
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

        if (pub_pose_)
        {
            whi_interfaces::WhiTcpPose msg;
            msg.tcp_pose.header.frame_id = pose_frame_;
            msg.tcp_pose.header.stamp = ros::Time::now();
            msg.tcp_pose.pose = Eigen::toMsg(transformed);
#ifndef BEBUG
#ifndef TRANS
            Eigen::Vector3d unitX(1.0, 0.0, 0.0);
            Eigen::Vector3d norm = transformed.rotation() * unitX;
            Eigen::Vector3d rZ = unitX.cross(norm) / unitX.cross(norm).norm();
            Eigen::Vector3d rY = rZ.cross(norm) / rZ.cross(norm).norm();
            Eigen::Matrix3d rotation;
            rotation << norm, rY, rZ;
            Eigen::Quaterniond eigenQ(rotation);
            geometry_msgs::Quaternion msgQ = tf2::toMsg(eigenQ);
            tf2::Quaternion tfQ(msgQ.x, msgQ.y, msgQ.z, msgQ.w);
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		    tf2::Matrix3x3(tfQ).getRPY(roll, pitch, yaw);
            std::cout << "aligned roll:" << angles::to_degrees(roll) << ",pitch:" <<
                angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;

            tf2::Quaternion q(msg.tcp_pose.pose.orientation.x, msg.tcp_pose.pose.orientation.y,
                msg.tcp_pose.pose.orientation.z, msg.tcp_pose.pose.orientation.w);
  		    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            std::cout << "m3t roll:" << angles::to_degrees(roll) << ",pitch:" <<
                angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;
		    // // q.setRPY(0.25 * m3t::kPi - roll, 0.25 * m3t::kPi - pitch, yaw);
            // q.setRPY(roll, 0.0, 0.0);
            // msg.tcp_pose.pose.orientation = tf2::toMsg(q);
            // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            // std::cout << "afterrrrrrrrrrrrrrrrr roll:" << angles::to_degrees(roll) << ",pitch:" <<
            //     angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;
#else
            msg.tcp_pose.pose.orientation.x = 0.0;
            msg.tcp_pose.pose.orientation.y = 0.0;
            msg.tcp_pose.pose.orientation.z = 0.0;
            msg.tcp_pose.pose.orientation.w = 1.0;
#endif
#endif
            pub_pose_->publish(msg);
        }
        if (client_pose_ && service_standby_.load())
        {
            std::thread
            {
                [this, transformed]() -> void
                {
                    this->service_standby_.store(false);

                    if (this->client_pose_->waitForExistence(ros::Duration(3.0)))
				    {
                        whi_interfaces::WhiSrvTcpPose srv;
                        srv.request.tcp_pose.header.frame_id = this->pose_frame_;
                        srv.request.tcp_pose.header.stamp = ros::Time::now();
                        srv.request.tcp_pose.pose = Eigen::toMsg(transformed);
#ifndef DEBUG
#ifdef TRANS
                        tf2::Quaternion q(srv.request.tcp_pose.pose.orientation.x,
                            srv.request.tcp_pose.pose.orientation.y,
                            srv.request.tcp_pose.pose.orientation.z, srv.request.tcp_pose.pose.orientation.w);
                        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                        q.setRPY(0.0, 0.0, yaw / 15.0);
                        srv.request.tcp_pose.pose.orientation = tf2::toMsg(q);
#else
                        srv.request.tcp_pose.pose.orientation.x = 0.0;
                        srv.request.tcp_pose.pose.orientation.y = 0.0;
                        srv.request.tcp_pose.pose.orientation.z = 0.0;
                        srv.request.tcp_pose.pose.orientation.w = 1.0;
#endif
#endif
                        this->client_pose_->call(srv);
				    }
                    else
                    {
                        ROS_WARN_STREAM("failed to get the tcp_pose service, please check if it is launched");
                    }

                    this->service_standby_.store(true);
                }
            }.detach();
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
            // copy by row by row
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

    void TriDObjectTracking::toggleRightAndLeftHand(const Eigen::Isometry3d& Src, Eigen::Isometry3d& Dst)
    {
        Dst = Src;
        Dst.matrix()(0, 1) = Src.matrix()(0, 2);
        Dst.matrix()(0, 2) = Src.matrix()(0, 1);
        Dst.matrix()(1, 0) = Src.matrix()(2, 0);
        Dst.matrix()(1, 1) = Src.matrix()(2, 2);
        Dst.matrix()(1, 2) = Src.matrix()(2, 1);
        Dst.matrix()(2, 0) = Src.matrix()(1, 0);
        Dst.matrix()(2, 1) = Src.matrix()(1, 2);
        Dst.matrix()(2, 2) = Src.matrix()(1, 1);
        Dst.matrix()(1, 3) = Src.matrix()(2, 3);
        Dst.matrix()(2, 3) = Src.matrix()(1, 3);
        Dst.matrix()(3, 0) = 0.0;
        Dst.matrix()(3, 1) = 0.0;
        Dst.matrix()(3, 2) = 0.0;
        Dst.matrix()(3, 3) = 1.0;
    }
} // namespace whi_3DObjectTracking
