/******************************************************************
3D object tracking and its 6DOF pose estimation under ROS 1

Features:
- 3D target tracking and its 6DOF pose estimation
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-08: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include <memory>
#include <thread>

namespace whi_3DObjectTracking
{
	class TriDObjectTracking
	{
    public:
        TriDObjectTracking(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~TriDObjectTracking();

    protected:
        void init();
        void initM3t();
        void poseCallback(const std::string& Object, const Eigen::Isometry3d& Pose);
        void colorImageCallback(const std::string& Name, const cv::Mat& Image);
        void colorOverlayImageCallback(const std::string& Name, const cv::Mat& Image);
        void depthImageCallback(const std::string& Name, const cv::Mat& Image);
        void depthOverlayImageCallback(const std::string& Name, const cv::Mat& Image);

    protected:
        static void toImageMsg(const cv::Mat& SrcImg, const std::string& SrcEncoding,
            sensor_msgs::Image& RosImage);
        static void publishImage(std::shared_ptr<image_transport::Publisher> Publisher,
            const std::string& Name, const cv::Mat& Image, const std::string& Encoding, unsigned long Seq = 0);
        static void toggleRightAndLeftHand(const Eigen::Isometry3d& Src, Eigen::Isometry3d& Dst);
        static void scalingEuler(geometry_msgs::Quaternion& Src, const std::array<double, 3>& Multiplier);
        static bool retrieveTransform(std::shared_ptr<ros::NodeHandle> Node,
            const std::string ParamName, geometry_msgs::TransformStamped& Trans);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::thread th_tracking_;
        std::unique_ptr<ros::Publisher> pub_pose_{ nullptr };
        std::unique_ptr<ros::ServiceClient> client_pose_{ nullptr };
        std::shared_ptr<image_transport::Publisher> pub_color_{ nullptr };
        std::shared_ptr<image_transport::Publisher> pub_color_overlay_{ nullptr };
        std::shared_ptr<image_transport::Publisher> pub_depth_{ nullptr };
        std::shared_ptr<image_transport::Publisher> pub_depth_overlay_{ nullptr };
        std::unique_ptr<image_transport::ImageTransport> image_transport_{ nullptr };
        std::string pose_frame_{ "world" };
        std::shared_ptr<geometry_msgs::TransformStamped> world_to_tcp_{ nullptr };
        std::shared_ptr<geometry_msgs::TransformStamped> object_to_tcp_{ nullptr };
        std::array<double, 3> position_reference_;
        std::atomic_bool service_standby_{ true };
        std::map<std::string, Eigen::Isometry3d> link_2_world_pose_map_;
        std::array<double, 3> euler_multipliers_;
        unsigned long seq_{ 0 };
	};
} // namespace whi_3DObjectTracking
