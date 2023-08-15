/******************************************************************
3D object tracking and its 6DOF pose estimation under ROS 1

Features:
- 3D target tracking and its 6DOF pose estimation
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
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
        void depthImageCallback(const std::string& Name, const cv::Mat& Image);

    protected:
        static void toImageMsg(const cv::Mat& SrcImg, const std::string& SrcEncoding,
            sensor_msgs::Image& RosImage);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::thread th_tracking_;
        std::unique_ptr<ros::Publisher> pub_pose_{ nullptr };
        std::unique_ptr<ros::ServiceClient> client_pose_{ nullptr };
        std::unique_ptr<image_transport::Publisher> pub_color_{ nullptr };
        std::unique_ptr<image_transport::Publisher> pub_depth_{ nullptr };
        std::unique_ptr<image_transport::ImageTransport> image_transport_{ nullptr };
        std::string pose_frame_{ "world" };
        std::shared_ptr<geometry_msgs::TransformStamped> transform_to_tcp_{ nullptr };
        std::array<double, 3> transformed_reference_;
        std::thread th_client_;
	};
} // namespace whi_3DObjectTracking
