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

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::thread th_tracking_;
        std::atomic<bool> terminated_{ false };
	};
} // namespace whi_3DObjectTracking
