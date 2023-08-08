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

Changelog:
2023-08-08: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include <memory>

namespace whi_3DObjectTracking
{
	class 3DObjectTracking
	{
    public:
        3DObjectTracking(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~3DObjectTracking() = default;

    protected:
        void init();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	};
} // namespace whi_3DObjectTracking
