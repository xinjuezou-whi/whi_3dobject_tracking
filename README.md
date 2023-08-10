# whi_3dobject_tracking
3D object tracking node leveraging the M3T from 3DObjectTracking

![tracking_ros](https://github.com/xinjuezou-whi/whi_3dobject_tracking/assets/72239958/3309453c-e553-490c-a9af-d7e68ba1c717)

## Citation

If you find our work useful, please cite us with: 

```
@InProceedings{Stoiber_2023_IROS,
    author = {Stoiber, Manuel and Elsayed, Mariam and Rechert, Anne E. and Steidle, Florian and Lee, Dongheui and Triebel, Rudolph},
    title  = {Fusing Visual Appearance and Geometry for Multi-Modality 6DoF Object Tracking},
    year   = {2023},
}
```

## Dependency

The WHI's msg interface:
```
cd <your workspace>/src
git clone https://github.com/xinjuezou-whi/whi_motion_interface.git
```

And the Intel librealsense2, please refer to its [website](https://github.com/IntelRealSense/librealsense)

## Build

```
cd <your workspace>/src
git clone https://github.com/xinjuezou-whi/whi_3dobject_tracking.git
catkin build
source <your workspace>/devel/setup.bash
```

## Publish

**color_view**(sensor_msgs::Image): the stream of color camera

**depth_view**(sensor_msgs::Image): the stream of depth camera

**tcp_pose**(whi_interfaces::WhiTcpPose): the estimated 6DOF pose

## Config

```
whi_3dobject_tracking:
  view_color: true
  view_depth: true
  visualize_pose_result: true
  use_region_modality: true
  use_depth_modality: true
  use_texture_modality: false
  measure_occlusions: false
  model_occlusions: false
  tracking_on_start: false
  bodies: ["io_485"] #[ "io_485", "triangle" ]
  directory: /home/whi/catkin_workspace/src/whi_3dobject_tracking/data
  color_topic: color_view
  depth_topic: depth_view
  pose_topic: tcp_pose
```
