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
#include <m3t/tracker.h>
#include <m3t/renderer_geometry.h>
#include <m3t/realsense_camera.h>
#include <m3t/renderer_geometry.h>
#include <m3t/normal_viewer.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>

namespace whi_3DObjectTracking
{
    TriDObjectTracking::TriDObjectTracking(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    TriDObjectTracking::~TriDObjectTracking()
    {
        terminated_.store(true);
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
        bool useRegionModality = true;
        bool useDepthModality = true;
        bool useTextureModality = false;
        bool measureOcclusions = false;
        bool modelOcclusions = false;
        node_handle_->param("view_color", viewColor, true);
        node_handle_->param("view_depth", viewDepth, true);
        node_handle_->param("use_region_modality", useRegionModality, true);
        node_handle_->param("use_depth_modality", useDepthModality, true);
        node_handle_->param("use_texture_modality", useTextureModality, false);
        node_handle_->param("measure_occlusions", measureOcclusions, false);
        node_handle_->param("model_occlusions", modelOcclusions, false);
        std::vector<std::string> bodyNames;
        node_handle_->getParam("bodies", bodyNames);

        /// M3T
        // set up tracker and renderer geometry
        auto tracker{ std::make_shared<m3t::Tracker>("tracker") };
        auto rendererGeometry{ std::make_shared<m3t::RendererGeometry>("renderer geometry") };
        // create cameras
        auto colorCamera{ std::make_shared<m3t::RealSenseColorCamera>("realsense_color") };
        auto depthCamera{ std::make_shared<m3t::RealSenseDepthCamera>("realsense_depth") };
        // set up viewers
        auto colorViewer{ std::make_shared<m3t::NormalColorViewer>("color_viewer",
            colorCamera, rendererGeometry) };
        //if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
        tracker->AddViewer(colorViewer);
        if (viewDepth)
        {
            auto depthViewer{ std::make_shared<m3t::NormalDepthViewer>("depth_viewer",
                depthCamera, rendererGeometry, 0.3f, 1.0f)};
            //if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
            tracker->AddViewer(depthViewer);
        }
        // set up depth renderer
        auto colorDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("color_depth_renderer",
            rendererGeometry, colorCamera) };
        auto depthDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("depth_depth_renderer",
            rendererGeometry, depthCamera) } ;
        // set up silhouette renderer
        auto colorSilhouetteRenderer{ std::make_shared<m3t::FocusedSilhouetteRenderer>("color_silhouette_renderer",
            rendererGeometry, colorCamera) };

        th_tracking_ = std::thread
        {
            [this]() -> void
            {
                while (!terminated_.load())
                {
                    std::cout << "dddddddddddddd" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        };
    }
} // namespace whi_3DObjectTracking
