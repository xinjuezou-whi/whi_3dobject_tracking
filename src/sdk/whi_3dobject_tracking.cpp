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
#include <m3t/region_model.h>
#include <m3t/depth_model.h>
#include <m3t/region_modality.h>
#include <m3t/texture_modality.h>
#include <m3t/depth_modality.h>
#include <m3t/link.h>
#include <m3t/static_detector.h>

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
        // setup tracker and renderer geometry
        auto tracker{ std::make_shared<m3t::Tracker>("tracker") };
        auto rendererGeometry{ std::make_shared<m3t::RendererGeometry>("renderer geometry") };
        // create cameras
        auto colorCamera{ std::make_shared<m3t::RealSenseColorCamera>("realsense_color") };
        auto depthCamera{ std::make_shared<m3t::RealSenseDepthCamera>("realsense_depth") };
        // setup viewers
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
                directory / (bodyName + "_region_model.bin"))};
            auto depthModel{ std::make_shared<m3t::DepthModel>(bodyName + "_depth_model", body,
                directory / (bodyName + "_depth_model.bin"))};
            // setup modalities
            auto regionModality{ std::make_shared<m3t::RegionModality>(bodyName + "_region_modality",
                body, colorCamera, regionModel)};
            auto textureModality{ std::make_shared<m3t::TextureModality>(bodyName + "_texture_modality",
                body, colorCamera, colorSilhouetteRenderer)};
            auto depthModality{ std::make_shared<m3t::DepthModality>(bodyName + "_depth_modality",
                body, depthCamera, depthModel)};
            //if (kVisualizePoseResult)
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

        th_tracking_ = std::thread
        {
            [this]() -> void
            {
                while (!terminated_.load())
                {
                    std::cout << "TODO" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        };
    }
} // namespace whi_3DObjectTracking
