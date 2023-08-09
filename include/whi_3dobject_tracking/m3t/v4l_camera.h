#pragma once
#include <filesystem/filesystem.h>
#include <m3t/camera.h>
#include <m3t/common.h>
#include <opencv2/videoio.hpp>

namespace m3t
{
/**
 * \brief Singleton class that allows getting data from a single RealSense
 * instance and that is used by \ref RealSenseColorCamera and \ref
 * RealSenseDepthCamera.
 *
 * \details The method `UpdateCapture()` updates the `capture` object if
 * `UpdateCapture()` was already called with the same `id` before. If
 * `UpdateCapture()` was not yet called by the `id`, the same capture is used.
 * If the capture is updated, all memory values except for the `id` that called
 * the function are reset. All methods that are required to operate multiple
 * \ref Camera objects are thread-safe.
 */
// class
// {
// public:
//     /// Singleton instance getter
//     static RealSense &GetInstance();
//     RealSense(const RealSense &) = delete;
//     RealSense &operator=(const RealSense &) = delete;
//     ~RealSense();

//     // Configuration and setup
//     void UseColorCamera();
//     void UseDepthCamera();
//     int RegisterID();
//     bool UnregisterID(int id);
//     bool SetUp();

//     // Main methods
//     bool UpdateCapture(int id, bool synchronized);

//     // Getters
//     bool use_color_camera() const;
//     bool use_depth_camera() const;
//     const rs2::frameset &frameset() const;
//     const rs2::pipeline_profile &profile() const;
//     const Transform3fA *color2depth_pose() const;
//     const Transform3fA *depth2color_pose() const;

// private:
//     RealSense() = default;

//     // Private data
//     rs2::config config_;
//     rs2::pipeline pipe_;
//     std::map<int, bool> update_capture_ids_{};
//     int next_id_ = 0;

//     // Public data
//     rs2::pipeline_profile profile_;
//     rs2::frameset frameset_;
//     Transform3fA color2depth_pose_{Transform3fA::Identity()};
//     Transform3fA depth2color_pose_{Transform3fA::Identity()};

//     // Internal state variables
//     std::mutex mutex_;
//     bool use_color_camera_ = false;
//     bool use_depth_camera_ = false;
//     bool initial_set_up_ = false;
// };

/**
 * \brief \ref Camera that allows getting color images from a \ref RealSense
 * camera.
 *
 * @param use_depth_as_world_frame specifies the depth camera frame as world
 * frame and automatically defines `camera2world_pose` as `color2depth_pose`.
 */
class V4lColorCamera : public ColorCamera
{
public:
    // Constructors, destructor, and setup method
    V4lColorCamera(const std::string& Name);
    V4lColorCamera(const std::string& Name, const std::filesystem::path& MetafilePath);
    V4lColorCamera(const V4lColorCamera&) = delete;
    V4lColorCamera &operator=(const V4lColorCamera&) = delete;
    ~V4lColorCamera() = default;
    bool SetUp() override;

    // Main method
    bool UpdateImage(bool Synchronized) override;

    // Getters
    bool useDepthAsWorldFrame() const;
    const Transform3fA* color2depthPose() const;
    const Transform3fA* depth2colorPose() const;

private:
    // Helper methods
    bool loadMetaData();
    void getIntrinsics();

    // Data
    int realsense_id_{};
    bool use_depth_as_world_frame_ = false;
    bool initial_set_up_ = false;
    cv::VideoCapture cap_;
};
} // namespace m3t
