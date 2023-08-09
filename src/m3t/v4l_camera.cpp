#include <m3t/v4l_camera.h>

namespace m3t
{
V4lColorCamera::V4lColorCamera(const std::string& Name)
    : ColorCamera(Name) {}

V4lColorCamera::V4lColorCamera(const std::string& Name, const std::filesystem::path& MetafilePath)
    : ColorCamera(Name, MetafilePath) {}

bool V4lColorCamera::SetUp()
{
    set_up_ = false;
    if (!metafile_path_.empty())
    {
        if (!loadMetaData())
        {
            return false;
        }
    }
    cap_.open(0, cv::CAP_ANY);
    // if (use_depth_as_world_frame_)
    // {
    //     set_camera2world_pose(*realsense_.color2depth_pose());
    // }
    getIntrinsics();
    SaveMetaDataIfDesired();
    set_up_ = true;
    initial_set_up_ = true;
    
    return UpdateImage(true);
}

bool V4lColorCamera::UpdateImage(bool Synchronized)
{
    if (!set_up_)
    {
        std::cerr << "Set up real sense color camera " << name_ << " first" << std::endl;
        return false;
    }

    // Get frameset and copy data to image
    cap_.read(image_);
    // realsense_.UpdateCapture(realsense_id_, synchronized);
    // cv::Mat{cv::Size{intrinsics_.width, intrinsics_.height}, CV_8UC3,
    //     (void *)realsense_.frameset().get_color_frame().get_data(), cv::Mat::AUTO_STEP}.copyTo(image_);

    SaveImageIfDesired();
    return true;
}

bool V4lColorCamera::useDepthAsWorldFrame() const
{
    return false;
}

const Transform3fA* V4lColorCamera::color2depthPose() const
{
    return nullptr;
}

const Transform3fA* V4lColorCamera::depth2colorPose() const
{
    return nullptr;
}

bool V4lColorCamera::loadMetaData()
{
    // Open file storage from yaml
    cv::FileStorage fs;
    if (!OpenYamlFileStorage(metafile_path_, &fs))
    {
        return false;
    }

    // Read parameters from yaml
    ReadOptionalValueFromYaml(fs, "camera2world_pose", &camera2world_pose_);
    ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
    ReadOptionalValueFromYaml(fs, "save_index", &save_index_);
    ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
    ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
    // ReadOptionalValueFromYaml(fs, "use_depth_as_world_frame", &use_depth_as_world_frame_);
    fs.release();

    // Process parameters
    if (save_directory_.is_relative())
    {
        save_directory_ = metafile_path_.parent_path() / save_directory_;
    }
    world2camera_pose_ = camera2world_pose_.inverse();
    return true;
}

void V4lColorCamera::getIntrinsics()
{
    // const rs2_intrinsics intrinsics = realsense_.profile()
    //                                     .get_stream(RS2_STREAM_COLOR)
    //                                     .as<rs2::video_stream_profile>()
    //                                     .get_intrinsics();
    // intrinsics_.fu = intrinsics.fx;
    // intrinsics_.fv = intrinsics.fy;
    // intrinsics_.ppu = intrinsics.ppx;
    // intrinsics_.ppv = intrinsics.ppy;
    // intrinsics_.width = intrinsics.width;
    // intrinsics_.height = intrinsics.height;
} 
} // namespace m3t
