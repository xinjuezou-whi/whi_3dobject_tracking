// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <m3t/normal_viewer.h>

namespace m3t {

cv::Mat CalculateAlphaBlend(NormalColorViewer* Viewer,
  const cv::Mat &camera_image, const cv::Mat &renderer_image, float opacity)
{
#ifdef DEBUG
  cv::imshow("camera_img", camera_image);
  cv::imshow("rendered_img", renderer_image);
	cv::waitKey(10);
#endif

  // Declare variables
  cv::Mat image{camera_image.size(), CV_8UC3};

  const cv::Vec3b *ptr_camera_image;
  const cv::Vec4b *ptr_renderer_image;
  cv::Vec3b *ptr_image;
  const uchar *val_camera_image;
  const uchar *val_renderer_image;
  uchar *val_image;
  float alpha, alpha_inv;
  float alpha_scale = opacity / 255.0f;

  if (renderer_image.empty())
  {
    std::cout << "[warn]: renderer image is empty" << std::endl;
  }

  // Iterate over all pixels
  const uchar dummy[3] = { 0, 0, 0 };
  for (int v = 0; v < camera_image.rows; ++v)
  {
    ptr_camera_image = camera_image.ptr<cv::Vec3b>(v);
    ptr_renderer_image = renderer_image.ptr<cv::Vec4b>(v);
    ptr_image = image.ptr<cv::Vec3b>(v);
    for (int u = 0; u < camera_image.cols; ++u)
    {
      val_camera_image = ptr_camera_image[u].val;
      val_renderer_image = ptr_renderer_image ? ptr_renderer_image[u].val : dummy;
      val_image = ptr_image[u].val;

      // Blend images
      alpha = float(val_renderer_image[3]) * alpha_scale;
      alpha_inv = 1.0f - alpha;
      val_image[0] = char(val_camera_image[0] * alpha_inv + val_renderer_image[0] * alpha);
      val_image[1] = char(val_camera_image[1] * alpha_inv + val_renderer_image[1] * alpha);
      val_image[2] = char(val_camera_image[2] * alpha_inv + val_renderer_image[2] * alpha);
    }
  }

  if (Viewer)
  {
    Viewer->visualizeFrame(image);
  }

  return image;
}

// draw an arrow
void drawArrow(cv::Mat Image, cv::Point2i Start, cv::Point2i End,
    cv::Scalar Color, int ArrowMagnitude = 9, int Thickness = 1, int LineType = cv::LINE_8, int Shift = 0)
{
    // draw the principle line
    cv::line(Image, Start, End, Color, Thickness, LineType, Shift);
    // compute the angle alpha
    double angle = atan2((double)Start.y - End.y, (double)Start.x - End.x);
    // compute the coordinates of the first segment
    Start.x = (int)(End.x +  ArrowMagnitude * cos(angle + CV_PI / 4.0));
    Start.y = (int)(End.y +  ArrowMagnitude * sin(angle + CV_PI / 4.0));
    // draw the first segment
    cv::line(Image, Start, End, Color, Thickness, LineType, Shift);
    // compute the coordinates of the second segment
    Start.x = (int)(End.x +  ArrowMagnitude * cos(angle - CV_PI / 4.0));
    Start.y = (int)(End.y +  ArrowMagnitude * sin(angle - CV_PI / 4.0));
    // draw the second segment
    cv::line(Image, Start, End, Color, Thickness, LineType, Shift);
}

// draw the 3D coordinate axes
void draw3DCoordinateAxes(cv::Mat Image, const std::vector<cv::Point2f>& Points2dList, int Radius = 4)
{
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar black(0, 0, 0);

    cv::Point2i origin = Points2dList[0];
    cv::Point2i pointX = Points2dList[1];
    cv::Point2i pointY = Points2dList[2];
    cv::Point2i pointZ = Points2dList[3];

    drawArrow(Image, origin, pointX, red, 9, 2);
    drawArrow(Image, origin, pointY, green, 9, 2);
    drawArrow(Image, origin, pointZ, blue, 9, 2);
    cv::circle(Image, origin, Radius, black, -1);
}

NormalColorViewer::NormalColorViewer(
    const std::string &name,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    float opacity)
    : ColorViewer{name, color_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      opacity_{opacity},
      renderer_{"renderer", renderer_geometry_ptr, color_camera_ptr} {}

NormalColorViewer::NormalColorViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<ColorCamera> &color_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr)
    : ColorViewer{name, metafile_path, color_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      renderer_{"renderer", renderer_geometry_ptr, color_camera_ptr} {}

bool NormalColorViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  // Configure and set up renderer
  renderer_.set_renderer_geometry_ptr(renderer_geometry_ptr_);
  renderer_.set_camera_ptr(color_camera_ptr_);
  if (!renderer_.SetUp()) return false;
  set_up_ = true;
  return true;
}

void NormalColorViewer::set_renderer_geometry_ptr(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  set_up_ = false;
}

void NormalColorViewer::set_opacity(float opacity) { opacity_ = opacity; }

bool NormalColorViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up normal color viewer " << name_ << " first"
              << std::endl;
    return false;
  }

  // Render overlay
  renderer_.StartRendering();
  renderer_.FetchNormalImage();

  // Display and save images
  DisplayAndSaveImage(save_index, CalculateAlphaBlend(this,
    color_camera_ptr_->image(), renderer_.normal_image(), opacity_));

  return true;
}

std::shared_ptr<RendererGeometry> NormalColorViewer::renderer_geometry_ptr()
    const {
  return renderer_geometry_ptr_;
}

float NormalColorViewer::opacity() const { return opacity_; }

bool NormalColorViewer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "opacity", &opacity_);
  ReadOptionalValueFromYaml(fs, "display_images", &display_images_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  return true;
}

void NormalColorViewer::visualizeFrame(cv::Mat& DstImage)
{
    float lengthRatio = 0.1;
    for (const auto& body : renderer_geometry_ptr_->render_data_bodies())
    {
        std::vector<cv::Point2f> points2dPoses;
        points2dPoses.push_back(backproject3DPoint(body.body_ptr->geometry2world_pose(), cv::Point3f(0, 0, 0))); // axis center
        points2dPoses.push_back(backproject3DPoint(body.body_ptr->geometry2world_pose(), cv::Point3f(lengthRatio, 0, 0))); // axis x
        points2dPoses.push_back(backproject3DPoint(body.body_ptr->geometry2world_pose(), cv::Point3f(0, lengthRatio, 0))); // axis y
        points2dPoses.push_back(backproject3DPoint(body.body_ptr->geometry2world_pose(), cv::Point3f(0, 0, lengthRatio))); // axis z
        draw3DCoordinateAxes(DstImage, points2dPoses);
    }
}

cv::Point2f NormalColorViewer::backproject3DPoint(const Transform3fA& Pose, const cv::Point3f& Point3d)
{
    // 3D point vector [x y z 1]'
    cv::Mat point3dVector = cv::Mat(4, 1, CV_64FC1);
    point3dVector.at<double>(0) = Point3d.x;
    point3dVector.at<double>(1) = Point3d.y;
    point3dVector.at<double>(2) = Point3d.z;
    point3dVector.at<double>(3) = 1;

    // create the intrinsic camera parameters
    cv::Mat intinsicMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
    intinsicMatrix.at<double>(0, 0) = color_camera_ptr_->intrinsics().fu;  // [ fx   0  cx ]
    intinsicMatrix.at<double>(1, 1) = color_camera_ptr_->intrinsics().fv;  // [  0  fy  cy ]
    intinsicMatrix.at<double>(0, 2) = color_camera_ptr_->intrinsics().width / 2; // [  0   0   1 ]
    intinsicMatrix.at<double>(1, 2) = color_camera_ptr_->intrinsics().height / 2;
    intinsicMatrix.at<double>(2, 2) = 1;
    // convert Eigen to cv::Mat
    cv::Mat poseMatrix = cv::Mat::zeros(3, 4, CV_64FC1);
    poseMatrix.at<double>(0, 0) = Pose.matrix()(0, 0);
    poseMatrix.at<double>(0, 1) = Pose.matrix()(0, 1);
    poseMatrix.at<double>(0, 2) = Pose.matrix()(0, 2);
    poseMatrix.at<double>(1, 0) = Pose.matrix()(1, 0);
    poseMatrix.at<double>(1, 1) = Pose.matrix()(1, 1);
    poseMatrix.at<double>(1, 2) = Pose.matrix()(1, 2);
    poseMatrix.at<double>(2, 0) = Pose.matrix()(2, 0);
    poseMatrix.at<double>(2, 1) = Pose.matrix()(2, 1);
    poseMatrix.at<double>(2, 2) = Pose.matrix()(2, 2);
    poseMatrix.at<double>(0, 3) = Pose.matrix()(0, 3);
    poseMatrix.at<double>(1, 3) = Pose.matrix()(1, 3);
    poseMatrix.at<double>(2, 3) = Pose.matrix()(2, 3);

    // 2D point vector [u v 1]'
    cv::Mat point2dVector = cv::Mat(4, 1, CV_64FC1);
    point2dVector = intinsicMatrix * poseMatrix * point3dVector;

    // normalization of [u v]'
    cv::Point2f point2d;
    point2d.x = (float)(point2dVector.at<double>(0) / point2dVector.at<double>(2));
    point2d.y = (float)(point2dVector.at<double>(1) / point2dVector.at<double>(2));

    return point2d;
}

NormalDepthViewer::NormalDepthViewer(
    const std::string &name,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr,
    float min_depth, float max_depth, float opacity)
    : DepthViewer{name, depth_camera_ptr, min_depth, max_depth},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      opacity_{opacity},
      renderer_{"renderer", renderer_geometry_ptr, depth_camera_ptr} {}

NormalDepthViewer::NormalDepthViewer(
    const std::string &name, const std::filesystem::path &metafile_path,
    const std::shared_ptr<DepthCamera> &depth_camera_ptr,
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr)
    : DepthViewer{name, metafile_path, depth_camera_ptr},
      renderer_geometry_ptr_{renderer_geometry_ptr},
      renderer_{"renderer", renderer_geometry_ptr, depth_camera_ptr} {}

bool NormalDepthViewer::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!depth_camera_ptr_->set_up()) {
    std::cerr << "Depth camera " << depth_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }
  if (!renderer_geometry_ptr_->set_up()) {
    std::cerr << "Renderer geometry " << renderer_geometry_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  // Configure and set up renderer
  renderer_.set_renderer_geometry_ptr(renderer_geometry_ptr_);
  renderer_.set_camera_ptr(depth_camera_ptr_);
  if (!renderer_.SetUp()) return false;
  set_up_ = true;
  return true;
}

void NormalDepthViewer::set_renderer_geometry_ptr(
    const std::shared_ptr<RendererGeometry> &renderer_geometry_ptr) {
  renderer_geometry_ptr_ = renderer_geometry_ptr;
  set_up_ = false;
}

void NormalDepthViewer::set_opacity(float opacity) { opacity_ = opacity; }

bool NormalDepthViewer::UpdateViewer(int save_index) {
  if (!set_up_) {
    std::cerr << "Set up normal depth viewer " << name_ << " first"
              << std::endl;
    return false;
  }

  // Render overlay
  renderer_.StartRendering();
  renderer_.FetchNormalImage();

  // Display and save images
  cv::Mat normalized_depth_image_rgb;
  cv::cvtColor(depth_camera_ptr_->NormalizedDepthImage(min_depth_, max_depth_),
               normalized_depth_image_rgb, cv::COLOR_GRAY2BGR);
  DisplayAndSaveImage(save_index, CalculateAlphaBlend(nullptr,
    normalized_depth_image_rgb, renderer_.normal_image(), opacity_));

  return true;
}

std::shared_ptr<RendererGeometry> NormalDepthViewer::renderer_geometry_ptr()
    const {
  return renderer_geometry_ptr_;
}

float NormalDepthViewer::opacity() const { return opacity_; }

bool NormalDepthViewer::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  ReadOptionalValueFromYaml(fs, "min_depth", &min_depth_);
  ReadOptionalValueFromYaml(fs, "max_depth", &max_depth_);
  ReadOptionalValueFromYaml(fs, "opacity", &opacity_);
  ReadOptionalValueFromYaml(fs, "display_images", &display_images_);
  ReadOptionalValueFromYaml(fs, "save_images", &save_images_);
  ReadOptionalValueFromYaml(fs, "save_directory", &save_directory_);
  ReadOptionalValueFromYaml(fs, "save_image_type", &save_image_type_);
  fs.release();

  // Process parameters
  if (save_directory_.is_relative())
    save_directory_ = metafile_path_.parent_path() / save_directory_;
  return true;
}

}  // namespace m3t
