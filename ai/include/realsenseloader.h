#pragma once
#include "librealsense2/h/rs_sensor.h"
#include "nvblox/sensors/camera.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <generator>
#include <librealsense2/rs.hpp>
#include <nvblox/nvblox.h>
#include <stdexcept>

namespace drone {
struct CameraIntrinsics {
  float fu, fv, cu, cv;
  int width, height;
};

struct RealSenseFrameSet {
  rs2::frameset frameset;

  rs2::frame getColorFrame() { return frameset.get_color_frame(); }

  rs2::frame getDepthFrame() { return frameset.get_depth_frame(); }

  rs2_pose getPoseFrame() {
    auto pose_frame = frameset.get_pose_frame();
    if (!pose_frame)
      throw std::runtime_error("No pose frame available");
    return pose_frame.get_pose_data();
  }

  Eigen::Isometry3f getTransform() {
    auto pose = getPoseFrame();
    Eigen::Quaternionf q(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                         pose.rotation.z);
    Eigen::Vector3f t(pose.translation.x, pose.translation.y,
                      pose.translation.z);
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.rotate(q);
    T.pretranslate(t);
    return T;
  }
};

void integrateDepth(nvblox::Mapper &mapper, const rs2::depth_frame &depth_frame,
                    nvblox::Transform T_L_C, const nvblox::Camera &camera);
void integrateColor(nvblox::Mapper &mapper, const rs2::frame &color_frame,
                    nvblox::Transform T_L_C, const nvblox::Camera &camera);

class RealSenseLoader {
public:
  RealSenseLoader() : pipe_() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    pipe_.start(cfg);
  };
  ~RealSenseLoader() { pipe_.stop(); };

  CameraIntrinsics getDepthIntrinsics() {
    auto profile = pipe_.get_active_profile();
    auto depth_stream =
        profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrin = depth_stream.get_intrinsics();
    return {intrin.fx,  intrin.fy,    intrin.ppx,
            intrin.ppy, intrin.width, intrin.height};
  }

  CameraIntrinsics getColorIntrinsics() {
    auto profile = pipe_.get_active_profile();
    auto color_stream =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intrin = color_stream.get_intrinsics();
    return {intrin.fx,  intrin.fy,    intrin.ppx,
            intrin.ppy, intrin.width, intrin.height};
  }

  RealSenseFrameSet loadImage() {
    return RealSenseFrameSet(pipe_.wait_for_frames());
  }

  std::generator<RealSenseFrameSet> streamFrames() {
      rs2::frameset frameset;
      while((frameset = pipe_.wait_for_frames())){
          co_yield RealSenseFrameSet(frameset);
      }
  }
private:
  rs2::pipeline pipe_;
  rs2::frameset frameset_;
};
} // namespace drone
