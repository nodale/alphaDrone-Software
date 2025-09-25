#pragma once
#include "librealsense2/h/rs_sensor.h"
#include "librealsense2/h/rs_types.h"
#include "librealsense2/hpp/rs_frame.hpp"
#include "nvblox/sensors/camera.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <nvblox/nvblox.h>

namespace drone {
using PosePair = std::pair<Eigen::Vector3f, double>;    
struct CameraIntrinsics {
  float fu, fv, cu, cv;
  int width, height;
};

struct RealSenseFrameSet {
  rs2::frameset frameset;

  rs2::frame getColorFrame() { return frameset.get_color_frame(); }

  rs2::frame getDepthFrame() { return frameset.get_depth_frame(); }

  PosePair getGyro() {
    auto gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO);
    if (!gyro_frame)
      throw std::runtime_error("No gyro frame available");
    auto gyro_data = gyro_frame.as<rs2::motion_frame>().get_motion_data();
    return std::make_pair<Eigen::Vector3f, double>({gyro_data.x, gyro_data.y, gyro_data.z}, gyro_frame.get_timestamp());
  }

  PosePair getAccel() {
    auto accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL);
    if (!accel_frame)
      throw std::runtime_error("No accel frame available");
    auto accel_data = accel_frame.as<rs2::motion_frame>().get_motion_data();
    return std::make_pair<Eigen::Vector3f, double>({accel_data.x, accel_data.y, accel_data.z}, accel_frame.get_timestamp());
  }

  Eigen::Isometry3f getTransform() { return Eigen::Isometry3f::Identity(); }
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
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    auto profile =  pipe_.start(cfg);
  };
  ~RealSenseLoader() { pipe_.stop(); };

  CameraIntrinsics getDepthIntrinsics() const {
    auto profile = pipe_.get_active_profile();
    auto depth_stream =
        profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrin = depth_stream.get_intrinsics();
    return {intrin.fx,  intrin.fy,    intrin.ppx,
            intrin.ppy, intrin.width, intrin.height};
  }

  CameraIntrinsics getColorIntrinsics() const {
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

  struct iterator {
    RealSenseLoader *loader;
    RealSenseFrameSet frame_set;
    bool operator!=(const iterator &other) const { return true; }
    void operator++() { frame_set = loader->loadImage(); }
    RealSenseFrameSet &operator*() { return frame_set; }
    iterator(RealSenseLoader *loader) : loader(loader) {
      if (loader)
        frame_set = loader->loadImage();
    }
  };

  iterator begin() { return iterator(this); }
  iterator end() { return iterator(nullptr); }

private:
  rs2::pipeline pipe_;
  rs2::frameset frameset_;
};
} // namespace drone
