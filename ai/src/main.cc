
#include "realsenseloader.h"
#include "imu.h"
#include <librealsense2/rs.h>
#include <nvblox/nvblox.h>
#include <iostream>

int main() {
  drone::RealSenseLoader realsense_loader;
  drone::CameraIntrinsics depth_intrinsics =
      realsense_loader.getDepthIntrinsics();
  drone::CameraIntrinsics color_intrinsics =
      realsense_loader.getColorIntrinsics();
  nvblox::Camera nvblox_depth_camera(
      depth_intrinsics.fu, depth_intrinsics.fv, depth_intrinsics.cu,
      depth_intrinsics.cv, depth_intrinsics.width, depth_intrinsics.height);
  nvblox::Camera nvblox_color_camera(
      color_intrinsics.fu, color_intrinsics.fv, color_intrinsics.cu,
      color_intrinsics.cv, color_intrinsics.width, color_intrinsics.height);
  float voxel_size = 0.05f;
  int iterations = 10;
  int i = 0;
  drone::ImuPoseEstimator<> imu_pose_estimator;
  auto calibration_frame = realsense_loader.loadImage();
  for (auto &frame_set : realsense_loader) {
    auto gyro = frame_set.getGyro();
    auto accel = frame_set.getAccel();
    std::cout << "Accel:" << accel.first << "\n";
    imu_pose_estimator.updateMotion(gyro, accel);
    auto T_L_C = imu_pose_estimator.getPose();
    std::cout << "Current Pose: \n"
              << T_L_C.matrix() << std::endl;
    if (++i >= iterations)
      break;
  }

  return 0;
}
