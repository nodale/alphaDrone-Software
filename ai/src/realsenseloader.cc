#include "nvblox/sensors/camera.h"
#include "nvblox/sensors/image.h"
#include "realsenseloader.h"

namespace drone {
static nvblox::DepthImage
convertDepthFrameToDepthImage(const rs2::depth_frame &depth_frame) {
  auto width = depth_frame.get_width();
  auto height = depth_frame.get_height();
  nvblox::DepthImage depth_image(height, width);
  const uint16_t *depth_data =
      reinterpret_cast<const uint16_t *>(depth_frame.get_data());
  for (auto i = 0; i < height; ++i) {
    for (auto j = 0; j < width; ++j) {
      depth_image(i, j) = static_cast<float>(depth_data[i * width + j]);
    }
  }
  return depth_image;
}

static nvblox::ColorImage
convertColorFrameToColorImage(const rs2::frame &color_frame) {
  auto width = color_frame.as<rs2::video_frame>().get_width();
  auto height = color_frame.as<rs2::video_frame>().get_height();
  nvblox::ColorImage color_image(height, width);
  const uint8_t *color_data =
      reinterpret_cast<const uint8_t *>(color_frame.get_data());
  for (auto i = 0; i < height; ++i) {
    for (auto j = 0; j < width; ++j) {
      int idx = (i * width + j) * 3;
      color_image(i, j) = nvblox::Color(color_data[idx], color_data[idx + 1],
                                        color_data[idx + 2]);
    }
  }
  return color_image;
}

void integrateDepth(nvblox::Mapper &mapper, const rs2::depth_frame &depth_frame,
                    nvblox::Transform T_L_C, const nvblox::Camera &camera) {
  nvblox::DepthImage depth_image = convertDepthFrameToDepthImage(depth_frame);
  nvblox::DepthImage gpu_depth_image =
      nvblox::DepthImage(depth_image.rows(), depth_image.cols());
  gpu_depth_image.copyFrom(depth_image);
  mapper.integrateDepth(depth_image, T_L_C, camera);
}

void integrateColor(nvblox::Mapper &mapper, const rs2::frame &color_frame,
                    nvblox::Transform T_L_C, const nvblox::Camera &camera) {
  nvblox::ColorImage color_image = convertColorFrameToColorImage(color_frame);
  mapper.integrateColor(color_image, T_L_C, camera);
}

}; // namespace drone
