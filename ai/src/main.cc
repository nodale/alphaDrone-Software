
#include "nvblox/io/mesh_io.h"
#include "realsenseloader.h"
#include <librealsense2/rs.h>
#include <nvblox/nvblox.h>

int main() {
    drone::RealSenseLoader realsense_loader;
    drone::CameraIntrinsics depth_intrinsics = realsense_loader.getDepthIntrinsics();
    drone::CameraIntrinsics color_intrinsics = realsense_loader.getColorIntrinsics();
    nvblox::Camera nvblox_depth_camera(depth_intrinsics.fu, depth_intrinsics.fv, depth_intrinsics.cu, depth_intrinsics.cv, depth_intrinsics.width, depth_intrinsics.height);
    nvblox::Camera nvblox_color_camera(color_intrinsics.fu, color_intrinsics.fv, color_intrinsics.cu, color_intrinsics.cv, color_intrinsics.width, color_intrinsics.height);
    float voxel_size = 0.05f;
    int iterations = 1000;
    int i = 0;
    nvblox::Mapper mapper(voxel_size);
    for(auto& frame_set : realsense_loader){
        auto transform = frame_set.getTransform();
        drone::integrateDepth(mapper, frame_set.getDepthFrame(), {transform}, nvblox_depth_camera);
        drone::integrateColor(mapper, frame_set.getColorFrame(), {transform}, nvblox_depth_camera);
        mapper.updateColorMesh();
        if(++i >= iterations) break;
    }
    nvblox::io::outputColorMeshLayerToPly(mapper.color_mesh_layer(), "color_mesh.ply");
    return 0;
}
