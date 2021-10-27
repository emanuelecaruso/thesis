#pragma once
#include "defs.h"
#include "image.h"
#include "camera.h"
#include <cuda_runtime.h>

using namespace pr;

class Camera_gpu{
  public:

    std::string name_;
    float lens_;
    float aspect_;
    float width_;
    int resolution_;
    float max_depth_;
    float min_depth_;
    Eigen::Matrix3f K_;
    Eigen::Matrix3f Kinv_;
    cv::cuda::PtrStepSz<float> invdepth_map_;
    cv::cuda::PtrStepSz<uchar3> image_rgb_;
    Eigen::Isometry3f frame_world_wrt_camera_;
    Eigen::Isometry3f frame_camera_wrt_world_;
    Eigen::Isometry3f frame_world_wrt_camera_gt_;
    Eigen::Isometry3f frame_camera_wrt_world_gt_;
    Cp_gpu* cp_array_;

    Camera_gpu(std::string name, float lens, float aspect, float width, int resolution,
       float max_depth,float min_depth, Eigen::Matrix3f K, Eigen::Matrix3f Kinv, Eigen::Isometry3f frame_camera_wrt_world, Eigen::Isometry3f frame_world_wrt_camera,
       Eigen::Isometry3f frame_camera_wrt_world_gt, Eigen::Isometry3f frame_world_wrt_camera_gt,
       cv::cuda::PtrStepSz<float> invdepth_map, cv::cuda::PtrStepSz<uchar3> image_rgb, Cp_gpu* cp_array){
       name_ = name;
       lens_ = lens;
       aspect_ = aspect;
       width_ = width;
       resolution_ = resolution;
       max_depth_ = max_depth;
       min_depth_ = min_depth;
       K_ = K;
       Kinv_ = Kinv;
       frame_camera_wrt_world_ = frame_camera_wrt_world;
       frame_world_wrt_camera_ = frame_world_wrt_camera;
       frame_camera_wrt_world_gt_ = frame_camera_wrt_world_gt;
       frame_world_wrt_camera_gt_ = frame_world_wrt_camera_gt;
       invdepth_map_ = invdepth_map;
       image_rgb_ = image_rgb;
       cp_array_ = cp_array;

    };


    __device__ void pixelCoords2uv(Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv);
    __device__ void uv2pixelCoords( Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords);

    __device__ void pointAtDepth(Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p);
    __device__ bool projectPoint(Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z );


    inline Camera_gpu* clone(){
      return new Camera_gpu(*this);
    }


};
