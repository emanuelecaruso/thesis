#pragma once
#include "defs.h"
#include "image.h"
#include "camera.h"
#include "camera_gpu.cuh"


using namespace pr;


class Camera_cpu : public Camera{
  public:

    cv::cuda::GpuMat invdepth_map_gpu_;
    cv::cuda::GpuMat image_rgb_gpu_;
    Camera_gpu* camera_gpu_;
    Cp_gpu* cp_array_gpu_;

    Camera_cpu(std::string name, float lens, float aspect, float width, int resolution,
       float max_depth, float min_depth, Eigen::Isometry3f* frame_camera_wrt_world, Eigen::Isometry3f* frame_world_wrt_camera,
       Eigen::Isometry3f* frame_camera_wrt_world_gt, Eigen::Isometry3f* frame_world_wrt_camera_gt)

    : Camera(name, lens, aspect, width, resolution, max_depth, min_depth,
            frame_camera_wrt_world, frame_world_wrt_camera, frame_camera_wrt_world_gt,
            frame_world_wrt_camera_gt){}


    void gpuFree();
    void getCamera_gpu();
    void computeDataForDtam(int index_r);
    void cloneCameraImages(Camera* camera);
    void showInvdepthmap(int scale);
    void setGroundtruthPose();
    // inline Camera_cpu* clone(){
    //   return new Camera_cpu(*this);
    // }

};

typedef std::vector<Camera_cpu*> CameraVector_cpu;
