#pragma once
#include "camera_cpu.cuh"
#include "camera_gpu.cuh"
#include "image.h"
#include <cuda_runtime.h>
#include "environment.cuh"
// #include "matplotlibcpp.h"


#define NUM_INTERPOLATIONS 64
#define MAX_THREADS 1024

struct cameraDataForDtam{
  Eigen::Matrix3f T_r;
  Eigen::Vector3f T_t;
  Eigen::Vector2f cam_r_projected_on_cam_m;
  float cam_r_depth_on_camera_m;
  bool cam_r_in_front;
  cv::cuda::PtrStepSz<float3> query_proj_matrix;
};

class Mapper{

  public:
    Mapper(Environment_gpu* environment){};

    void addCamera(Camera_cpu* camera_cpu, Camera_gpu* camera_gpu);
    double StudyCostVolumeMin(int index_m, int row, int col, bool showbaseline);
    void UpdateDepthmap();
    void StateFromGt();
    void ComputeCostVolumeMin();
    void UpdateCostVolume(int index_m, bool occl );
    void depthSamplingInit(Environment_gpu* environment);
    void Initialize();

    CameraVector_cpu camera_vector_cpu_;
    cv::cuda::GpuMat depth_groundtruth_;
    int index_r_;
    int frames_computed_=0;

  private:

    int switch_idx_;
    float switch_depth_;

    cv::cuda::GpuMat invdepth_r_volume_;
    cv::cuda::GpuMat cost_volume_;
    cv::cuda::GpuMat weight_matrix_;
    cv::cuda::GpuMat query_proj_matrix_;
    // cv::cuda::GpuMat variance_matrix_;
    cameraDataForDtam* camera_data_for_dtam_;
    Eigen::Matrix3f T_r;
    Eigen::Vector3f T_t;


    void prepareCameraForDtam(int index_m);





};
