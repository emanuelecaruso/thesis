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

__global__ void prepareCameraForDtam_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m, cv::cuda::PtrStepSz<float3> query_proj_matrix);

__global__ void UpdateCostVolume_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m, cv::cuda::PtrStepSz<int2> cost_volume,
                                                cameraDataForDtam* camera_data_for_dtam_, float* invdepth_r_array, int threshold, bool occl);

__global__ void ComputeWeights_kernel(Camera_gpu* camera_r, cv::cuda::PtrStepSz<float> weight_matrix, float alpha, float beta);

__global__ void StudyCostVolumeMin_kernel(Camera_gpu* camera_r, Camera_gpu* camera_m, cv::cuda::PtrStepSz<int2> cost_volume,
                                    cameraDataForDtam* camera_data_for_dtam_, float* invdepth_r_array, int row, int col,
                                    int threshold, cv::cuda::PtrStepSz<float> depth_groundtruth_, cv::cuda::PtrStepSz<float> a );

__global__ void ComputeCostVolumeMin_kernel( cv::cuda::PtrStepSz<float> d, cv::cuda::PtrStepSz<float> a, float* invdepth_r_array);

__global__ void ComputeGradientSobelImage_kernel(cv::cuda::PtrStepSz<float> image_in, cv::cuda::PtrStepSz<float> image_out);

__global__ void ComputeDivergenceSobelImage_kernel(cv::cuda::PtrStepSz<float> image_in, cv::cuda::PtrStepSz<float> image_out);

__global__ void ComputeWeightedGradientSobelImage_kernel(cv::cuda::PtrStepSz<float> image_in, cv::cuda::PtrStepSz<float> image_out, cv::cuda::PtrStepSz<float> weight_matrix);

__global__ void ComputeWeightedDivergenceSobelImage_kernel(cv::cuda::PtrStepSz<float> image_in, cv::cuda::PtrStepSz<float> image_out, cv::cuda::PtrStepSz<float> weight_matrix);

__global__ void gradDesc_Q_toNormalize_kernel(cv::cuda::PtrStepSz<float> q, cv::cuda::PtrStepSz<float> gradient_d, float eps, float sigma_q, float* vector_to_normalize );

__global__ void gradDesc_Q_kernel(cv::cuda::PtrStepSz<float> q, cv::cuda::PtrStepSz<float> gradient_d, float eps, float sigma_q );

__global__ void gradDesc_D_kernel(cv::cuda::PtrStepSz<float> d, cv::cuda::PtrStepSz<float> a, cv::cuda::PtrStepSz<float> gradient_q, float sigma_d, float theta);

__global__ void search_A_kernel(cv::cuda::PtrStepSz<float> d, cv::cuda::PtrStepSz<float> a, cv::cuda::PtrStepSz<int2> cost_volume , float lambda, float theta, float* invdepth_r_array);

__global__ void sumReduction_kernel(float* v, float* v_r, int size);

__global__ void maxReduction_kernel(float* v, float* v_r, int size);

__global__ void copyArray_kernel(float* original, float* copy);

__global__ void sqrt_kernel(float* v);

__global__ void normalize_Q_kernel(float *norm, cv::cuda::PtrStepSz<float> q, float* vector_to_normalize);

__global__ void squareVectorElements_kernel(float *vector);

__global__ void Image2Vector_kernel(cv::cuda::PtrStepSz<float> image, float* vector);

__global__ void UpdateDepthmap_kernel(Camera_gpu* camera, cv::cuda::PtrStepSz<int2> cost_volume, cv::cuda::PtrStepSz<float> weight_matrix,
                                  cv::cuda::PtrStepSz<float> a, cv::cuda::PtrStepSz<float> d, cv::cuda::PtrStepSz<float> gradient_q,
                                  int switch_idx, float switch_depth, float depth1_r, float depth2_r);

__global__ void PopulateState_kernel(Camera_gpu* camera);


class Mapper{

  public:
    Mapper(Environment_gpu* environment){};

    void addCamera(Camera_cpu* camera_cpu, Camera_gpu* camera_gpu);
    bool Regularize();
    double StudyCostVolumeMin(int index_m, int row, int col, bool showbaseline);
    void UpdateDepthmap();
    void PopulateState();
    void StateFromGt();
    void ComputeCostVolumeMin();
    void UpdateCostVolume(int index_m, bool occl );
    void depthSampling(Environment_gpu* environment);
    void Initialize();

    CameraVector_cpu camera_vector_cpu_;
    cv::cuda::GpuMat d;
    cv::cuda::GpuMat a;
    cv::cuda::GpuMat q;
    cv::cuda::GpuMat gradient_d;
    cv::cuda::GpuMat gradient_q;
    cv::cuda::GpuMat weight_matrix_;
    cv::cuda::GpuMat depth_groundtruth_;
    int index_r_;
    int frames_computed_=0;

  private:

    float* invdepth_r_array_;
    int n_;
    int switch_idx_;
    float switch_depth_;
    float theta_;
    float theta_end_;
    float eps_;
    float alpha_;
    float beta1_;
    float beta2_;
    float lambda_;
    int threshold_;
    float sigma_q0_;
    float sigma_d0_;
    float sigma_q_;
    float sigma_d_;
    float r1_;
    float r2_;
    float theta_switch_;
    cv::cuda::GpuMat cost_volume_;
    cv::cuda::GpuMat query_proj_matrix_;
    // cv::cuda::GpuMat variance_matrix_;
    cameraDataForDtam* camera_data_for_dtam_;
    Eigen::Matrix3f T_r;
    Eigen::Vector3f T_t;


    void prepareCameraForDtam(int index_m);
    void ComputeWeights();
    void UpdateParametersReg(bool trigger, bool print);
    void ComputeGradientSobelImage(cv::cuda::GpuMat* image_in, cv::cuda::GpuMat* image_out);
    void ComputeDivergenceSobelImage(cv::cuda::GpuMat* image_in, cv::cuda::GpuMat* image_out);
    void ComputeWeightedGradientSobelImage(cv::cuda::GpuMat* image_in, cv::cuda::GpuMat* image_out);
    void ComputeWeightedDivergenceSobelImage(cv::cuda::GpuMat* image_in, cv::cuda::GpuMat* image_out);
    void gradDesc_Q(cv::cuda::GpuMat* q, cv::cuda::GpuMat* gradient_d );
    void gradDesc_D(cv::cuda::GpuMat* d, cv::cuda::GpuMat* a, cv::cuda::GpuMat* gradient_q );
    void search_A(cv::cuda::GpuMat* d, cv::cuda::GpuMat* a );
    void getVectorNorm(float* vector_to_normalize, float* norm, int N);
    void getVectorMax(float* vector_to_normalize, float* max, int N);
    void getImageNorm(cv::cuda::GpuMat* image, float* norm);
    void Image2Vector(cv::cuda::GpuMat* image, float* vector);





};
