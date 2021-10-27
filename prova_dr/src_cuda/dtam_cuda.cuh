#pragma once
#include "camera_cpu.cuh"
#include "camera_gpu.cuh"
#include "image.h"
#include <cuda_runtime.h>
#include "environment.cuh"
#include "mapper.cuh"
#include "tracker.cuh"


class Dtam{

  public:
    Dtam(Environment_gpu* environment){
      mapper_ = new Mapper(environment);
      tracker_ = new Tracker(environment);
    };

    void test_mapping(Environment_gpu* environment);
    void test_tracking(Environment_gpu* environment);

  private:
    CameraVector_cpu camera_vector_cpu_;
    Mapper* mapper_;
    Tracker* tracker_;
    int index_r_;
    float* invdepth_r_array_;

    double showImgs(int scale);
    void addCamera(Camera_cpu* camera_cpu);
    bool setReferenceCamera(int index_r);


};
