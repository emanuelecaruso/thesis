#pragma once
#include "defs.h"
#include "environment.h"
#include "camera_cpu.cuh"
#include "camera_gpu.cuh"


class Environment_gpu : public Environment{
  public:
    // // cameras parameters
    // int resolution_;
    // float film_;
    // float lens_;
    // float aspect_;
    // float max_depth_;
    // // environment state
    // cpVector cp_vector_;

    CameraVector_cpu camera_vector_cpu_; // vector containing pointers to camera objects


    Environment_gpu(int resolution = 640, float aspect = 1, float film = 0.024,
                            float lens = 0.035, float max_depth =20)

    : Environment(resolution, aspect, film, lens, max_depth ) {}

    // void generateSinusoidalSurface(float picks_depth, int density);
    // void generateTexturedPlane(std::string path, float size, Eigen::Isometry3f pose, int density);
    // void generateTexturedCube(float size, Eigen::Isometry3f pose, int density);

    void generateCamera(std::string name, float t1, float t2, float t3, float alpha1, float alpha2, float alpha3);
    bool loadEnvironment_gpu(std::string path_name, std::string dataset_name);

};
