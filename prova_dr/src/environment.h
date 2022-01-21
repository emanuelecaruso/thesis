#pragma once
#include "camera.h"

class Environment{
  public:

    const std::string dataset_name_;
    const int fps_;
    const int coarsest_level_;
    const CamParameters* cam_parameters_;
    const std::vector<Camera*>* camera_vector_; // vector containing pointers to camera objects


    Environment(const std::string& path_name, const std::string& dataset_name, Params* parameters):
    dataset_name_(dataset_name),
    fps_(parameters->fps),
    coarsest_level_(parameters->coarsest_level),
    cam_parameters_(loadCamParameters( path_name, dataset_name)),
    camera_vector_(loadCameraVector( path_name, dataset_name, parameters->end_frame))
    { };

    void debugAllCameras(bool show_imgs=false) const;

  private:
    std::vector<Camera*>* loadCameraVector(const std::string& path_name, const std::string& dataset_name, int end_frame);
    CamParameters* loadCamParameters(const std::string& path_name, const std::string& dataset_name);

    // double saveState(std::string path_name, Camera_cpu* camera_cpu);

};
