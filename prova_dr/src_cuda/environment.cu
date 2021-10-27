#include "environment.cuh"
// #include "json.hpp"
// #include <fstream>
#include "utils.h"
// #include <sys/types.h>
// #include <sys/stat.h>
// using json = nlohmann::json;


void Environment_gpu::generateCamera(std::string name, float t1, float t2, float t3, float alpha1, float alpha2, float alpha3){
  // Eigen::Vector3f t_r(t1,t2,t3);
  // Eigen::Isometry3f* frame_world_wrt_camera_r = new Eigen::Isometry3f;
  // frame_world_wrt_camera_r->linear().setIdentity();  //TODO implement orientation
  // frame_world_wrt_camera_r->translation()=t_r;
  // Eigen::Isometry3f* frame_camera_wrt_world_r = new Eigen::Isometry3f;
  // *frame_camera_wrt_world_r = frame_world_wrt_camera_r->inverse();
  // Camera_cpu* camera = new Camera_cpu(name,lens_,aspect_,film_,resolution_,max_depth_,min_depth_,frame_camera_wrt_world_r,frame_world_wrt_camera_r);
  // camera_vector_cpu_.push_back(camera);
  //
  // Camera_gpu* camera_d = camera->getCamera_gpu();
  // camera_vector_gpu_.push_back(camera_d);

}

bool Environment_gpu::loadEnvironment_gpu(std::string path_name, std::string dataset_name){

    std::cout << "loading dataset at: " << path_name << std::endl;

    camera_vector_cpu_.clear();
    loadEnvironment(path_name, dataset_name);

    for( Camera* camera : camera_vector_){
      Camera_cpu* camera_cpu = new Camera_cpu(
        camera->name_,
        camera->lens_,
        camera->aspect_,
        camera->width_,
        camera->resolution_,
        camera->max_depth_,
        camera->min_depth_,
        camera->frame_camera_wrt_world_,
        camera->frame_world_wrt_camera_,
        camera->frame_camera_wrt_world_gt_,
        camera->frame_world_wrt_camera_gt_
      );
      camera_cpu->cloneCameraImages(camera);
      camera_cpu->getCamera_gpu();

      camera_vector_cpu_.push_back(camera_cpu);

    }
    camera_vector_.clear();

    std::cout << "dataset loaded successfully" << std::endl;


  // }

}
