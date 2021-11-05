#include "mapper.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>



void Dtam::debugAllCameras(bool show_imgs){

  sharedCoutDebug("DEBUGGING ALL CAMERAS:");
  sharedCoutDebug("camera vector size: " + std::to_string(camera_vector_->size()));

  for(Camera* camera : *camera_vector_){
    camera->printMembers();
    if (show_imgs){
      camera->showRGB();
      camera->showDepthMap();
    }
  }
  cv::waitKey(0);
}

void Dtam::addCamera(bool takeGtPoses){
  Camera* new_camera=environment_->camera_vector_->at(frame_current_)->clone();
  if(!takeGtPoses){
    new_camera->frame_camera_wrt_world_->linear().setIdentity();
    new_camera->frame_camera_wrt_world_->translation()= Eigen::Vector3f(0,0,0);
    new_camera->frame_world_wrt_camera_->linear().setIdentity();
    new_camera->frame_world_wrt_camera_->translation()= Eigen::Vector3f(0,0,0);
  }
  camera_vector_->push_back(new_camera);
}

int Dtam::getFrameCurrent(){
  return frame_current_;
}

void Dtam::doMapping(){
  mapper_->doMapping();
}

void Dtam::doTracking(){

}

void Dtam::updateCamerasFromVideostream(bool takeGtPoses){

  float fps=environment_->fps_;

  while(true){


    double t_start=getTime();

    if (frame_current_>= environment_->camera_vector_->size()){
      sharedCout("\nVideo stream ended");
      break;
    }
    sharedCout("\nFrame: "+ std::to_string(frame_current_));
    addCamera(takeGtPoses);
    std::unique_lock<std::mutex> locker(mu_frame_);
    if (frame_current_==2){
      first_2_frames_available_.notify_all();
    }
    frame_current_++;
    locker.unlock();

    double t_end=getTime();

    long int waitDelay=(t_end-t_start)*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
  }

}

void Dtam::test_mapping(){

  bool takeGtPoses=true;

  std::thread update_cameras_thread_(&Dtam::updateCamerasFromVideostream, this, takeGtPoses);
  std::thread mapping_thread_(&Dtam::doMapping, this);

  // mapping_thread_.join();
  mapping_thread_.detach();
  update_cameras_thread_.join();

}
