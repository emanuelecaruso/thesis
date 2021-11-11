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

    std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    if (frame_current_>= environment_->camera_vector_->size()){
      sharedCout("\nVideo stream ended");
      break;
    }
    sharedCout("\nFrame: "+ std::to_string(frame_current_));
    addCamera(takeGtPoses);
    if (frame_current_==2){
      first_2_frames_available_.notify_all();
    }
    frame_current_++;

    double t_end=getTime();
    locker.unlock();

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

void Dtam::testFeatures(){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  // float size=1;
  float size=1.3;
  // float size=2;

  for (int i=1; i<environment_->camera_vector_->size(); i++){
    showFeatures(i, size);
    cv::waitKey(0);
  }
};

void Dtam::showFeatures(int idx, float size=1){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  CameraForStudy* cam_2=environment_->camera_vector_->at(idx);
  cam_1->image_rgb_->showWithOtherImage(cam_2->image_rgb_,"image comparison",size);
  cam_1->curvature_->showWithOtherImage(cam_2->curvature_,"curvature comparison",size);
  // cam_1->grad_intensity_->showWithOtherImage(cam_2->grad_intensity_,"grad intensity comparison",size);
  // cam_1->grad_x_->showWithOtherImage(cam_2->grad_x_,"grad x comparison",size);
  // cam_1->grad_y_->showWithOtherImage(cam_2->grad_y_,"grad y comparison",size);
  cam_1->grad_robust_x_->showWithOtherImage(cam_2->grad_robust_x_,"grad x rob comparison",size);


};
