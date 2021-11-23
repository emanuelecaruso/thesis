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

void Dtam::addCamera(){

  CameraForStudy* env_cam=environment_->camera_vector_->at(frame_current_);
  CameraForMapping* new_cam=new CameraForMapping (env_cam, wavelet_levels_);

  camera_vector_->push_back(new_cam);
}

int Dtam::getFrameCurrent(){
  return frame_current_;
}

void Dtam::waitForNewFrame(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  frame_updated_.wait(locker);
  locker.unlock();
}


void Dtam::doInitialization(bool all_keyframes, bool takeGtPoses){
  while(true){

    waitForNewFrame();

    double t_start=getTime();

    tracker_->trackCam(takeGtPoses);
    if(keyframe_handler_->addKeyframe(all_keyframes)){
      bundle_adj_->projectAndMarginalizeActivePoints();
      mapper_->trackExistingCandidates();
      mapper_->selectNewCandidates();
      bundle_adj_->activateNewPoints();
      bundle_adj_->optimize();
    }
    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("Initialization time: "+ std::to_string(deltaTime)+" ms");
  }


}

void Dtam::doMapping(){
  mapper_->doMapping();
}

void Dtam::doTracking(){

}

void Dtam::updateCamerasFromVideostream(){

  float fps=environment_->fps_;

  while(frame_current_< environment_->camera_vector_->size()){

    std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    sharedCout("\nFrame: "+ std::to_string(frame_current_));
    addCamera();

    frame_current_++;
    frame_updated_.notify_one();

    double t_end=getTime();
    locker.unlock();

    int deltaTime=(t_end-t_start);
    sharedCoutDebug("Add camera computation time: "+ std::to_string(deltaTime)+" ms");
    long int waitDelay=deltaTime*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    if(time_to_wait>0)
      std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
  }
  sharedCout("\nVideo stream ended");

}

void Dtam::test_mapping(){

  bool takeGtPoses=true;
  bool allKeyframes=true;

  std::thread update_cameras_thread_(&Dtam::updateCamerasFromVideostream, this);
  // std::thread track
  std::thread initialization_thread(&Dtam::doInitialization, this, allKeyframes, takeGtPoses);
  // std::thread mapping_thread_(&Dtam::doMapping, this);


  initialization_thread.detach();
  // mapping_thread_.detach();
  update_cameras_thread_.join();

  // debugAllCameras();

  // camera_vector_->at(keyframe_vector_->at(0))->showCandidates_1(2);
  // cv::waitKey(0);


}

void Dtam::testFeatures(){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  float size=1;
  // float size=1.3;image_
  // float size=2;

  for (int i=1; i<environment_->camera_vector_->size(); i++){
    showFeatures(i, size);
    cv::waitKey(0);
  }
};

void Dtam::showFeatures(int idx, float size=1){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  CameraForStudy* cam_2=environment_->camera_vector_->at(idx);
  // cam_1->image_rgb_->showWithOtherImage(cam_2->image_rgb_,"image comparison",size);
  cam_1->wavelet_dec_->showWaveletDec(size);
  // cam_1->wavelet_dec_->compareThreshold(0.1,size);
  // cam_1->curvature_->showWithOtherImage(cam_2->curvature_,"curvature comparison",size);
  // cam_1->grad_intensity_->showWithOtherImage(cam_2->grad_intensity_,"grad intensity comparison",size);
  // cam_1->grad_x_->showWithOtherImage(cam_2->grad_x_,"grad x comparison",size);
  // cam_1->grad_x_->getChannel(2)->showWithOtherImage(cam_2->grad_x_->getChannel(2),"r grad x comparison",size);
  // cam_1->grad_x_->getChannel(1)->showWithOtherImage(cam_2->grad_x_->getChannel(1),"g grad x comparison",size);
  // cam_1->grad_x_->getChannel(0)->showWithOtherImage(cam_2->grad_x_->getChannel(0),"b grad x comparison",size);
  // cam_1->grad_y_->showWithOtherImage(cam_2->grad_y_,"grad y comparison",size);
  // cam_1->grad_robust_x_->showWithOtherImage(cam_2->grad_robust_x_,"grad x rob comparison",size);


};
