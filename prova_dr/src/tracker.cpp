#include "dtam.h"
#include "tracker.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void Tracker::trackGroundtruth(){
  const std::vector<Camera*>* cam_vec_env = dtam_->environment_->camera_vector_;

  Camera* cam_gt = cam_vec_env->at(dtam_->frame_current_);
  CameraForMapping* cam = dtam_->camera_vector_->at(dtam_->frame_current_);

  cam->frame_world_wrt_camera_=cam_gt->frame_world_wrt_camera_;
  cam->frame_camera_wrt_world_=cam_gt->frame_camera_wrt_world_;

  sharedCoutDebug("   - Frame tracked (groundtruth)");
}

void Tracker::trackLS(bool const_acc){
  Eigen::Isometry3f initial_guess = computeInitialGuess( const_acc );

  doLS(initial_guess);

  Camera* cam_curr_gt = dtam_->environment_->camera_vector_->at(dtam_->frame_current_);
  Camera* cam_prev_gt = dtam_->environment_->camera_vector_->at(dtam_->frame_current_-1);
  Eigen::Isometry3f gt = *(cam_curr_gt->frame_world_wrt_camera_)*(*(cam_prev_gt->frame_camera_wrt_world_));
  std::cout << gt.translation() << "\n and \n " << initial_guess.translation() << std::endl;

}

void Tracker::doLS(Eigen::Isometry3f& initial_guess){


  int n_observations = 0 ;
  while(true){
    for(int i=0; i<n_observations; i++){

    }
  }
}

void Tracker::trackCam(bool takeGtPoses, bool const_acc){
  if(takeGtPoses){
    trackGroundtruth();
  }
  else{
    trackLS(const_acc);
  }
}

Eigen::Isometry3f Tracker::computeInitialGuess(bool const_acc ){
  Eigen::Isometry3f pose_initial_guess;
  if(const_acc){
    pose_initial_guess = accelerationConstantModel();
  }else{
    pose_initial_guess = velocityConstantModel();
  }

  return pose_initial_guess;
}

Eigen::Isometry3f Tracker::accelerationConstantModel(){
  Eigen::Isometry3f pose_initial_guess;

  std::cout << "ACCELLERATION CONSTANT MODEL IS MISSING !!!" << std::endl;

  return pose_initial_guess;
}

Eigen::Isometry3f Tracker::velocityConstantModel(){

  CameraForMapping* last_cam = dtam_->getLastCamera();
  CameraForMapping* prev_cam = dtam_->getSecondLastCamera();

  Eigen::Isometry3f* last_T_w = last_cam->frame_world_wrt_camera_;
  Eigen::Isometry3f* w_T_prev = prev_cam->frame_camera_wrt_world_;

  Eigen::Isometry3f last_T_prev = (*last_T_w)*(*w_T_prev);

  return last_T_prev;
}
