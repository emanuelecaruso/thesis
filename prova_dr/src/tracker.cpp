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

void Tracker::trackCam(bool takeGtPoses, bool const_acc){
  if(takeGtPoses){
    trackGroundtruth();
  }
  else{
    computeInitialGuess( const_acc );
  }
}

void Tracker::computeInitialGuess(bool const_acc ){
  Eigen::Isometry3f pose_initial_guess;
  if(const_acc){
    pose_initial_guess = accelerationConstantModel();
  }else{
    pose_initial_guess = velocityConstantModel();
  }
}

Eigen::Isometry3f Tracker::accelerationConstantModel(){
  Eigen::Isometry3f pose_initial_guess;

  CameraForMapping* current_cam = dtam_->getCurrentCamera();
  CameraForMapping* prev_cam = dtam_->getPreviousCamera();

  return pose_initial_guess;
}

Eigen::Isometry3f Tracker::velocityConstantModel(){
  Eigen::Isometry3f pose_initial_guess;

  CameraForMapping* current_cam = dtam_->getCurrentCamera();
  CameraForMapping* prev_cam = dtam_->getPreviousCamera();



  return pose_initial_guess;
}
