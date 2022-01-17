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

void Tracker::trackLS(bool track_candidates){
  CameraForMapping* last_cam = dtam_->getLastCamera();
  CameraForMapping* curr_cam = dtam_->getCurrentCamera();

  Eigen::Isometry3f initial_guess = computeInitialGuess( );

  Eigen::Isometry3f final_guess = doLS(initial_guess, track_candidates);

  // final guess = curr_T_last -> pose = w_T_last * last_T_curr
  Eigen::Isometry3f frame_camera_wrt_world_ = (*last_cam->frame_camera_wrt_world_)*final_guess.inverse();

  curr_cam->assignPose(frame_camera_wrt_world_);

  Camera* cam_curr_gt = dtam_->environment_->camera_vector_->at(dtam_->frame_current_);
  Camera* cam_lastkf_gt = dtam_->environment_->camera_vector_->at(dtam_->keyframe_vector_->back());
  Eigen::Isometry3f gt = *(cam_curr_gt->frame_world_wrt_camera_)*(*(cam_lastkf_gt->frame_camera_wrt_world_));
  std::cout << gt.translation() << "\n and \n " << initial_guess.translation() << std::endl;


}

Eigen::Isometry3f Tracker::doLS(Eigen::Isometry3f& initial_guess, bool track_candidates){

  // get last keyframe
  CameraForMapping* last_keyframe = dtam_->getLastKeyframe();
  if(track_candidates){
    // for each level of the pyramid

      // for each pixel at coarser level containing at least 1 candidate

        // intensity of that pixel
        // compute invdepth as average of invdepths with invdepth var as weight
        // compute invdepth variance

    // std::vector<RegionWithProjCandidates*>* last_keyframe->regions_projected_cands_

  }

  int n_observations = 0 ;
  // while(true){
  //   for(int i=0; i<n_observations; i++){
  //
  //   }
  // }

  return initial_guess;
}

void Tracker::trackCam(bool takeGtPoses, bool track_candidates){
  if(takeGtPoses){
    trackGroundtruth();
  }
  else{
    trackLS(track_candidates);
  }
}

Eigen::Isometry3f Tracker::computeInitialGuess( ){
  Eigen::Isometry3f pose_initial_guess;

  pose_initial_guess = velocityConstantModel();


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
  CameraForMapping* lastkeyframe_cam = dtam_->getLastKeyframe();

  Eigen::Isometry3f* last_T_w = last_cam->frame_world_wrt_camera_;
  Eigen::Isometry3f* w_T_prev = prev_cam->frame_camera_wrt_world_;

  Eigen::Isometry3f last_T_prev = (*last_T_w)*(*w_T_prev);

  Eigen::Isometry3f* w_T_lastkeyframe = lastkeyframe_cam->frame_camera_wrt_world_;

  // we assume that the relative pose between last and previous frame
  // is the same between current and last (tracked) frame (constant velocity model)
  // so last_T_prev = curr_T_last

  // the initial guess is the pose of the last keyframe wrt current camera
  // so iitial_guess = curr_T_lastkeyframe = curr_T_last*last_T_lastkeyframe
  // so curr_T_lastkeyframe = curr_T_last*last_T_w*w_T_lastkeyframe
  // since curr_T_last = last_T_prev -> curr_T_lastkeyframe = last_T_prev*last_T_w*w_T_lastkeyframe
  Eigen::Isometry3f curr_T_lastkeyframe =  (last_T_prev*(*last_T_w))*(*w_T_lastkeyframe);

  return curr_T_lastkeyframe;
}
