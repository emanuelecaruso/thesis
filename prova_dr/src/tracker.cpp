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

void Tracker::collectCoarseRegions(CameraForMapping* keyframe){
  // std::vector<RegionsWithCandidates*>* coarse_regions_vec = new std::vector<RegionsWithCandidates*> ;
  //
  // // collect regions at coarser levels
  //
  // // iterate along all coarser levels
  // for(int i=1; i<dtam_->parameters_->coarsest_level; i++){
  //   // create empty regions
  //   RegionsWithCandidates* coarse_regions = new RegionsWithCandidates(dtam_->parameters_,keyframe,i);
  //   coarse_regions_vec->push_back(coarse_regions);
  //   // for each candidate of keyframe
  //   for(Candidate* cand : *(keyframe->candidates_)){
  //     // if level of candidate is less than current coarse level
  //     // and if candidate has one min
  //     if(cand->level_<i && cand->one_min_){
  //       int level_diff = i-cand->level_;
  //       // from pixel of candidate find pixel at level i
  //       int coarse_pxl_x = cand->pixel_.x()/(pow(2,level_diff));
  //       int coarse_pxl_y = cand->pixel_.y()/(pow(2,level_diff));
  //       int idx = coarse_regions->xyToIdx(coarse_pxl_x,coarse_pxl_y);
  //       // push candidate inside region
  //       coarse_regions->region_vec_->at(idx)->cands_vec_->push_back(cand);
  //       // save coarse region inside candidate
  //       cand->regions_coarse_->push_back(coarse_regions->region_vec_->at(idx));
  //     }
  //   }
  // }
}

void Tracker::collectCandidatesAtLowerResolution(CameraForMapping* keyframe){

  // // create candidates at coarser levels
  //
  // // iterate along all coarser levels
  // for(int i=1; i<dtam_->parameters_->coarsest_level; i++){
  //   RegionsWithCandidates* coarse_regions = coarse_regions_vec->at(i-1);
  //   // iterate along all regions
  //   for ( RegionWithCandidates* reg : *(coarse_regions->region_vec_)){
  //     // if region is not empty
  //     if(!reg->cands_vec_->empty()){
  //
  //       // intensity
  //       Eigen::Vector2i pixel {reg->x_, reg->y_};
  //       Eigen::Vector2f uv;
  //       keyframe->pixelCoords2uv(pixel,uv, i);
  //
  //       pixelIntensity intensity = keyframe->wavelet_dec_->getWavLevel(i)->c->evalPixel(pixel);
  //       // grad magnitude
  //       pixelIntensity grad_magnitude = keyframe->wavelet_dec_->getWavLevel(i)->magnitude_img->evalPixel(reg->y_,reg->x_);
  //
  //
  //       // iterate along collected candidates
  //       float invdepth, invdepth_var;
  //       float d_over_v_sum = 0, inv_v_sum = 0;
  //       int num_cands = reg->cands_vec_->size();
  //       for(Candidate* cand : *(reg->cands_vec_)){
  //         // update d_over_v_sum
  //         d_over_v_sum+= cand->invdepth_/cand->invdepth_var_;
  //         // update inv_v_sum
  //         inv_v_sum+= 1./cand->invdepth_var_;
  //
  //       }
  //       // compute invdepth as weighted average of invdepths with invdepth certainty as weight
  //       invdepth = d_over_v_sum/inv_v_sum;
  //       // compute invdepth variance as average of variances
  //       invdepth_var = num_cands/inv_v_sum;
  //
  //       // create coarse candidate
  //       Candidate* candidate_coarse = new Candidate(i,pixel, uv, grad_magnitude,intensity,invdepth,invdepth_var);
  //       // push candidate inside candidates
  //     }
  //   }
  // }
}

Eigen::Isometry3f Tracker::doLS(Eigen::Isometry3f& initial_guess, bool track_candidates){

  // get last keyframe
  CameraForMapping* last_keyframe = dtam_->getLastKeyframe();

  // prepare candidates at lower resolutions
  // prepareCandidatesAtLowerResolutions(last_keyframe);

  if(track_candidates){
    // for each level of the pyramid

      // while chi square is not converged

        // for each pixel at coarser level containing at least 1 candidate



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
