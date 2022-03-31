#include "keyframe.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>


bool KeyframeHandler::addKeyframe(bool all_keyframes){
  bool frame_added;


  if(all_keyframes){
    frame_added=pushKF();
  }else{
    frame_added=addKeyframe_select();
  }
  marginalize_keyframe(all_keyframes);

  return frame_added;
}

void KeyframeHandler::addFirstKeyframe(){
  dtam_->camera_vector_->at(dtam_->frame_current_)->first_keyframe_=true;
  pushKeyframeFrontend();
  // pushKeyframeBundleadj();
  sharedCoutDebug("   - First keyframe added ( "+ dtam_->camera_vector_->at(dtam_->frame_current_)->name_ +" )");
}

bool KeyframeHandler::pushKF(){
  pushKeyframeFrontend();
  // pushKeyframeBundleadj();
  sharedCoutDebug("   - Keyframe added ( "+ dtam_->camera_vector_->at(dtam_->frame_current_)->name_ +" )");
  return true;
}

bool KeyframeHandler::addKeyframe_select(){
  // compute optical flow distance
  float flow_dist = getFlowDist();
  // decide wether take keyframe or not
  // std::cout << "FLOW DIST " << flow_dist << "\n";
  bool take_kf = (flow_dist>dtam_->parameters_->flow_dist_threshold);

  if(take_kf){
    pushKF();
    return true;
  }

  return false;
}

float KeyframeHandler::getFlowDist(){
  // project active points in last two frames
  CameraForMapping* curr_frame = dtam_->camera_vector_->at(dtam_->frame_current_);
  CameraForMapping* last_kf = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());

  float sum_dist = 0;
  int num_dists = 0;
  // iterate through all keyframe (except the last)
  for (int i=0; i<dtam_->keyframe_vector_->size()-1; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(i));
    CamCouple* cam_couple_curr_frame = new CamCouple(keyframe,curr_frame);
    CamCouple* cam_couple_last_kf = new CamCouple(keyframe,last_kf);
    // iterate along all active points
    for (ActivePoint* active_pt : *keyframe->active_points_){

      if(active_pt->to_marginalize_){
        continue;
      }

      // project active point in new keyframe
      ActivePointProjected* active_point_proj_curr_frame = dtam_->bundle_adj_->projectActivePoint(active_pt, cam_couple_curr_frame);
      if (active_point_proj_curr_frame==nullptr)
        continue;
      ActivePointProjected* active_point_proj_last_kf = dtam_->bundle_adj_->projectActivePoint(active_pt, cam_couple_last_kf);
      if (active_point_proj_last_kf==nullptr)
        continue;

      float dist = getEuclideanDistance(active_point_proj_curr_frame->uv_, active_point_proj_last_kf->uv_);
      sum_dist+=dist;
      num_dists++;
    }

    delete cam_couple_curr_frame;
    delete cam_couple_last_kf;
  }
  float avg_dist = sum_dist/num_dists;

  return avg_dist;

}


bool KeyframeHandler::marginalize_keyframe(bool all_keyframes){
  bool marginalize=false;
  if(dtam_->keyframe_vector_->size()>dtam_->parameters_->num_active_keyframes){
    marginalize=true;
    if (all_keyframes){
      marginalizeKeyframe(0);
    }else{
      marginalizeKeyframeSelect();
    }

  }
  return marginalize;
}

void KeyframeHandler::marginalizeKeyframeFrontend(int idx){
  dtam_->camera_vector_->at(idx)->to_be_marginalized_=true;

  // v->erase(std::remove(v->begin(), v->end(), idx), v->end());
}

void KeyframeHandler::marginalizeKeyframeBundleadj(int idx){
  dtam_->camera_vector_->at(idx)->to_be_marginalized_ba_=true;

}


void KeyframeHandler::marginalizeKeyframe(int idx_){
  int idx = dtam_->keyframe_vector_->at(idx_);
  sharedCoutDebug("   - Keyframe marginalized (frame "+ dtam_->camera_vector_->at(idx)->name_ +")");
  marginalizeKeyframeFrontend(idx);
  // marginalizeKeyframeBundleadj(idx);


}

void KeyframeHandler::marginalizeKeyframeSelect(){

  bool percentage_marginalization = false;
  // iterate through all keyframe (except the last two)
  for (int i=0; i<dtam_->keyframe_vector_->size()-2; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(i));

    float percentage_marg = getPercentuageMarg(keyframe);
    if(percentage_marg<dtam_->parameters_->percentage_marg_pts_threshold){
      marginalizeKeyframe(i);
      break;
      percentage_marginalization=true;
    }

  }

  if(!percentage_marginalization){

    int max_score_idx = -1;
    float max_score = 0;
    // iterate through all keyframe (except the last two)
    for (int i=0; i<dtam_->keyframe_vector_->size()-2; i++){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(i));

      float score = getScore(keyframe);
      if (score>max_score){
        max_score=score;
        max_score_idx=i;
      }
    }
    marginalizeKeyframe(max_score_idx);


  }

}

float KeyframeHandler::getPercentuageMarg(CameraForMapping* keyframe){
  int num_active_pts = keyframe->active_points_->size();
  int num_non_active_pts = keyframe->marginalized_points_->size();

  return (((float)num_active_pts)/((float)(num_active_pts+num_non_active_pts)));
  // return 0;
}

float KeyframeHandler::getScore(CameraForMapping* keyframe){

  CameraForMapping* last_kf = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());

  Eigen::Vector3f pos_diff_last_kf = (keyframe->frame_camera_wrt_world_->translation())-(last_kf->frame_camera_wrt_world_->translation());
  float d1 = sqrt(pos_diff_last_kf.norm());
  float sum = 0;
  // iterate through all keyframe (except the last two)
  for (int i=0; i<dtam_->keyframe_vector_->size()-2; i++){
    CameraForMapping* keyframe_ = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(i));

    if(keyframe_==keyframe)
      continue;

    Eigen::Vector3f pos_diff = (keyframe->frame_camera_wrt_world_->translation())-(keyframe_->frame_camera_wrt_world_->translation());
    sum += 1.0/(pos_diff.norm()+0.00000001);

  }
  return d1*sum;


}

void KeyframeHandler::pushKeyframeFrontend(){
  dtam_->keyframe_vector_->push_back(dtam_->frame_current_);
  dtam_->camera_vector_->at(dtam_->frame_current_)->keyframe_=true;
  dtam_->camera_vector_->at(dtam_->frame_current_)->new_=true;

}

void KeyframeHandler::pushKeyframeBundleadj(){
  dtam_->bundle_adj_->addKeyframe(dtam_->frame_current_);
}

void KeyframeHandler::prepareDataForBA(){
  // iterate through all keyframe
  for (int i=0; i<dtam_->keyframe_vector_->size(); i++){
    int idx = dtam_->keyframe_vector_->at(i);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
    if(keyframe->to_be_marginalized_){
      keyframe->to_be_marginalized_ba_=true;
      std::vector<int>* v = dtam_->keyframe_vector_;
      v->erase(std::remove(v->begin(), v->end(), idx), v->end());
    }
  }
  pushKeyframeBundleadj();
}
