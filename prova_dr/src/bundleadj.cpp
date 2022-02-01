#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

CameraForMapping* BundleAdj::getFrameCurrentBA(){
  return dtam_->camera_vector_->at(frame_current_ba);
}

void BundleAdj::projectAndMarginalizeActivePoints(){

}


void BundleAdj::activateNewPoints(){

  frame_current_ba = dtam_->frame_current_; // update frame current

  sharedCoutDebug("   - activating point in frame "+std::to_string(frame_current_ba));

  double t_start=getTime();

  int num_activated = selectNewActivePoints();

  double t_end=getTime();
  int deltaTime=(t_end-t_start);

  sharedCoutDebug("   - Points activated in frame "+std::to_string(frame_current_ba)+": "+std::to_string(num_activated)+", number of active points: "+std::to_string(num_active_points_)+", time: "+std::to_string(deltaTime)+" ms");

  // DEBUG
  if(debug_optimization_){
    // cam on which active points are projected
    CameraForMapping* last_keyframe = getFrameCurrentBA();
    last_keyframe->showProjActivePoints(1);
    last_keyframe->showProjCandidates(1);
    cv::waitKey(0);
  }


}

bool BundleAdj::activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs){

  Candidate* cand = cand_proj->cand_;
  std::vector<Candidate*>* cands_vec = cand->region_sampling_->cands_vec_;

  int num_proj_active_points = regs->getNumOfActivePointsInReg( reg );
  if( num_proj_active_points<=min_num_of_active_pts_per_region_ ){
    // create active point from candidate
    ActivePoint* active_point = new ActivePoint(cand);

    // push active point
    active_point->cam_->active_points_->push_back(active_point);

    // create active point projected from projected candidate
    ActivePointProjected* active_point_proj = new ActivePointProjected(cand_proj, active_point) ;

    // push active point projected
    active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);

    // marginalize candidate
    cand->marginalize();
    reg->cands_proj_vec_->erase(reg->cands_proj_vec_->begin()); // remove projected active point from the region
    delete cand_proj; // delete projected candidate

    return true;
  }
  return false;
}

int BundleAdj::selectNewActivePoints(){

  CameraForMapping* last_keyframe=dtam_->getLastKeyframe();

  // compute min_num_of_active_pts_per_region_
  min_num_of_active_pts_per_region_=INT_MAX;
  RegionsWithProjActivePoints* regs = last_keyframe->regions_projected_active_points_;
  // find min number of active points in region
  for (RegionWithProjActivePoints* reg : *(regs->region_vec_)){
    if(reg->active_pts_proj_vec_->size()>0 && reg->active_pts_proj_vec_->size()<min_num_of_active_pts_per_region_){
      min_num_of_active_pts_per_region_=reg->active_pts_proj_vec_->size();
    }
  }
  if(min_num_of_active_pts_per_region_==INT_MAX){
    min_num_of_active_pts_per_region_=0;
  }

  // num of points to be activated
  int num_to_be_activated=parameters_->max_num_active_points- num_active_points_+num_points_to_marginalize_;
  int num_activated = 0;

  // RegionsWithProjCandidates* regions = last_keyframe->regions_projected_cands_;
  std::vector<RegionWithProjCandidates*>* reg_vec= last_keyframe->regions_projected_cands_->region_vec_;

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  int i = 0;
  int round = 0;
  while(num_activated<num_to_be_activated){
  // for (int i=0; i<num_to_be_activated; i++){


    if (reg_vec->empty())
      break;

    int reg_idx = i%reg_vec->size();
    RegionWithProjCandidates* reg = reg_vec->at(reg_idx);
    if (reg->cands_proj_vec_->empty()){
      reg_vec->erase(std::remove(reg_vec->begin(), reg_vec->end(), reg), reg_vec->end());
      i--;
      continue;
    }

    i++;

    CandidateProjected* cand_proj = reg->cands_proj_vec_->at(0);
    bool candidate_activated = activateCandidate(cand_proj,reg,regs);

    if (candidate_activated){
      // active point
      num_activated++;
      num_active_points_++;
    }

    // if it's the last round
    if(reg_idx==reg_vec->size()-1){
      min_num_of_active_pts_per_region_++;
    }
  }



  return num_activated;

}


void BundleAdj::optimize(){
  // optimize TODO

}

// void BundleAdj::projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe){
//   // iterate through all keyframe candidates
//
//   // project active point in new keyframe
//
//   // put it in the relative region
// }



void BundleAdj::sortRegions(){
  // sort region depending on how many active points there are
}
