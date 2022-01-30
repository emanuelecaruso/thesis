#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void BundleAdj::projectAndMarginalizeActivePoints(){

}


void BundleAdj::activateNewPoints(){

  frame_current_ba = dtam_->frame_current_;
  sharedCoutDebug("   - activating point in frame "+std::to_string(frame_current_ba));

  double t_start=getTime();

  int num_activated = selectNewActivePoints();

  double t_end=getTime();
  int deltaTime=(t_end-t_start);

  sharedCoutDebug("   - Points activated in frame "+std::to_string(frame_current_ba)+": "+std::to_string(num_activated)+", number of active points: "+std::to_string(num_active_points_)+", time: "+std::to_string(deltaTime)+" ms");



}

void BundleAdj::activateCandidate(CandidateProjected* cand_proj){
  // remove candidate from region of candidates

  const Candidate* cand = cand_proj->cand_;
  std::vector<Candidate*>* cands_vec = cand->region_sampling_->cands_vec_;
  CameraForMapping* cam = cand->region_sampling_->cam_;
  cands_vec->erase(remove(cands_vec->begin(), cands_vec->end(), cand));
  cam->candidates_->erase(remove(cam->candidates_->begin(), cam->candidates_->end(), cand));


  // create active point

  delete cand;
  delete cand_proj;
  // push active point
  // cam_->active_points_->push_back();
}

int BundleAdj::selectNewActivePoints(){

  // num of points to be activated
  int num_to_be_activated=parameters_->max_num_active_points- num_active_points_+num_points_to_marginalize_;
  int num_activated = 0;

  CameraForMapping* last_keyframe=dtam_->getLastKeyframe();
  // RegionsWithProjCandidates* regions = last_keyframe->regions_projected_cands_;
  std::vector<RegionWithProjCandidates*>* reg_vec= last_keyframe->regions_projected_cands_->region_vec_;

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  // while(num_activated<num_to_be_activated){
  for (int i=0; i<num_to_be_activated; i++){
    if (reg_vec->empty())
      break;

    RegionWithProjCandidates* reg = reg_vec->at(i%reg_vec->size());
    if (reg->cands_vec_->empty()){
      reg_vec->erase(std::remove(reg_vec->begin(), reg_vec->end(), reg), reg_vec->end());
      i--;
      continue;
    }

    // i++;

    CandidateProjected* cand_proj = reg->cands_vec_->at(0);
    activateCandidate(cand_proj);
    reg->cands_vec_->erase(reg->cands_vec_->begin());

    // active point
    num_activated++;
    num_active_points_++;
  }

  return num_activated;


  // // iterate through keyframes
  // for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){
  //
  //   int idx = dtam_->keyframe_vector_->at(i);
  //   CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
  //   CameraForMapping* new_keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  //
  //
  // }

}


void BundleAdj::optimize(){

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
