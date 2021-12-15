#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void BundleAdj::projectAndMarginalizeActivePoints(){

}

void BundleAdj::activateNewPoints(){
  if(dtam_->frame_current_<=0)
    dtam_->waitForNewFrame();

  if(frame_current_ba>=dtam_->frame_current_)
    dtam_->waitForTrackedCandidates();

  selectNewActivePoints();
  sharedCoutDebug("   - New points activated ");

  frame_current_ba++;
}

void BundleAdj::activateCandidate(const Candidate* cand){
  // remove candidate from region of candidates
  // remove(cand->region_->cands_vec_->begin(), cand->region_->cands_vec_->end(), cand);
  // create active point

  // push active point
  // cam_->active_points_->push_back();
}

void BundleAdj::selectNewActivePoints(){

  // num of points to be activated
  int num_to_be_activated=parameters_->max_num_active_points- num_active_points_;

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  // RegionsWithProjCandidates* regions = last_keyframe->regions_projected_cands_;
  std::vector<RegionWithProjCandidates*>* reg_vec= last_keyframe->regions_projected_cands_->region_vec_;

  for (int i=0; i<num_to_be_activated; i++){
    RegionWithProjCandidates* reg = reg_vec->at(i%reg_vec->size());
    const Candidate* cand ;
    if (!reg->cands_vec_->empty())
      cand = reg_vec->at(i%reg_vec->size())->cands_vec_->at(0)->cand_;

    activateCandidate(cand);

    // active point

    num_active_points_++;
  }


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
