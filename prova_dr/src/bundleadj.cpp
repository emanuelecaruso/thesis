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


void BundleAdj::selectNewActivePoints(){
  // num of points to be activated
  int num_to_be_activated=parameters_->num_active_points- active_points_vec_->size();

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  RegionsWithProjCandidates* regions = last_keyframe->regions_projected_cands_;
  // iterate through all regions of projected cands
  for(RegionWithProjCandidates* reg : *(regions->region_vec_)){
    // take first candidate (with min depth variance)
    // CandidateProjected* cand : *(reg->cands_vec_)

  }

      //



  // iterate through keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
    CameraForMapping* new_keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());


  }

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
