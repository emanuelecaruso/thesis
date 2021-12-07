#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void BundleAdj::projectAndMarginalizeActivePoints(){

}

void BundleAdj::activateNewPoints(bool active_all_candidates){
  if(dtam_->frame_current_<=0)
    dtam_->waitForNewFrame();

  if(frame_current_ba>=dtam_->frame_current_)
    dtam_->waitForTrackedCandidates();

  if(active_all_candidates){
    activateNewPointsAllCand();
    sharedCoutDebug("   - New points activated (all points)");
  }else{
    activateNewPoints();
    sharedCoutDebug("   - New points activated ");
  }
  frame_current_ba++;
}

void BundleAdj::activateNewPoints(){
  // iterate through keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
    CameraForMapping* new_keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
    // project active points in new keyframe
    projectActivePoints(keyframe, new_keyframe);
    projectCandidates(keyframe, new_keyframe);

  }
  sortRegions();
  selectNewActivePoints();
  // select new active points

}

void BundleAdj::selectNewActivePointsAll(){
  // iterate through sorted regions

}

void BundleAdj::activateNewPointsAllCand(){
  // iterate through keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
    CameraForMapping* new_keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
    // project active points in new keyframe
    projectCandidates(keyframe, new_keyframe);
  }

}

void BundleAdj::optimize(){

}

void BundleAdj::projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe){
  // iterate through all keyframe candidates

  // project active point in new keyframe

  // put it in the relative region
}


void BundleAdj::projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe){
  // iterate through all candidates

    // project candidates in new keyframe

    // put it in the relative region, sorted by depth var
}

void BundleAdj::sortRegions(){
  // sort region depending on how many active points there are
}


void BundleAdj::selectNewActivePoints(){
  // iterate through sorted regions

}
