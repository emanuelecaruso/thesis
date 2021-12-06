#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void BundleAdj::projectAndMarginalizeActivePoints(){

}

void BundleAdj::activateNewPoints(){
  // iterate through keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);
    // project active points in new keyframe
  }
  // select new active points



}

void BundleAdj::optimize(){

}
