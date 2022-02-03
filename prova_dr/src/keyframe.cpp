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
    frame_added=addKeyframe_all();
  }else{
    frame_added=addKeyframe_select();
  }
  marginalize_keyframe(all_keyframes);

  return frame_added;
}

bool KeyframeHandler::addKeyframe_all(){
  pushKeyframeFrontend();
  pushKeyframeBundleadj();
  sharedCoutDebug("   - Keyframe added (frame "+ std::to_string(dtam_->frame_current_) +")");
  return true;
}

bool KeyframeHandler::addKeyframe_select(){
  // select keyframe TODO
  return false;
}

bool KeyframeHandler::marginalize_keyframe(bool all_keyframes){
  bool marginalize=false;
  if(dtam_->keyframe_vector_->size()>num_active_keyframes_){
    marginalize=true;
    if (all_keyframes){
      marginalizeKeyframeAll();
    }else{
      marginalizeKeyframeSelect();
    }

  }
  return marginalize;
}

void KeyframeHandler::marginalizeKeyframeFrontend(int idx){
  std::vector<int>* v = dtam_->keyframe_vector_;
  v->erase(std::remove(v->begin(), v->end(), idx), v->end());
}

void KeyframeHandler::marginalizeKeyframeBundleadj(int idx){
  dtam_->camera_vector_->at(idx)->to_be_marginalized_ba_=true;

}


void KeyframeHandler::marginalizeKeyframeAll(){
  int idx = dtam_->keyframe_vector_->at(0);
  marginalizeKeyframeFrontend(idx);
  marginalizeKeyframeBundleadj(idx);

  sharedCoutDebug("   - Keyframe marginalized (frame "+ std::to_string(dtam_->keyframe_vector_->at(0)) +")");

}

void KeyframeHandler::marginalizeKeyframeSelect(){
  // select keyframe TODO
}



void KeyframeHandler::pushKeyframeFrontend(){
  dtam_->keyframe_vector_->push_back(dtam_->frame_current_);
}

void KeyframeHandler::pushKeyframeBundleadj(){
  dtam_->bundle_adj_->addKeyframe(dtam_->frame_current_);
  dtam_->camera_vector_->at(dtam_->frame_current_)->added_ba_=true;

}
