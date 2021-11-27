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
  push_keyframe();
  return true;
}

bool KeyframeHandler::addKeyframe_select(){
  // select keyframe TODO
  return true;
}

bool KeyframeHandler::marginalize_keyframe(bool all_keyframes){
  bool marginalize=false;
  if(dtam_->keyframe_vector_->size()>num_active_keyframes_){
    marginalize=true;

    if (all_keyframes){
      marginalize_keyframe_all();
    }else{
      marginalize_keyframe_select();
    }

  }
  return marginalize;
}

void KeyframeHandler::marginalize_keyframe_all(){
  dtam_->keyframe_vector_->erase(dtam_->keyframe_vector_->begin());
}

void KeyframeHandler::marginalize_keyframe_select(){
  // select keyframe TODO
}



void KeyframeHandler::push_keyframe(){
  dtam_->keyframe_vector_->push_back(dtam_->frame_current_);
}
