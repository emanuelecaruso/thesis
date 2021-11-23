#include "dtam.h"
#include "tracker.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void Tracker::trackGroundtruth(){
  const std::vector<CameraForStudy*>* cam_vec_env = dtam_->environment_->camera_vector_;
  int last_cam_idx=dtam_->camera_vector_->size()-1;

  Camera* cam_gt = cam_vec_env->at(last_cam_idx);
  CameraForMapping* cam = dtam_->camera_vector_->back();

  cam->frame_world_wrt_camera_=cam_gt->frame_world_wrt_camera_;
  cam->frame_camera_wrt_world_=cam_gt->frame_camera_wrt_world_;
}

void Tracker::trackCam(bool takeGtPoses){
  if(takeGtPoses){
    trackGroundtruth();
  }
  else{

  }
}

void Tracker::track(){
}
