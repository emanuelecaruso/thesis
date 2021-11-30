#include "mapper.h"
#include "epline.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <cstdlib>
#include <chrono>

void CamCouple::getSlopeParameters(){
  A_s=2*r(2,0)*t(1) - 2*r(1,0)*t(2);
  B_s=2*r(1,1)*t(2) - 2*r(1,2)*t(1);
  C_s=2*f*r(1,2)*t(2) - 2*f*r(2,2)*t(1) - h*r(1,1)*t(2) + h*r(1,2)*t(1) + r(1,0)*t(2)*w - r(2,0)*t(1)*w;
  D_s=2*r(0,0)*t(2) - 2*r(2,0)*t(0);
  E_s=2*r(1,2)*t(0) - 2*r(0,1)*t(2);
  F_s=2*f*r(2,2)*t(0) - 2*f*r(0,2)*t(2) + h*r(0,1)*t(2) - h*r(1,2)*t(0) - r(0,0)*t(2)*w + r(2,0)*t(0)*w;
}

void CamCouple::getBoundsParameters(){

  A_bu=2*r(2,0)*w - 4*f*r(0,0);
  B_bu=4*f*r(0,1) - 2*r(1,2)*w;
  C_bu=4*f2*r(0,2) - r(2,0)*w2 - 2*f*h*r(0,1) + 2*f*r(0,0)*w - 2*f*r(2,2)*w + h*r(1,2)*w;
  D_bu=- 4*t(0)*f2 + 2*t(2)*w*f;
  E_bu=4*r(2,0);
  F_bu=-4*r(1,2);
  G_bu=2*h*r(1,2) - 4*f*r(2,2) - 2*r(2,0)*w;
  H_bu= 4*f*t(2);

  A_bv=4*f*r(1,0) + 2*h*r(2,0);
  B_bv=- 4*f*r(1,1) - 2*h*r(1,2);
  C_bv=h2*r(1,2) - 4*f2*r(1,2) + 2*f*h*r(1,1) - 2*f*h*r(2,2) - 2*f*r(1,0)*w - h*r(2,0)*w;
  D_bv=4*t(1)*f2 + 2*h*t(2)*f;
  E_bv=4*r(2,0);
  F_bv=-4*r(1,2);
  G_bv=2*h*r(1,2) - 4*f*r(2,2) - 2*r(2,0)*w;
  H_bv=4*f*t(2);

}

void CamCouple::getSlope(float u1, float v1, float& slope_m){
  slope_m=(A_s*u1+B_s*v1+C_s)/(D_s*u1+E_s*v1+F_s);
}

void CamCouple::getBounds(float u1, float v1, float min_depth, float max_depth, float& bound_low, float& bound_up , bool u_or_v){

  getBound(u1, v1, min_depth, bound_low, u_or_v);
  getBound(u1, v1, max_depth, bound_up, u_or_v);

}

void CamCouple::getBound(float u1, float v1, float d1, float& bound, bool u_or_v){
  // u2
  if (u_or_v){
    bound=(A_bu*u1*d1+B_bu*v1*d1+C_bu*d1+D_bu)/(E_bu*u1*d1+F_bu*v1*d1+G_bu*d1+H_bu);
  }
  // v2
  else{
    bound=(A_bv*u1*d1+B_bv*v1*d1+C_bv*d1+D_bv)/(E_bv*u1*d1+F_bv*v1*d1+G_bv*d1+H_bv);
  }

}
EpipolarLine* CamCouple::getEpSegment(Candidate* candidate){

  float u1=candidate->uv_.x();
  float v1=candidate->uv_.y();


  float slope_m=0, bound_up=0, bound_low=0;
  getSlope(u1, v1, slope_m);

  bool u_or_v = (slope_m<1 && slope_m>-1);

  getBounds(u1, v1, candidate->min_depth_, candidate->max_depth_, bound_low, bound_up, u_or_v);

  EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m, candidate->level_);
  return ep_seg;
}

EpipolarLine* CamCouple::getEpSegmentGt(Candidate* candidate){



  Eigen::Vector2f uv = candidate->uv_;

  float min_depth = candidate->min_depth_;
  float max_depth = candidate->max_depth_;

  Eigen::Vector3f p_min;
  cam_r_->pointAtDepth( uv, min_depth, p_min);
  Eigen::Vector2f uv_min;
  cam_m_->projectPoint(p_min,uv_min);

  Eigen::Vector3f p_max;
  cam_r_->pointAtDepth( uv, max_depth, p_max);
  Eigen::Vector2f uv_max;
  cam_m_->projectPoint(p_max,uv_max);

  float slope = (uv_max.y()-uv_min.y())/(uv_max.x()-uv_min.x());

  EpipolarLine* ep_seg_gt= new EpipolarLine( cam_m_, uv_min, uv_max, candidate->level_);

  return ep_seg_gt;
}

void CamCouple::compareEpSegmentWithGt(Candidate* candidate){
  std::cout << "camCouple: " << cam_r_->name_ << " " << cam_m_->name_ << std::endl;

  EpipolarLine* ep_query = getEpSegment( candidate);
  EpipolarLine* ep_gt = getEpSegmentGt(candidate);


  ep_gt->showEpipolarComparison(ep_query, true, 2);


}

void CamCouple::showEpSegment(Candidate* candidate){
  // std::cout << "camCouple: " << cam_r_->name_ << " " << cam_m_->name_ << std::endl;
  EpipolarLine* ep_query = getEpSegment( candidate);

  // sharedCoutDebug(", slope query: "+std::to_string(ep_query->slope));
  // sharedCoutDebug(", start query: "+std::to_string(ep_query->start));
  // sharedCoutDebug(", end query: "+std::to_string(ep_query->end));
  // sharedCoutDebug(", c0 query: "+std::to_string(ep_query->c0));
  // sharedCoutDebug(", u_or_v query: "+std::to_string(ep_query->u_or_v));
  // sharedCoutDebug(","+std::to_string(cam_r_projected_in_cam_m.y()));


  ep_query->showEpipolar(candidate->level_,2);
}



void Mapper::selectNewCandidates(){
  int idx=dtam_->keyframe_vector_->back();
  CameraForMapping* cam_r= dtam_->camera_vector_->at(idx);
  cam_r->collectRegions(grad_threshold_);
  cam_r->selectNewCandidates(num_candidates_);
  sharedCoutDebug("   - New candidates added");
}


EpipolarLine* CamCouple::trackCandidate(Candidate* candidate){
  EpipolarLine* ep_segment = getEpSegment( candidate );

  return ep_segment;
}

void Mapper::trackExistingCandidates(){

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  sharedCoutDebug("   - tracking existing candidates");

  //iterate through active keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);

    sharedCoutDebug("      - keyframe "+std::to_string(i)+" on "+std::to_string(dtam_->keyframe_vector_->size()-1)+
                    " (frame "+std::to_string(i)+" on "+std::to_string(dtam_->keyframe_vector_->back())+")");

    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);

    // std::cout << keyframe->name_ << " on " << last_keyframe->name_ << std::endl;
    // sharedCoutDebug(std::to_string(keyframe->name_));
    // sharedCoutDebug(" on ");
    // sharedCoutDebug(std::to_string(last_keyframe->name_));

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);
    // iterate through all candidates

    for(Candidate* cand : *(keyframe->candidates_)){

      // compute epipolar segment of candidate in new keyframe
      EpipolarLine* epSegment = cam_couple->trackCandidate(cand);
      epSegment->updateBounds(cand, cost_threshold_ , grad_threshold_);

      epSegment->showEpipolarWithMin(cand->level_);
      keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->c->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_);
      cv::waitKey(0);

      // cam_couple->compareEpSegmentWithGt(cand);
      // cam_couple->showEpSegment(cand);
    }

  }

}

bool Mapper::initializeCandidates(const CameraForMapping* cam_r,
            const CameraForMapping* cam_m, int& current_r_idx){

   CamCouple* cam_couple = new CamCouple(cam_r, cam_m);

  // iterate along candidates
  while(!(cam_r->candidates_->empty())){
    // check if there is a new frame
    if(current_r_idx<dtam_->frame_current_-1){
      current_r_idx=dtam_->frame_current_-1;
      cam_r = dtam_->camera_vector_->at(current_r_idx);
      cam_m = dtam_->camera_vector_->at(current_r_idx-1);
    }

    Candidate* curr_cand=cam_r->candidates_->back();
    cam_r->candidates_->pop_back();
    // get epipolar segment
    // cam_couple->getEpSegment(curr_cand->uv_[0],curr_cand->uv_[1]);
    // search along epipolar line

    // compute mse with c,dd,dh,dv
    // roll -> associate dd,dh,dv

    // if minimum is clear, add as active point

  }



  return false;
}

void Mapper::doMapping(){

  // int frame_idx_r;
  // int frame_idx_m;
  //
  // while(true){
  //   // frameCouplingRandom(frame_idx_r, frame_idx_m );
  //   // frameCouplingLast(frame_idx_r, frame_idx_m );
  //   frameCouplingOpposite(frame_idx_r, frame_idx_m );
  //   // sharedCoutDebug("chosen frames: "+std::to_string(frame_idx_r)+ " " +std::to_string(frame_idx_m));
  //
  //   CameraForMapping* cam_r = dtam_->camera_vector_->at(frame_idx_r);
  //   CameraForMapping* cam_m = dtam_->camera_vector_->at(frame_idx_m);
  //   EpipolarLine* ep_line_r; EpipolarLine* ep_line_m;
  //
  //
  //
  //   // float size = 1;
  //   // float size = 1.3;
  //   // float size = 2;
  //   // locker.lock();
  //   //
  //   // cam_r->showCandidates_1(size);
  //   // cv::waitKey(0);
  //   // // cv::destroyAllWindows();
  //   // locker.unlock();
  //
  // }

}
