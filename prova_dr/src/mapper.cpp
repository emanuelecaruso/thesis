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

void CamCouple::getDepth(float u1, float v1, float& d1, float coord, bool u_or_v){
  // u2
  if (u_or_v){
    d1=((-H_bu)*coord + D_bu)/(E_bu*coord*u1 + F_bu*coord*v1 + G_bu*coord + (-A_bu)*u1 + (-B_bu)*v1 - C_bu);

  }
  // v2
  else{
    d1=((-H_bv)*coord + D_bv)/(E_bv*coord*u1 + F_bv*coord*v1 + G_bv*coord + (-A_bv)*u1 + (-B_bv)*v1 - C_bv);
  }
}

EpipolarLine* CamCouple::getEpSegment(Candidate* candidate, int bound_idx){

  float u1=candidate->uv_.x();
  float v1=candidate->uv_.y();

  float min_depth=candidate->bounds_->at(bound_idx).first;
  float max_depth=candidate->bounds_->at(bound_idx).second;

  float slope_m=0, bound_up=0, bound_low=0;
  getSlope(u1, v1, slope_m);

  bool u_or_v = (slope_m<1 && slope_m>-1);

  getBounds(u1, v1, min_depth, max_depth, bound_low, bound_up, u_or_v);

  EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m, candidate->level_);
  return ep_seg;
}

// EpipolarLine* CamCouple::getEpSegmentGt(Candidate* candidate){
//
//
//
//   Eigen::Vector2f uv = candidate->uv_;
//
//   float min_depth = candidate->min_depth_;
//   float max_depth = candidate->max_depth_;
//
//   Eigen::Vector3f p_min;
//   cam_r_->pointAtDepth( uv, min_depth, p_min);
//   Eigen::Vector2f uv_min;
//   cam_m_->projectPoint(p_min,uv_min);
//
//   Eigen::Vector3f p_max;
//   cam_r_->pointAtDepth( uv, max_depth, p_max);
//   Eigen::Vector2f uv_max;
//   cam_m_->projectPoint(p_max,uv_max);
//
//   float slope = (uv_max.y()-uv_min.y())/(uv_max.x()-uv_min.x());
//
//   EpipolarLine* ep_seg_gt= new EpipolarLine( cam_m_, uv_min, uv_max, candidate->level_);
//
//   return ep_seg_gt;
// }

// void CamCouple::compareEpSegmentWithGt(Candidate* candidate){
//   std::cout << "camCouple: " << cam_r_->name_ << " " << cam_m_->name_ << std::endl;
//
//   EpipolarLine* ep_query = getEpSegment( candidate);
//   EpipolarLine* ep_gt = getEpSegmentGt(candidate);
//
//
//   ep_gt->showEpipolarComparison(ep_query, true, 2);
// }

// void CamCouple::showEpSegment(Candidate* candidate){
//   // std::cout << "camCouple: " << cam_r_->name_ << " " << cam_m_->name_ << std::endl;
//   EpipolarLine* ep_query = getEpSegment( candidate);
//
//   // sharedCoutDebug(", slope query: "+std::to_string(ep_query->slope));
//   // sharedCoutDebug(", start query: "+std::to_string(ep_query->start));
//   // sharedCoutDebug(", end query: "+std::to_string(ep_query->end));
//   // sharedCoutDebug(", c0 query: "+std::to_string(ep_query->c0));
//   // sharedCoutDebug(", u_or_v query: "+std::to_string(ep_query->u_or_v));
//   // sharedCoutDebug(","+std::to_string(cam_r_projected_in_cam_m.y()));
//
//
//   ep_query->showEpipolar(candidate->level_,2);
// }



void Mapper::selectNewCandidates(){
  int idx=dtam_->keyframe_vector_->back();
  CameraForMapping* cam_r= dtam_->camera_vector_->at(idx);
  cam_r->collectRegions(parameters_->grad_threshold);
  cam_r->selectNewCandidates(parameters_->num_candidates);
  sharedCoutDebug("   - New candidates added");
}


EpipolarLine* CamCouple::trackCandidate(Candidate* candidate, int bound_idx){

  EpipolarLine* ep_segment = getEpSegment( candidate, bound_idx );

  return ep_segment;
}

bool Mapper::updateBounds(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple){


  int num_mins = ep_line->uv_idxs_mins->size();
  if (num_mins<=0)
    return 0;

  float bound_min;
  float bound_max;
  float coord;
  int sign = pow(-1,(ep_line->start>ep_line->end));
  float pixel_width = ep_line->cam->getPixelWidth(candidate->level_);  // GET PIXEL WIDTH

  // iterate through mins
  for (int i=0; i<num_mins; i++){
    // uv min
    Eigen::Vector2f uv_curr=ep_line->uvs->at(ep_line->uv_idxs_mins->at(i));

    if(ep_line->u_or_v)
      coord=uv_curr.x();
    else
      coord=uv_curr.y();

    // from uv to bound
    float coord_min = coord-sign*pixel_width*0.5;
    float coord_max = coord+sign*pixel_width*0.5;
    // std::cout << coord_min<< " "<< coord_max << std::endl;

    cam_couple->getDepth(candidate->uv_.x(), candidate->uv_.y(), bound_min, coord_min, ep_line->u_or_v);
    cam_couple->getDepth(candidate->uv_.x(), candidate->uv_.y(), bound_max, coord_max, ep_line->u_or_v);

    if ((bound_max-bound_min)< parameters_->max_depth_var && num_mins==1){

      // point is ready to be activate
      candidate->ready_=true;
      break;
    }
    // update bound
    bound bound_{bound_min,bound_max};

    // push back new bound
    candidate->bounds_->push_back(bound_);

  }

  return 1;

}

void Mapper::trackExistingCandidates(){

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  sharedCoutDebug("   - Tracking existing candidates");

  int num_of_ready_candidates=0;

  //iterate through active keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);

    sharedCoutDebug("      - Keyframe "+std::to_string(i)+" on "+std::to_string(dtam_->keyframe_vector_->size()-1)+
                    " (frame "+std::to_string(idx)+" on "+std::to_string(dtam_->keyframe_vector_->back())+")");

    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);

    sharedCoutDebug("         - N. candidates: "+std::to_string(keyframe->candidates_->size()));

    bool keep_cand = false;
    bool flag = 1;

    // keyframe->showCandidates_2(2);
    // cv::waitKey(0);

    // iterate through all candidates
    for(int k=0; k<keyframe->candidates_->size(); k++){

      Candidate* cand = keyframe->candidates_->at(k);


      if(cand->ready_){
        num_of_ready_candidates++;
        // break;
        continue;
      }

      int bounds_size =cand->bounds_->size();
      // iterate along all bounds
      for(int j=0; j<bounds_size; j++){

        EpipolarLine* ep_segment = cam_couple->trackCandidate(cand, j);
        ep_segment->searchMin(cand, parameters_);

        // if (dtam_->frame_current_==1){
        // if (flag){
        //   if(j==cand->bounds_->size()-1)
        //     flag=0;
        //   ep_segment->showEpipolarWithMin(cand->level_);
        //   keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->c->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_);
        //   cv::waitKey(0);
        //   cv::destroyAllWindows();
        // }

        // cam_couple->compareEpSegmentWithGt(cand);
        // cam_couple->showEpSegment(cand);

        if(updateBounds(cand,ep_segment,cam_couple)){
          keep_cand=true;
        }

      }
      if (keep_cand)
        cand->bounds_->erase (cand->bounds_->begin(),cand->bounds_->begin()+bounds_size);
      else{
        keyframe->candidates_->erase(keyframe->candidates_->begin()+k);
        k--;
        delete cand;
      }


    }

  }
  sharedCoutDebug("   - Num of ready candidates: "+ std::to_string(num_of_ready_candidates));

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
