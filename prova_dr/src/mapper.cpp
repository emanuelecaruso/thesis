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
  // A_s=2*r(2,0)*t(1) - 2*r(1,0)*t(2);
  // B_s=2*r(1,1)*t(2) - 2*r(1,2)*t(1);
  // C_s=2*f*r(1,2)*t(2) - 2*f*r(2,2)*t(1) - h*r(1,1)*t(2) + h*r(1,2)*t(1) + r(1,0)*t(2)*w - r(2,0)*t(1)*w;
  // D_s=2*r(0,0)*t(2) - 2*r(2,0)*t(0);
  // E_s=2*r(1,2)*t(0) - 2*r(0,1)*t(2);
  // F_s=2*f*r(2,2)*t(0) - 2*f*r(0,2)*t(2) + h*r(0,1)*t(2) - h*r(1,2)*t(0) - r(0,0)*t(2)*w + r(2,0)*t(0)*w;

  A_s=2*r(1,0)*t(2) - 2*r(2,0)*t(1);
  B_s=2*r(1,1)*t(2) - 2*r(1,2)*t(1);
  C_s=2*f*r(1,2)*t(2) - 2*f*r(2,2)*t(1) - h*r(1,1)*t(2) + h*r(1,2)*t(1) - r(1,0)*t(2)*w + r(2,0)*t(1)*w;
  D_s=2*r(0,0)*t(2) - 2*r(2,0)*t(0);
  E_s=2*r(0,1)*t(2) - 2*r(1,2)*t(0);
  F_s=2*f*r(0,2)*t(2) - 2*f*r(2,2)*t(0) - h*r(0,1)*t(2) + h*r(1,2)*t(0) - r(0,0)*t(2)*w + r(2,0)*t(0)*w;
}

void CamCouple::getBoundsParameters(){

  A_bu=4*f*r(0,0) + 2*r(2,0)*w;
  B_bu=4*f*r(0,1) + 2*r(1,2)*w;
  C_bu=4*f2*r(0,2) - r(2,0)*w2 - 2*f*h*r(0,1) - 2*f*r(0,0)*w + 2*f*r(2,2)*w - h*r(1,2)*w;
  D_bu=4*t(0)*f2 + 2*t(2)*w*f;
  E_bu=4*r(2,0);
  F_bu=4*r(1,2);
  G_bu=4*f*r(2,2) - 2*h*r(1,2) - 2*r(2,0)*w;
  H_bu= 4*f*t(2);

  A_bv=4*f*r(1,0) + 2*h*r(2,0);
  B_bv=4*f*r(1,1) + 2*h*r(1,2);
  C_bv=4*f2*r(1,2) - h2*r(1,2) - 2*f*h*r(1,1) + 2*f*h*r(2,2) - 2*f*r(1,0)*w - h*r(2,0)*w;
  D_bv=4*t(1)*f2 + 2*h*t(2)*f;
  E_bv=4*r(2,0);
  F_bv=4*r(1,2);
  G_bv=4*f*r(2,2) - 2*h*r(1,2) - 2*r(2,0)*w;
  H_bv=4*f*t(2);

}

void CamCouple::getDepthParameters(){
  A_d=r(2,0)/f;
  B_d=r(1,2)/f;
  C_d=r(2,2) - (h*r(1,2) - r(2,0)*w)/(2*f);
  D_d=t(2);
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

void CamCouple::getD1(float u1, float v1, float& d1, float coord, bool u_or_v){
  // u2
  if (u_or_v){
    d1=((-H_bu)*coord + D_bu)/(E_bu*coord*u1 + F_bu*coord*v1 + G_bu*coord + (-A_bu)*u1 + (-B_bu)*v1 - C_bu);

  }
  // v2
  else{
    d1=((-H_bv)*coord + D_bv)/(E_bv*coord*u1 + F_bv*coord*v1 + G_bv*coord + (-A_bv)*u1 + (-B_bv)*v1 - C_bv);
  }
}

void CamCouple::getD2(float u1, float v1, float d1, float& d2){
  d2=(A_d*u1*d1+B_d*v1*d1+C_d*d1+D_d);
}

EpipolarLine* CamCouple::getEpSegment(Candidate* candidate, int bound_idx){

  float u1=candidate->uv_.x();
  float v1=candidate->uv_.y();

  float min_depth=candidate->bounds_->at(bound_idx).first;
  float max_depth=candidate->bounds_->at(bound_idx).second;

  float slope_m, bound_up, bound_low;
  getSlope(u1, v1, slope_m);

  bool u_or_v = (slope_m<1 && slope_m>-1);

  getBounds(u1, v1, min_depth, max_depth, bound_low, bound_up, u_or_v);

  EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m, candidate->level_);


  return ep_seg;

}



EpipolarLine* CamCouple::getEpSegmentDefaultBounds(float u1, float v1){

  float min_depth=cam_r_->cam_parameters_->min_depth;
  float max_depth=cam_r_->cam_parameters_->max_depth;

  float slope_m, bound_up, bound_low;
  getSlope(u1, v1, slope_m);

  bool u_or_v = (slope_m<1 && slope_m>-1);

  getBounds(u1, v1, min_depth, max_depth, bound_low, bound_up, u_or_v);

  EpipolarLine* ep_seg = new EpipolarLine(  cam_m_, slope_m, bound_low, bound_up, cam_r_projected_in_cam_m );

  return ep_seg;

}

void Mapper::selectNewCandidates(){
  int idx=dtam_->keyframe_vector_->back();
  CameraForMapping* cam_r= dtam_->camera_vector_->at(idx);
  cam_r->selectNewCandidates(parameters_->num_candidates);
  sharedCoutDebug("   - New candidates selected: ("+std::to_string(cam_r->n_candidates_)+")");
}



void Mapper::updateBounds(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple){

  int num_mins = ep_line->uv_idxs_mins->size();
  float bound_min;
  float bound_max;
  float d2;
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

    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_min, coord_min, ep_line->u_or_v);
    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_max, coord_max, ep_line->u_or_v);

    bound bound_{bound_min,bound_max};

    // push back new bound
    candidate->bounds_->push_back(bound_);

  }


}



// void Mapper::updateBounds(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple,
//                           CandidateProjected*& projected_cand, bool no_mins_till_now){
//
//   int num_mins = ep_line->uv_idxs_mins->size();
//   float bound_min;
//   float bound_max;
//   float d2;
//   float coord;
//   int sign = pow(-1,(ep_line->start>ep_line->end));
//   float pixel_width = ep_line->cam->getPixelWidth(candidate->level_);  // GET PIXEL WIDTH
//
//   // iterate through mins
//   for (int i=0; i<num_mins; i++){
//     // uv min
//     Eigen::Vector2f uv_curr=ep_line->uvs->at(ep_line->uv_idxs_mins->at(i));
//
//     if(ep_line->u_or_v)
//       coord=uv_curr.x();
//     else
//       coord=uv_curr.y();
//
//     // from uv to bound
//     float coord_min = coord-sign*pixel_width*0.5;
//     float coord_max = coord+sign*pixel_width*0.5;
//
//
//     cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_min, coord_min, ep_line->u_or_v);
//     cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), candidate->depth_, coord, ep_line->u_or_v);
//     cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_max, coord_max, ep_line->u_or_v);
//
//     cam_couple->getD2(candidate->uv_.x(), candidate->uv_.y(), candidate->depth_, d2);
//
//     bound bound_{bound_min,bound_max};
//
//     if (num_mins==1 && no_mins_till_now){
//
//       Eigen::Vector2i pixel_curr;
//       cam_couple->cam_m_->uv2pixelCoords(uv_curr,pixel_curr,candidate->level_);
//
//       // create projected
//       projected_cand = new CandidateProjected(candidate, pixel_curr, uv_curr, 1.0/d2 );
//
//     }
//
//
//     // push back new bound
//     candidate->bounds_->push_back(bound_);
//
//   }
//
//
// }

void Mapper::rotateDhAndDvInCands(CameraForMapping* keyframe){
  // cand
  Eigen::Matrix3f R = keyframe->frame_camera_wrt_world_->linear();
  float rollAngle= -atan2(R(1,0),R(1,1));

  // float slope_r, slope_m;
  // CameraForMapping* cam_r = dtam_->camera_vector_->at(0);
  // CameraForMapping* cam_m = keyframe;
  // CamCouple* cam_couple_rm = new CamCouple(cam_r,cam_m);
  // CamCouple* cam_couple_mr = new CamCouple(cam_m,cam_r);
  // Eigen::Vector2f uv_m{cam_m->cam_parameters_->width*0.5,cam_m->cam_parameters_->height*0.5};
  // Eigen::Vector3f p; Eigen::Vector2f uv_r;
  // cam_m->pointAtDepth(uv_m,1,p);
  // cam_r->projectPoint( p, uv_r );
  // cam_couple_rm->getSlope(uv_m.x(),uv_m.y(),slope_r);
  // cam_couple_mr->getSlope(uv_r.x(),uv_r.y(),slope_m);
  // float rollAngle = radiansSub( atan2(slope_m, 1), atan2(slope_r, 1));
  // std::cout << "aoo: " << rollAngle << std::endl;

  float c=cos(rollAngle);
  float s=sin(rollAngle);


  for(Candidate* cand : *(keyframe->candidates_)){
    // rotate dh and dv in current candidate
    pixelIntensity dh = cand->dh_;
    pixelIntensity dv = cand->dv_;
    cand->dh_robust_=dh*c-dv*s;
    cand->dv_robust_=dv*c+dh*s;

  }
}

void Mapper::rotateDhAndDvInWvltDec(CameraForMapping* keyframe){

  // cand
  Eigen::Matrix3f R = keyframe->frame_camera_wrt_world_->linear();
  float rollAngle= -atan2(R(1,0),R(1,1));

  // float slope_r, slope_m;
  // CameraForMapping* cam_r = dtam_->camera_vector_->at(0);
  // CameraForMapping* cam_m = keyframe;
  // CamCouple* cam_couple_rm = new CamCouple(cam_r,cam_m);
  // CamCouple* cam_couple_mr = new CamCouple(cam_m,cam_r);
  // Eigen::Vector2f uv_m{cam_m->cam_parameters_->width*0.5,cam_m->cam_parameters_->height*0.5};
  // Eigen::Vector3f p; Eigen::Vector2f uv_r;
  // cam_m->pointAtDepth(uv_m,1,p);
  // cam_r->projectPoint( p, uv_r );
  // cam_couple_rm->getSlope(uv_m.x(),uv_m.y(),slope_r);
  // cam_couple_mr->getSlope(uv_r.x(),uv_r.y(),slope_m);
  // float rollAngle = radiansSub( atan2(slope_m, 1), atan2(slope_r, 1));
  // std::cout << "aoo: " << rollAngle << std::endl;

  float c=cos(rollAngle);
  float s=sin(rollAngle);

  // iterate through wavelet levels
  for (Wvlt_lvl* wvlt_lvl : *(keyframe->wavelet_dec_->vector_wavelets)){
    cv::Mat_<pixelIntensity> dv_ = wvlt_lvl->dv->image_.clone();
    cv::Mat_<pixelIntensity> dh_ = wvlt_lvl->dh->image_.clone();
    cv::Mat_<pixelIntensity> dvs;
    cv::Mat_<pixelIntensity> dvc;
    cv::Mat_<pixelIntensity> dhs;
    cv::Mat_<pixelIntensity> dhc;
    cv::multiply(dv_, cv::Scalar(s,s,s), dvs);
    cv::multiply(dv_, cv::Scalar(c,c,c), dvc);
    cv::multiply(dh_, cv::Scalar(s,s,s), dhs);
    cv::multiply(dh_, cv::Scalar(c,c,c), dhc);
    cv::add(dhc,-dvs,wvlt_lvl->dh_robust->image_);
    cv::add(dvc,dhs,wvlt_lvl->dv_robust->image_);
  }


}

void Mapper::updateRotationalInvariantGradients(){
  // get current keyframe
  CameraForMapping* keyframe_current = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  if(dtam_->frame_current_==0){
    for (Wvlt_lvl* wvlt_lvl : *(keyframe_current->wavelet_dec_->vector_wavelets)){
      wvlt_lvl->dh_robust->image_=wvlt_lvl->dh->image_.clone();
      wvlt_lvl->dv_robust->image_=wvlt_lvl->dv->image_.clone();
    }
  }
  else{
    rotateDhAndDvInWvltDec(keyframe_current);
    CameraForMapping* keyframe_prev = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(dtam_->keyframe_vector_->size()-2));
    rotateDhAndDvInCands(keyframe_prev);
  }


}

CandidateProjected* Mapper::projectCandidate(Candidate* candidate, CamCouple* cam_couple){

  Eigen::Vector3f p;
  Eigen::Vector2f uv;
  Eigen::Vector2i pixel_coords;
  float depth_m;
  cam_couple->cam_r_->pointAtDepth(candidate->uv_,1.0/candidate->invdepth_,p);
  cam_couple->cam_m_->projectPoint( p, uv, depth_m );
  cam_couple->cam_m_->uv2pixelCoords( uv, pixel_coords, candidate->level_);
  CandidateProjected* projected_cand = new CandidateProjected(candidate, pixel_coords, uv, 1.0/depth_m );
  return projected_cand;
}

CandidateProjected* Mapper::projectCandidate(Candidate* candidate, CamCouple* cam_couple, EpipolarLine* ep_line ){

  Eigen::Vector2f uv_curr=ep_line->uvs->at(ep_line->uv_idxs_mins->at(0));
  Eigen::Vector2i pixel_curr;
  cam_couple->cam_m_->uv2pixelCoords(uv_curr,pixel_curr,candidate->level_);

  float coord, d1, d2;
  if(ep_line->u_or_v)
    coord=uv_curr.x();
  else
    coord=uv_curr.y();

  // update depth of candidate (unique, used only if there is one min)
  cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), d1, coord, ep_line->u_or_v);
  cam_couple->getD2(candidate->uv_.x(), candidate->uv_.y(), d1, d2);

  candidate->invdepth_=1/d1;

  CandidateProjected* projected_cand = new CandidateProjected(candidate, pixel_curr, uv_curr, 1.0/d2 );

  return projected_cand;

}

void Mapper::trackExistingCandidates(){

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  sharedCoutDebug("   - Tracking existing candidates");

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  //iterate through active keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);

    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);

    sharedCoutDebug("      - Keyframe "+std::to_string(i)+" on "+std::to_string(dtam_->keyframe_vector_->size()-1)+
                    " (frame "+std::to_string(idx)+" on "+std::to_string(dtam_->keyframe_vector_->back())+": "+ keyframe->name_ +" on "+ last_keyframe->name_ +")");

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);

    int n_cand_to_track = keyframe->candidates_->size();
    int n_cand_tracked = 0;

    // bool flag = 1;


    // iterate through all candidates
    for(int k=keyframe->candidates_->size()-1; k>=0; k--){

      bool keep_cand = false;
      bool projected_cand_created = false;

      Candidate* cand = keyframe->candidates_->at(k);

      CandidateProjected* projected_cand;
      int num_mins=0;
      int bounds_size =cand->bounds_->size();

      // iterate along all bounds
      for(int j=0; j<bounds_size; j++){

        EpipolarLine* ep_segment = cam_couple->getEpSegment( cand, j );

        // if uvs is empty, uvs are outside the frustum
        if(ep_segment->uvs->empty()){
          // those bounds are non valid
          continue;
        }

        // if uvs<3, epipolar segment is too short to update bounds
        if(ep_segment->uvs->size()<3){

          // push the same bounds
          cand->bounds_->push_back(cand->bounds_->at(j));

          keep_cand=true;
          // if there are no mins till now
          if(!num_mins){
            // save projected cand in case is going to be pushed
            projected_cand=projectCandidate( cand, cam_couple);
            projected_cand_created=true;
          }

          num_mins++;

          continue;
        }

        // if no mins are found
        else if (!ep_segment->searchMin(cand, parameters_)){
          // those bounds are not valid
          continue;
        }
        // if too much mins are found (higly repetitive texture)
        else if (ep_segment->uv_idxs_mins->size()>parameters_->max_num_mins){
          keep_cand=false;
          // discard candidate
          break;
        }
        else{

          // // if (ep_segment->uv_idxs_mins->size()==1){
          //   // if (dtam_->frame_current_==7){
          //   // if (flag){
          //   // if(j==cand->bounds_->size()-1)
          //   flag=0;
          //   // ep_segment->showEpipolar(cand->level_);
          //   ep_segment->showEpipolarWithMin(cand->level_);
          //
          //   // Image<float>* dh_rob = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->dh_robust->returnImgForGradientVisualization("dh_rob");
          //   // dh_rob->show(pow(2,cand->level_+1), keyframe->name_+"_dh_rob");
          //   // Image<float>* dv_rob = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->dv_robust->returnImgForGradientVisualization("dv_rob");
          //   // dv_rob->show(pow(2,cand->level_+1), keyframe->name_+"_dv_rob");
          //   // Image<float>* dh_rob_l = last_keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->dh_robust->returnImgForGradientVisualization("dh_rob");
          //   // dh_rob_l->show(pow(2,cand->level_+1), last_keyframe->name_+"_dh_rob last");
          //   // Image<float>* dv_rob_l = last_keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->dv_robust->returnImgForGradientVisualization("dv_rob");
          //   // dv_rob_l->show(pow(2,cand->level_+1), last_keyframe->name_+"_dv_rob last");
          //   Image<float>* magn = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->magnitude_img;
          //   magn->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_+"magn");
          //
          //   // keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->c->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_);
          //   // keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->magnitude3C_img->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_+"_phase");
          //   cv::waitKey(0);
          //   // cv::destroyAllWindows();
          // // }

          // cam_couple->compareEpSegmentWithGt(cand);
          // cam_couple->showEpSegment(cand);

          // if there are no mins till now, and only 1 min has been found
          if(!num_mins && ep_segment->uv_idxs_mins->size()==1 ){
            // save projected cand in case is going to be pushed
            projected_cand=projectCandidate( cand, cam_couple, ep_segment);
            projected_cand_created=true;
          }


          keep_cand=true;

          updateBounds(cand,ep_segment,cam_couple);

          num_mins+=ep_segment->uv_idxs_mins->size();

        }

      }

      if (keep_cand){
        n_cand_tracked++;
        // erase old bounds
        cand->bounds_->erase (cand->bounds_->begin(),cand->bounds_->begin()+bounds_size);

        if(num_mins==1){
          // push inside "candidates projected vec" in new keyframe
          cam_couple->cam_m_->regions_projected_cands_->pushCandidate(projected_cand);
          cand->invdepth_var_=cand->getInvdepthVar();
          cand->one_min_=true;
        }
        else if(num_mins>1){
          cand->invdepth_var_=-1;
          cand->invdepth_=-1;
          cand->one_min_=false;
          if(projected_cand_created)
            delete projected_cand;
        }
      }
      // if candidate has not been tracked, marginalize it
      else{
        keyframe->candidates_->erase(keyframe->candidates_->begin()+k);
        cand->marginalize();
      }


    }

    sharedCoutDebug("         - # candidates tracked: "+std::to_string(n_cand_tracked)+ " out of "+std::to_string(n_cand_to_track));

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
