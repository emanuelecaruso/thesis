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

  getCoord(u1, v1, min_depth, bound_low, u_or_v);
  getCoord(u1, v1, max_depth, bound_up, u_or_v);

}

void CamCouple::getCoord(float u1, float v1, float d1, float& coord, bool u_or_v){
  // u2
  if (u_or_v){
    coord=(A_bu*u1*d1+B_bu*v1*d1+C_bu*d1+D_bu)/(E_bu*u1*d1+F_bu*v1*d1+G_bu*d1+H_bu);
  }
  // v2
  else{
    coord=(A_bv*u1*d1+B_bv*v1*d1+C_bv*d1+D_bv)/(E_bv*u1*d1+F_bv*v1*d1+G_bv*d1+H_bv);
  }
}

void CamCouple::getUv(float u1, float v1, float d1, float& u2, float& v2 ){
  u2=(A_bu*u1*d1+B_bu*v1*d1+C_bu*d1+D_bu)/(E_bu*u1*d1+F_bu*v1*d1+G_bu*d1+H_bu);
  v2=(A_bv*u1*d1+B_bv*v1*d1+C_bv*d1+D_bv)/(E_bv*u1*d1+F_bv*v1*d1+G_bv*d1+H_bv);
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

float Mapper::computeStandardDeviation(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple, Eigen::Vector2f& uv_min, float pixel_width){

  float standard_deviation;
  Eigen::Vector2i pixel_m;
  cam_couple->cam_m_->uv2pixelCoords(uv_min, pixel_m, candidate->level_);
  if (!cam_couple->cam_m_->wavelet_dec_->vector_wavelets->at(candidate->level_)->phase_cd->pixelInRange(pixel_m))
    return -1;
  // GEOMETRIC DISPARITY ERROR
  float g_dot_l; // squared scalar product between direction of gradient and ep_line -> |g| |l| cos(a)
                 // since |g|, |l| =1 -> cos(angle between g and l)
  float angle_g = cam_couple->cam_m_->wavelet_dec_->vector_wavelets->at(candidate->level_)->phase_cd->evalPixel(pixel_m);
  float angle_l =ep_line->slope2angle();

  float a = radiansSub(angle_g,angle_l);


  float c_a = cos(a);
  g_dot_l=abs(c_a);

  // standard deviation epipolar line (fixed)
  // float sd_epline_geo = parameters_->sd_epline_geo;
  float sd_epline_geo = pixel_width/4;

  // standard deviation disparity
  float sd_disparity_geo = sd_epline_geo/(g_dot_l+0.1);
  // std::cout << "sd: " << sd_disparity_geo << ", ca: " << c_a << ", a: " << a << ", angle g: " << angle_g << ", angle l: " << angle_l << std::endl;


  // PHOTOMETRIC ERROR
  // gradient on epline direction
  float magn_g = cam_couple->cam_m_->wavelet_dec_->vector_wavelets->at(candidate->level_)->magn_cd->evalPixel(pixel_m);
  float g_p = g_dot_l*magn_g;

  // standard deviation img noise
  // float sd_img_noise = parameters_->sd_img_noise;
  float sd_img_noise = pixel_width/400;

  // standard deviation photometric
  float sd_disparity_photometric = abs(sd_img_noise/(g_p+0.01));

  // SAMPLING ERROR
  // float sd_epline_sampling = pixel_width/8;


  // standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric+sd_epline_sampling);
  standard_deviation = 2*(sd_disparity_geo+sd_disparity_photometric);
  // standard_deviation = 2*(sd_disparity_photometric);
  // standard_deviation = 2*(sd_disparity_geo);
  // std::cout << "sd " << (sd_disparity_photometric+sd_epline_sampling) << ", sd photo: " << sd_disparity_photometric << std::endl;

  return standard_deviation;

}

void Mapper::updateBoundsAndGetSD(Candidate* candidate, EpipolarLine* ep_line, CamCouple* cam_couple, float& standard_deviation){
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

    standard_deviation = computeStandardDeviation(candidate, ep_line, cam_couple, uv_curr, pixel_width);

    if (standard_deviation==-1)
      continue;

    // DISPARITY -> INVDEPTH CONVERSION: ENGEL
    // TODO

    // DISPARITY -> INVDEPTH CONVERSION: FULL METHOD
    if(ep_line->u_or_v)
      coord=uv_curr.x();
    else
      coord=uv_curr.y();
    // from uv to bound
    float coord_min = coord-sign*standard_deviation;
    float coord_max = coord+sign*standard_deviation;
    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_min, coord_min, ep_line->u_or_v);
    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_max, coord_max, ep_line->u_or_v);

    bound bound_{bound_min,bound_max};

    // push back new bound
    candidate->bounds_->push_back(bound_);

  }
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
    float coord_min = coord-sign*pixel_width*0.75;
    float coord_max = coord+sign*pixel_width*0.75;

    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_min, coord_min, ep_line->u_or_v);
    cam_couple->getD1(candidate->uv_.x(), candidate->uv_.y(), bound_max, coord_max, ep_line->u_or_v);

    bound bound_{bound_min,bound_max};

    // push back new bound
    candidate->bounds_->push_back(bound_);

  }


}



CandidateProjected* Mapper::projectCandidate(Candidate* candidate, CamCouple* cam_couple){

  Eigen::Vector3f p;
  Eigen::Vector2f uv;
  Eigen::Vector2i pixel_coords;
  float depth_m;

  cam_couple->getD2(candidate->uv_.x(), candidate->uv_.y(), 1.0/candidate->invdepth_, depth_m );
  cam_couple->getUv(candidate->uv_.x(), candidate->uv_.y(), 1.0/candidate->invdepth_, uv.x(), uv.y() );

  cam_couple->cam_m_->uv2pixelCoords( uv, pixel_coords, candidate->level_);

  if (candidate->cam_->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->pixelInRange(pixel_coords)){
    CandidateProjected* projected_cand = new CandidateProjected(candidate, pixel_coords, uv, 1.0/depth_m, cam_couple->cam_m_ );
    return projected_cand;
  }
  return nullptr;

}



CandidateProjected* Mapper::projectCandidateAndUpdateCandInvdepth(Candidate* candidate, CamCouple* cam_couple, EpipolarLine* ep_line , Eigen::Vector2f uv_curr){

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
  cam_couple->cam_r_->pointAtDepth(candidate->uv_, d1, *(candidate->p_), *(candidate->p_incamframe_));

  if (candidate->cam_->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->pixelInRange(pixel_curr)){
    CandidateProjected* projected_cand = new CandidateProjected(candidate, pixel_curr, uv_curr, 1.0/d2, cam_couple->cam_m_ );
    return projected_cand;
  }
  return nullptr;

  // return projected_cand;

}


void Mapper::trackExistingCandidatesGT(){

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  sharedCoutDebug("   - Tracking existing candidates (grountruth)");

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  //iterate through active keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->at(i));
    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);



    // iterate through all candidates
    for(int k=keyframe->candidates_->size()-1; k>=0; k--){

      Candidate* cand = keyframe->candidates_->at(k);
      cand->invdepth_var_=0.0001;
      cand->setInvdepthGroundtruth();
      cand->one_min_=true;
      cand->cam_->pointAtDepth(cand->uv_, 1.0/cand->invdepth_, *(cand->p_), *(cand->p_incamframe_));
      CandidateProjected* projected_cand=projectCandidate( cand, cam_couple);
      if (projected_cand != nullptr)
        cam_couple->cam_m_->regions_projected_cands_->pushProjCandidate(projected_cand);
    }




  }
}

void Mapper::trackExistingCandidates(bool take_gt_points, bool debug_mapping){
  if(take_gt_points){
    trackExistingCandidatesGT();
  }
  else
    trackExistingCandidates_(  debug_mapping);
}



void Mapper::trackExistingCandidates_(bool debug_mapping){

  double t_start=getTime();

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  sharedCoutDebug("   - Tracking existing candidates");

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  float total_error=0;

  std::vector<Image<colorRGB>*> imgs_to_destroy;

  //iterate through active keyframes
  for(int i=0; i<dtam_->keyframe_vector_->size()-1; i++){

    int idx = dtam_->keyframe_vector_->at(i);

    CameraForMapping* keyframe = dtam_->camera_vector_->at(idx);

    sharedCoutDebug("      - Keyframe "+std::to_string(i)+" on "+std::to_string(dtam_->keyframe_vector_->size()-1)+
                    " (frame "+std::to_string(idx)+" on "+std::to_string(dtam_->keyframe_vector_->back())+": "+ keyframe->name_ +" on "+ last_keyframe->name_ +")");

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);

    int n_cand_to_track = keyframe->candidates_->size();
    int n_cand_tracked = 0;
    int n_cand_without_mins = 0;
    int n_cand_updated = 0;
    int n_cand_repetitive = 0;
    int n_cand_not_updated = 0;
    int n_cand_not_in_newframe = 0;


    // iterate through all candidates
    for(int k=keyframe->candidates_->size()-1; k>=0; k--){

      bool keep_cand = false;
      bool projected_cand_created = false;

      Candidate* cand = keyframe->candidates_->at(k);

      CandidateProjected* projected_cand;
      int num_mins=0;
      int bounds_size =cand->bounds_->size();

      float standard_deviation;

      // iterate along all bounds
      for(int j=0; j<bounds_size; j++){

        EpipolarLine* ep_segment = cam_couple->getEpSegment( cand, j );

        // if uvs is empty, uvs are outside the frustum
        if(ep_segment->uvs->empty()){
          // those bounds are non valid
          n_cand_not_in_newframe++;
          continue;
        }

        // if uvs<=3, epipolar segment is too short to update bounds
        // if(ep_segment->uvs->size()<=3){
        //
        //   // if(cam_couple->cam_m_->name_=="Camera0011" && cam_couple->cam_r_->name_=="Camera0010"){
        //     ep_segment->showEpipolarWithMin(cand->level_);
        //     Image<float>* magn = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->magn_cd;
        //     magn->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_+"magn");
        //     cv::waitKey(0);
        //   // }
        //
        //   // push the same bounds
        //   cand->bounds_->push_back(cand->bounds_->at(j));
        //
        //   keep_cand=true;
        //   n_cand_not_updated++;
        //
        //   // if there are no mins till now
        //   if(!num_mins ){
        //     // save projected cand in case is going to be pushed
        //     // projected_cand=projectCandidate( cand, cam_couple );
        //     projected_cand=projectCandidate( cand, cam_couple, ep_segment , ep_segment->uvs->at(1));
        //     projected_cand_created=true;
        //   }
        //
        //   num_mins++;
        //
        //   continue;
        // }

        // if no mins are found
        // else if (!ep_segment->searchMin(cand, parameters_)){
        else if (!ep_segment->searchMin(cand, parameters_, cam_couple)){
        // else if (!ep_segment->searchMinDSO(cand, parameters_, cam_couple)){
          // those bounds are not valid
          n_cand_without_mins++;
          continue;
        }
        // if too much mins are found (higly repetitive texture)
        else if (ep_segment->uv_idxs_mins->size()>parameters_->max_num_mins){
          keep_cand=false;
          n_cand_repetitive++;
          // discard candidate

          // //DEBUG
          // if(true){
          // if(cam_couple->cam_m_->name_=="Camera0011" && cam_couple->cam_r_->name_=="Camera0010"){
          //   ep_segment->showEpipolarWithMin(cand->level_);
          //   Image<float>* magn = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->magn_cd;
          //   magn->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_+"magn");
          //   cv::waitKey(0);
          // }

          break;
        }
        else{

          //DEBUG
          // if(true){
          // if(cam_couple->cam_m_->name_=="Camera0002"){
          // // if(cam_couple->cam_m_->name_=="Camera0010" && cam_couple->cam_r_->name_=="Camera0008"){
          //   ep_segment->showEpipolarWithMin(cand->level_);
          //   Image<float>* magn = keyframe->wavelet_dec_->vector_wavelets->at(cand->level_)->magn_cd;
          //   magn->showImgWithColoredPixel(cand->pixel_,pow(2,cand->level_+1), keyframe->name_+"magn");
          //   cv::waitKey(0);
          // }

          n_cand_updated++;

          // if there are no mins till now, and only 1 min has been found
          if(!num_mins && ep_segment->uv_idxs_mins->size()==1 ){
            // save projected cand in case is going to be pushed
            projected_cand=projectCandidateAndUpdateCandInvdepth( cand, cam_couple, ep_segment , ep_segment->uvs->at(ep_segment->uv_idxs_mins->at(0)) );
            projected_cand_created=true;
          }


          keep_cand=true;

          updateBoundsAndGetSD( cand, ep_segment, cam_couple, standard_deviation);
          // updateBounds(cand,ep_segment,cam_couple);

          num_mins+=ep_segment->uv_idxs_mins->size();

        }

      }


      if (keep_cand){
        n_cand_tracked++;
        // erase old bounds
        cand->bounds_->erase (cand->bounds_->begin(),cand->bounds_->begin()+bounds_size);

        if( num_mins==1 && projected_cand!=nullptr ){
          // push inside "candidates projected vec" in new keyframe
          cam_couple->cam_m_->regions_projected_cands_->pushProjCandidate(projected_cand);
          cand->invdepth_var_=cand->getInvdepthStandardDeviation();
          cand->one_min_=true;

          if(debug_mapping ){
            Eigen::Vector2i pixel_proj;
            last_keyframe->uv2pixelCoords(projected_cand->uv_, pixel_proj);
            float invdepth_val = last_keyframe->grountruth_camera_->invdepth_map_->evalPixel(pixel_proj);
            float invdepth_gt = invdepth_val/last_keyframe->cam_parameters_->min_depth;
            total_error+=pow(invdepth_gt-projected_cand->invdepth_,2);
          }

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
        cand->marginalize();
      }


    }

    // sharedCoutDebug("         - # candidates tracked: "+std::to_string(n_cand_tracked)+ " out of "+std::to_string(n_cand_to_track));
    sharedCoutDebug("         - tracked: "+std::to_string(n_cand_tracked)+ " ("+std::to_string(n_cand_updated)+" bounds updated, "+std::to_string(n_cand_not_updated)+" bounds not updated) out of "+std::to_string(n_cand_to_track));
    sharedCoutDebug("         - not tracked: "+std::to_string(n_cand_to_track-n_cand_tracked)+ " ("+std::to_string(n_cand_without_mins)+" bounds with no mins, "+std::to_string(n_cand_repetitive)+" bounds repetitive, "+std::to_string(n_cand_not_in_newframe)+" bounds outdide frustum ) out of "+std::to_string(n_cand_to_track));

    //DEBUG
    if(debug_mapping){
      keyframe->showCandidates(1);
    }
  }
  if(debug_mapping)
  {
    last_keyframe->showProjCandidates(1);
    sharedCoutDebug("         - DEBUG: total error: "+std::to_string(total_error));
    cv::waitKey(0);
    // cv::destroyAllWindows();
  }

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Candidates tracking, time: "+ std::to_string(deltaTime)+" ms");


}

bool Mapper::initializeCandidates(CameraForMapping* cam_r,
            CameraForMapping* cam_m, int& current_r_idx){

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
