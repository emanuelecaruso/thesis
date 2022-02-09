#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

CameraForMapping* BundleAdj::getFrameCurrentBA(){
  return dtam_->camera_vector_->at(frame_current_ba);
}


void BundleAdj::activateNewPointsAndGetCoarseActivePoints(){

  int last_keyframe_idx = keyframe_vector_ba_->back(); // update frame current

  sharedCoutDebug("   - activating point in frame "+std::to_string(last_keyframe_idx));

  double t_start=getTime();

  int num_activated = selectNewActivePoints();
  num_active_points_+= num_activated;

  double t_end=getTime();
  int deltaTime=(t_end-t_start);

  sharedCoutDebug("   - Points activated in frame "+std::to_string(last_keyframe_idx)+": "+std::to_string(num_activated)+", number of active points: "+std::to_string(num_active_points_)+", time: "+std::to_string(deltaTime)+" ms");


}

void BundleAdj::addCoarseActivePointInRegion(ActivePoint* active_pt){

  CameraForMapping* keyframe = active_pt->cam_;

  // iterate along all coarser levels
  for(int i=1; i<=dtam_->parameters_->coarsest_level; i++){

    RegionsWithActivePoints* coarse_regions = keyframe->regions_coarse_active_pts_vec_->at(i-1);

    // if level of active points is less than current coarse level
    // and if active points has one min
    if(active_pt->level_<i ){

      // PUSH CANDIDATE IN REGION ---------------------------
      int level_diff = i-active_pt->level_;
      // from pixel of active points find pixel at level i
      int coarse_pxl_x = active_pt->pixel_.x()/(pow(2,level_diff));
      int coarse_pxl_y = active_pt->pixel_.y()/(pow(2,level_diff));
      int idx = coarse_regions->xyToIdx(coarse_pxl_x,coarse_pxl_y);
      // push active points inside region
      RegionWithActivePoints* reg = coarse_regions->region_vec_->at(idx);
      reg->active_pts_vec_->push_back(active_pt);
      reg->to_update_=true;

      // save coarse region inside active points
      active_pt->regions_coarse_->push_back(coarse_regions->region_vec_->at(idx));

    }
  }

}



void BundleAdj::collectCoarseActivePoints(){

  // iterate along all keyframes (except last)
  for (int i=0; i<keyframe_vector_ba_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // clear coarse active point vec
    for(std::vector<ActivePoint*>* v : *(keyframe->active_points_coarse_)){
      for (ActivePoint* active_point : *v)
        delete active_point;
      v->clear();
    }

    // iterate along all coarser levels
    for(int i=1; i<=dtam_->parameters_->coarsest_level; i++){

      RegionsWithActivePoints* coarse_regions = keyframe->regions_coarse_active_pts_vec_->at(i-1);

      // iterate along all regions
      for ( RegionWithActivePoints* reg : *(coarse_regions->region_vec_)){
        // if region is not empty
        if(!reg->active_pts_vec_->empty()){
        //

          Eigen::Vector2i pixel {reg->x_, reg->y_};
          Eigen::Vector2f uv;
          keyframe->pixelCoords2uv(pixel,uv, i);

          pixelIntensity c = keyframe->wavelet_dec_->getWavLevel(i)->c->evalPixel(pixel);
          // pixelIntensity c_dx = keyframe->wavelet_dec_->getWavLevel(i)->c_dx->evalPixel(pixel);
          // pixelIntensity c_dy = keyframe->wavelet_dec_->getWavLevel(i)->c_dy->evalPixel(pixel);
          pixelIntensity magn_cd = keyframe->wavelet_dec_->getWavLevel(i)->magn_cd->evalPixel(reg->y_,reg->x_);
          // pixelIntensity magn_cd_dx = keyframe->wavelet_dec_->getWavLevel(i)->magn_cd_dx->evalPixel(reg->y_,reg->x_);
          // pixelIntensity magn_cd_dy = keyframe->wavelet_dec_->getWavLevel(i)->magn_cd_dy->evalPixel(reg->y_,reg->x_);
          pixelIntensity phase_cd = keyframe->wavelet_dec_->getWavLevel(i)->phase_cd->evalPixel(reg->y_,reg->x_);
          // pixelIntensity phase_cd_dx = keyframe->wavelet_dec_->getWavLevel(i)->phase_cd_dx->evalPixel(reg->y_,reg->x_);
          // pixelIntensity phase_cd_dy = keyframe->wavelet_dec_->getWavLevel(i)->phase_cd_dy->evalPixel(reg->y_,reg->x_);


          // iterate along collected active points
          float invdepth, invdepth_var;
          float d_over_v_sum = 0, inv_v_sum = 0;
          int num_cands = 0;

          for(ActivePoint* active_point : *(reg->active_pts_vec_)){

            num_cands++;
            // update d_over_v_sum
            d_over_v_sum+= active_point->invdepth_/active_point->invdepth_var_;
            // update inv_v_sum
            inv_v_sum+= 1./active_point->invdepth_var_;

            // d_over_v_sum = active_point->invdepth_;
            // inv_v_sum= 1;
            // num_cands=1;
            // break;

          }
          if (num_cands){
            // compute invdepth as weighted average of invdepths with invdepth certainty as weight
            invdepth = d_over_v_sum/inv_v_sum;
            Eigen::Vector3f* p = new Eigen::Vector3f ;
            Eigen::Vector3f* p_incamframe = new Eigen::Vector3f ;
            keyframe->pointAtDepth(uv,1.0/invdepth,*p,*p_incamframe);
            // compute invdepth variance as average of variances
            invdepth_var = (float)num_cands/inv_v_sum;

            // create coarse active point
            ActivePoint* active_point_coarse = new ActivePoint(i,pixel, uv, keyframe,
                                                        c,
                                                        magn_cd,
                                                        phase_cd,
                                                        // c_dx, c_dy,
                                                        // magn_cd_dx, magn_cd_dy,
                                                        // phase_cd_dx, phase_cd_dy,
                                                        invdepth,invdepth_var,
                                                        p,p_incamframe);
            // push active point inside coarse active points
            keyframe->active_points_coarse_->at(i-1)->push_back(active_point_coarse);
          }
        }

      }
    }

  }

}


ActivePoint* BundleAdj::activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs){

  Candidate* cand = cand_proj->cand_;
  std::vector<Candidate*>* cands_vec = cand->region_sampling_->cands_vec_;

  int num_proj_active_points = regs->getNumOfActivePointsInReg( reg );
  if( num_proj_active_points<=min_num_of_active_pts_per_region_ ){
    // create active point from candidate
    ActivePoint* active_point = new ActivePoint(cand);

    // push active point
    active_point->cam_->active_points_->push_back(active_point);

    // create active point projected from projected candidate
    ActivePointProjected* active_point_proj = new ActivePointProjected(cand_proj, active_point) ;

    // push active point projected
    active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);

    // marginalize candidate
    cand->marginalize();
    reg->cands_proj_vec_->erase(reg->cands_proj_vec_->begin()); // remove projected active point from the region
    delete cand_proj; // delete projected candidate

    return active_point;
  }
  return nullptr;
}

int BundleAdj::selectNewActivePoints(){

  int last_keyframe_idx = keyframe_vector_ba_->back(); // update frame current
  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(last_keyframe_idx);

  // compute min_num_of_active_pts_per_region_
  min_num_of_active_pts_per_region_=INT_MAX;
  RegionsWithProjActivePoints* regs = last_keyframe->regions_projected_active_points_;
  // find min number of active points in region
  for (RegionWithProjActivePoints* reg : *(regs->region_vec_)){
    if(reg->active_pts_proj_vec_->size()>0 && reg->active_pts_proj_vec_->size()<min_num_of_active_pts_per_region_){
      min_num_of_active_pts_per_region_=reg->active_pts_proj_vec_->size();
    }
  }
  if(min_num_of_active_pts_per_region_==INT_MAX){
    min_num_of_active_pts_per_region_=0;
  }

  // num of points to be activated
  int num_to_be_activated=parameters_->max_num_active_points- num_active_points_;

  int num_activated = 0;

  // RegionsWithProjCandidates* regions = last_keyframe->regions_projected_cands_;
  std::vector<RegionWithProjCandidates*>* reg_vec= last_keyframe->regions_projected_cands_->region_vec_;

  std::lock_guard<std::mutex> locker(dtam_->mu_candidate_tracking_);

  int i = 0;
  int round = 0;
  while(num_activated<num_to_be_activated){
  // for (int i=0; i<num_to_be_activated; i++){


    if (reg_vec->empty())
      break;

    int reg_idx = i%reg_vec->size();
    RegionWithProjCandidates* reg = reg_vec->at(reg_idx);
    if (reg->cands_proj_vec_->empty()){
      reg_vec->erase(std::remove(reg_vec->begin(), reg_vec->end(), reg), reg_vec->end());
      i--;
      continue;
    }

    i++;

    CandidateProjected* cand_proj = reg->cands_proj_vec_->at(0);
    ActivePoint* active_pt = activateCandidate(cand_proj,reg,regs);

    if (active_pt!=nullptr){
      // update coarse Active Point
      addCoarseActivePointInRegion(active_pt);

      // active point
      num_activated++;
    }

    // if it's the last round
    if(reg_idx==reg_vec->size()-1){
      min_num_of_active_pts_per_region_++;
    }
  }



  return num_activated;

}

void BundleAdj::marginalize(){

  // compute marginalization term

}

void BundleAdj::updateStateBlockIdxs(int& pose_block_size, int& point_block_size){

  int num_active_keyframes=0;
  int num_keyframes_to_be_marginalized=0;
  int num_active_points_=0;
  // iterate through all keyframes
  for (int i=0; i<keyframe_vector_ba_->size(); i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    if(!keyframe->to_be_marginalized_ba_){
      num_active_keyframes++;
      // set state pose block idx
      keyframe->state_pose_block_idx_=num_active_keyframes*6;
      for(int j=0; j<keyframe->active_points_->size(); j++ ){
        keyframe->active_points_->at(j)->state_point_block_idx_=num_active_points_+j;
      }
      num_active_points_+=keyframe->active_points_->size();
    }
    else{
      num_keyframes_to_be_marginalized++;
    }
  }

  pose_block_size=num_active_keyframes*6;
  point_block_size=num_active_points_;


}

Eigen::Matrix<float,1,3>* BundleAdj::getJfirst(ActivePointProjected* active_pt_proj, Eigen::Vector3f& point_m){
  ActivePoint* active_pt = active_pt_proj->active_point_;
  CameraForMapping* cam_m = active_pt_proj->cam_;
  CameraForMapping* cam_r = active_pt->cam_;

  float pixels_meter_ratio = dtam_->camera_vector_->at(0)->cam_parameters_->resolution_x/dtam_->camera_vector_->at(0)->cam_parameters_->width;
  Eigen::Matrix3f K = *(cam_m->K_);
  float variance = dtam_->parameters_->variance;
  int ni = dtam_->parameters_->robustifier_dofs;
  float coeff = pixels_meter_ratio/pow(2,active_pt->level_+1);

  // variables
  Eigen::Vector2f uv_newframe;
  Eigen::Vector2i pixel_newframe;
  Eigen::Vector3f* point = active_pt->p_0_; // 3D point wrt world frame

  point_m= *(cam_m->frame_world_wrt_camera_)*(*point);

  Eigen::Vector3f p_proj = K*point_m;
  // return false if the projected point is behind the camera
  if (p_proj.z()<cam_m->cam_parameters_->lens)
    return nullptr;
  uv_newframe = p_proj.head<2>()*(1./p_proj.z());

  cam_m->projectPointInCamFrame( point_m, uv_newframe );
  cam_m->uv2pixelCoords(uv_newframe, pixel_newframe, active_pt->level_);

  if(!cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->pixelInRange(pixel_newframe))
    return nullptr;

  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;
  // Eigen::Matrix<float, 3,1> invdepth_jacobian;

  // invdepth_jacobian << -active_pt->uv_.x()/pow(invdepth,2),
  //                        -active_pt->uv_.y()/pow(invdepth,2),
  //                        -1/pow(invdepth,2);
  //
  // jacobian_to_mul_normalizer = proj_jacobian*(K*(current_guess.linear()*(active_pt->cam_->frame_camera_wrt_world_->linear()*invdepth_jacobian)));


  proj_jacobian << 1./p_proj.z(), 0, -p_proj.x()/pow(p_proj.z(),2),
                   0, 1./p_proj.z(), -p_proj.y()/pow(p_proj.z(),2);

  Eigen::Matrix<float, 1,2> img_jacobian;
  img_jacobian << cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dx->evalPixel(pixel_newframe), cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dy->evalPixel(pixel_newframe);

  Eigen::Matrix<float,1,3>* J_first;
  *J_first = coeff*((img_jacobian*proj_jacobian)*K);
  return J_first;
}

Eigen::Matrix<float,1,6>* BundleAdj::getJr(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first){
  ActivePoint* active_pt = active_pt_proj->active_point_;
  CameraForMapping* cam_m = active_pt->cam_;

  Eigen::Vector3f point = *(active_pt->p_0_);
  Eigen::Matrix<float, 3,6> state_jacobian;

  state_jacobian << 1, 0, 0,  0         ,  point.z()  , -point.y(),
                    0, 1, 0, -point.z() ,  0          ,  point.x(),
                    0, 0, 1,  point.y() , -point.x()  ,  0         ;


  Eigen::Matrix<float,1,6>* J_r = new Eigen::Matrix<float,1,6>;
  // *J_r=((*J_first)*state_jacobian)/normalizer;
  *J_r=(*J_first)*(cam_m->frame_world_wrt_camera_->linear())*state_jacobian;

  return J_r;
}

Eigen::Matrix<float,1,6>* BundleAdj::getJm(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first, Eigen::Vector3f& point_m){


  Eigen::Matrix<float, 3,6> state_jacobian;

  state_jacobian << 1, 0, 0,  0           ,  point_m.z()  , -point_m.y(),
                    0, 1, 0, -point_m.z() ,  0            ,  point_m.x(),
                    0, 0, 1,  point_m.y() , -point_m.x()  ,  0         ;


  Eigen::Matrix<float,1,6>* J_m = new Eigen::Matrix<float,1,6>;
  // *J_m=(coeff*(img_jacobian*jacobian_to_mul))/normalizer;
  *J_m=(*J_first)*state_jacobian;



  return J_m;
}

float BundleAdj::getJd(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first){
  ActivePoint* active_pt = active_pt_proj->active_point_;

  return 1;
}

float BundleAdj::getError(ActivePointProjected* active_pt_proj){
  float z = active_pt_proj->active_point_->intensity_;
  float z_hat = active_pt_proj->cam_->wavelet_dec_->getWavLevel(active_pt_proj->level_)->c->evalPixel(active_pt_proj->pixel_);

  // error
  // float error = (z_hat-z)/normalizer;
  float error = (z_hat-z);

  return error;
}

JacobiansBA* BundleAdj::getJacobians(ActivePointProjected* active_pt_proj){

  Eigen::Matrix<float,1,3>* J_1;
  Eigen::Vector3f point_m;
  J_1= getJfirst( active_pt_proj,  point_m);

  if (J_1==nullptr)
    return nullptr;

  Eigen::Matrix<float,1,6>* J_r = getJr(active_pt_proj, J_1);
  Eigen::Matrix<float,1,6>* J_m = getJm(active_pt_proj, J_1, point_m);
  float J_d = getJd(active_pt_proj, J_1);

  int J_r_block_idx = active_pt_proj->active_point_->cam_->state_pose_block_idx_;
  int J_m_block_idx = active_pt_proj->cam_->state_pose_block_idx_;
  int J_d_block_idx = active_pt_proj->active_point_->state_point_block_idx_;

  // JacobiansBA* jacobians = new JacobiansBA();
}

void BundleAdj::optimize(int pose_block_size, int point_block_size ){

  // initialize H blocks
  Eigen::MatrixXf H_pose_pose(pose_block_size,point_block_size);
  Eigen::MatrixXf H_pose_point(point_block_size,point_block_size);
  Eigen::MatrixXf H_point_pose(pose_block_size,point_block_size);
  Eigen::MatrixXf H_point_point(point_block_size,point_block_size);

  // initialize b blocks
  Eigen::MatrixXf b_pose(pose_block_size,1);
  Eigen::MatrixXf b_point(point_block_size,1);

  // iterate through keyframes
  for(int i=0; i<keyframe_vector_ba_->size(); i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // iterate through projected active points
    for( ActivePointProjected* active_pt_proj : *(keyframe->regions_projected_active_points_->active_points_proj_)){

    }
  }

}

void BundleAdj::optimize(){

  double t_start=getTime();

  // marginalize
  marginalize();

  // get state sizes
  int pose_block_size=0, point_block_size=0;
  updateStateBlockIdxs(pose_block_size, point_block_size);

  // optimize
  optimize( pose_block_size, point_block_size );

  // after optimization, remove added_ba_ flag on keyframe

}

ActivePointProjected* BundleAdj::projectActivePoint(ActivePoint* active_pt, CamCouple* cam_couple){


    Eigen::Vector3f p;
    Eigen::Vector2f uv;
    Eigen::Vector2i pixel_coords;
    float depth_m;

    cam_couple->getD2(active_pt->uv_.x(), active_pt->uv_.y(), 1.0/active_pt->invdepth_, depth_m );
    cam_couple->getUv(active_pt->uv_.x(), active_pt->uv_.y(), 1.0/active_pt->invdepth_, uv.x(), uv.y() );

    cam_couple->cam_m_->uv2pixelCoords( uv, pixel_coords, active_pt->level_);

    if (active_pt->cam_->wavelet_dec_->vector_wavelets->at(active_pt->level_)->c->pixelInRange(pixel_coords)){
      ActivePointProjected* active_point_proj = new ActivePointProjected(active_pt, pixel_coords, uv, 1.0/depth_m, cam_couple->cam_m_ );
      return active_point_proj;
    }
    return nullptr;


}

void BundleAdj::updateCurrentGuess(){
  // update p_incamframe_
}

void BundleAdj::updateProjActivePoints(){
  int num_active_points_outside_frustum = 0;
  // iterate through all keyframe
  for (int i=0; i<keyframe_vector_ba_->size(); i++){
    CameraForMapping* cam_m = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    // iterate through projected active points
    for(ActivePointProjected* active_pt_proj : *(cam_m->regions_projected_active_points_->active_points_proj_)){
      ActivePoint* active_pt = active_pt_proj->active_point_;
      CameraForMapping* cam_r = active_pt->cam_;
      // point wrt world
      Eigen::Vector3f p = (*(cam_r->frame_camera_wrt_world_))*v2t_inv(*(cam_r->delta_update_x_))*(*(active_pt->p_incamframe_));
      // point wrt cam m
      Eigen::Vector3f p_cam_m = v2t(*(cam_r->delta_update_x_))*(*(cam_m->frame_world_wrt_camera_))*p;
      Eigen::Vector2f uv;
      cam_m->projectPointInCamFrame(p_cam_m, uv);

      Eigen::Vector2i pixel;
      cam_m->uv2pixelCoords(uv, pixel, active_pt->level_);

      if (active_pt->cam_->wavelet_dec_->vector_wavelets->at(active_pt->level_)->c->pixelInRange(pixel))
      {
        active_pt_proj->uv_=uv;
        active_pt_proj->pixel_=pixel;
        active_pt_proj->invdepth_=1.0/p_cam_m.z();
      }else{
        // active_pt_proj->marginalize(); TODO
        num_active_points_outside_frustum++;
      }

    }
  }
  if (num_active_points_outside_frustum)
    std::cout << "ATTENZIONEEEEEEEE, PUNTI FUORI CON OPT " << num_active_points_outside_frustum << std::endl;
}

void BundleAdj::projectActivePoints(){

  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  int n_active_points_not_observed =0;

  // iterate through all keyframe (except the last)
  for (int i=0; i<keyframe_vector_ba_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    if (keyframe->to_be_marginalized_ba_){
      if(!keyframe->active_points_removed_ ){
        num_active_points_-=(keyframe->active_points_->size()-keyframe->num_marginalized_active_points_);
        keyframe->active_points_removed_=true;
      }
      continue;
    }

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe);
    bool keyframe_link = false;
    // iterate along all active points
    for (ActivePoint* active_pt : *keyframe->active_points_){

      // if(active_pt->to_marginalize_){
      //   continue;
      // }

      // project active point in new keyframe
      ActivePointProjected* active_point_proj = projectActivePoint(active_pt, cam_couple);
      // if active point is in frustum
      if (active_point_proj!=nullptr){

        // push active point projected
        if(!keyframe_link){
          keyframe_link=true;
          // push keyframe link
          keyframe->keyframes_linked_->push_back(last_keyframe);
        }
        active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);
      }
      // otherwise
      else{
        // if already not seen in last keyframe, it has to be marginalized
        if(active_pt->not_seen_in_last_keyframe_){
          if(!active_pt->active_point_removed_){
            active_pt->marginalize();
            // active_pt->to_marginalize_=true;
            // active_pt->active_point_removed_=true;
            keyframe->num_marginalized_active_points_++;
            num_active_points_--;
          }
          continue;
        }
        // else mark as not seen
        else{
          active_pt->not_seen_in_last_keyframe_=true;
          n_active_points_not_observed++;
        }
      }


      // put it in the relative region

    }
  }

}

void BundleAdj::marginalizeActivePoints(){

}
