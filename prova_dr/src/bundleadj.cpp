#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include <assert.h>
#include "defs.h"
#include <Eigen/QR>

CameraForMapping* BundleAdj::getFrameCurrentBA(){
  return dtam_->camera_vector_->at(frame_current_ba);
}

void BundleAdj::getCoarseActivePoints(){
  int last_keyframe_idx = keyframe_vector_ba_->back(); // update frame current
  double t_start=getTime();

  CameraForMapping* last_keyframe=dtam_->camera_vector_->at(last_keyframe_idx);

  for (ActivePointProjected* active_pt_proj : *(last_keyframe->regions_projected_active_points_->active_points_proj_))
    addCoarseActivePointInRegion(active_pt_proj->active_point_);

  double t_end=getTime();
  int deltaTime=(t_end-t_start);

  sharedCoutDebug("   - Coarse points generated in frame "+std::to_string(last_keyframe_idx)+", time: "+std::to_string(deltaTime)+" ms");

}

void BundleAdj::activateNewPoints(){

  int last_keyframe_idx = keyframe_vector_ba_->back(); // update frame current


  double t_start=getTime();

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
      // addCoarseActivePointInRegion(active_pt);

      // active point
      num_activated++;
    }

    // if it's the last round
    if(reg_idx==reg_vec->size()-1){
      min_num_of_active_pts_per_region_++;
    }
  }


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

  getCoarseActivePoints();

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

          pxl pixel {reg->x_, reg->y_};
          Eigen::Vector2f uv;
          keyframe->pixelCoords2uv(pixel,uv, i);

          pixelIntensity c = keyframe->wavelet_dec_->getWavLevel(i)->c->evalPixel(reg->y_,reg->x_);
          // pixelIntensity c_dx = keyframe->wavelet_dec_->getWavLevel(i)->c_dx->evalPixel(reg->y_,reg->x_);
          // pixelIntensity c_dy = keyframe->wavelet_dec_->getWavLevel(i)->c_dy->evalPixel(reg->y_,reg->x_);
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


Eigen::Matrix<float,1,3>* BundleAdj::getJfirst(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Vector3f& point_m_0, pxl& pixel_m){
  CameraForMapping* cam_r = active_pt->cam_;

  float pixels_meter_ratio = dtam_->camera_vector_->at(0)->cam_parameters_->resolution_x/dtam_->camera_vector_->at(0)->cam_parameters_->width;
  Eigen::Matrix3f K = *(cam_m->K_);
  float coeff = pixels_meter_ratio/pow(2,active_pt->level_+1);

  // variables
  Eigen::Vector2f uv_m_0;
  pxl pixel_m_0;
  Eigen::Vector2f uv_m;
  Eigen::Vector3f* p_incamframe = active_pt->p_incamframe_;


  point_m_0= *(cam_m->frame_world_wrt_camera_0_)*(*active_pt->p_0_);
  Eigen::Vector3f point_m= (v2t(*(cam_m->delta_update_x_))*((*(cam_m->frame_world_wrt_camera_))*(*active_pt->p_)));

  Eigen::Vector3f p_proj_0 = K*point_m_0;
  // Eigen::Vector3f p_proj_0 = K*point_m_0;

  // return false if the projected point is behind the camera
  if (p_proj_0.z()<cam_m->cam_parameters_->lens)
    return nullptr;
  uv_m_0 = p_proj_0.head<2>()*(1./p_proj_0.z());

  cam_m->projectPointInCamFrame( point_m_0, uv_m_0 );
  cam_m->uv2pixelCoords(uv_m_0, pixel_m_0, active_pt->level_);

  cam_m->projectPointInCamFrame( point_m, uv_m );
  cam_m->uv2pixelCoords(uv_m, pixel_m, active_pt->level_);

  if(!cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->pixelInRange(pixel_m_0)){
    // std::cout << "pixel_m_0" << " " << pixel_m_0 << std::endl;
    return nullptr;}

  if(!cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->pixelInRange(pixel_m)){
    // std::cout << "pixel_m" << " " << pixel_m << std::endl;
    return nullptr;}


  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;

  proj_jacobian << 1./p_proj_0.z(), 0, -p_proj_0.x()/pow(p_proj_0.z(),2),
                   0, 1./p_proj_0.z(), -p_proj_0.y()/pow(p_proj_0.z(),2);

  Eigen::Matrix<float, 1,2> img_jacobian;
  img_jacobian << cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dx->evalPixelBilinear(pixel_m), cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dy->evalPixelBilinear(pixel_m);

  Eigen::Matrix<float,1,3>* J_first = new Eigen::Matrix<float,1,3>;
  *J_first = coeff*((img_jacobian*proj_jacobian)*K);

  return J_first;
}


Eigen::Matrix<float,1,6>* BundleAdj::getJm(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first, Eigen::Vector3f& point_m_0){

  Eigen::Matrix<float, 3,6> state_jacobian;

  state_jacobian << 1, 0, 0,  0             ,  point_m_0.z()  , -point_m_0.y(),
                    0, 1, 0, -point_m_0.z() ,  0              ,  point_m_0.x(),
                    0, 0, 1,  point_m_0.y() , -point_m_0.x()  ,  0         ;


  Eigen::Matrix<float,1,6>* J_m = new Eigen::Matrix<float,1,6>;
  // *J_m=(coeff*(img_jacobian*jacobian_to_mul))/normalizer;
  *J_m=(*J_first)*state_jacobian;


  return J_m;
}

Eigen::Matrix<float,1,6>* BundleAdj::getJr(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first){
  CameraForMapping* cam_r = active_pt->cam_;
  Eigen::Vector3f point_r_0 = *(active_pt->p_incamframe_0_);
  Eigen::Matrix<float, 3,6> state_jacobian;

  state_jacobian <<  -1,  0,  0,  0             , -point_r_0.z()  ,  point_r_0.y(),
                      0, -1,  0,  point_r_0.z() ,  0              , -point_r_0.x(),
                      0,  0, -1, -point_r_0.y() ,  point_r_0.x()  ,  0            ;


  Eigen::Matrix<float,1,6>* J_r = new Eigen::Matrix<float,1,6>;
  // *J_r=((*J_first)*state_jacobian)/normalizer;
  *J_r=((*J_first)*(cam_m->frame_world_wrt_camera_0_->linear())*(cam_r->frame_camera_wrt_world_0_->linear()))*state_jacobian;

  return J_r;
}

float BundleAdj::getJd(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first){
  CameraForMapping* cam_r = active_pt->cam_;
  Eigen::Matrix3f Kinv = *(cam_r->Kinv_);
  float pixels_meter_ratio = dtam_->camera_vector_->at(0)->cam_parameters_->resolution_x/dtam_->camera_vector_->at(0)->cam_parameters_->width;

  Eigen::Matrix<float, 3,1> invdepth_jacobian;

  invdepth_jacobian << -active_pt->uv_.x()/pow(active_pt->invdepth_0_,2),
                       -active_pt->uv_.y()/pow(active_pt->invdepth_0_,2),
                       -1/pow(active_pt->invdepth_0_,2);

  float J_d =(((*J_first)*(cam_m->frame_world_wrt_camera_0_->linear()))*(cam_r->frame_camera_wrt_world_0_->linear())*Kinv )*invdepth_jacobian;

  // Eigen::Matrix<float, 3,1> depth_jacobian;
  //
  // depth_jacobian << active_pt->uv_.x(),
  //                   active_pt->uv_.y(),
  //                   1;
  //
  // float J_d =(( (*J_first)*(cam_m->frame_world_wrt_camera_0_->linear()))*(cam_r->frame_camera_wrt_world_0_->linear())*Kinv )*depth_jacobian;

  return J_d;
}

void HessianAndB_base::mirrorTriangH(){
  *H_point_pose=H_pose_point->transpose();
  for (int m=0 ; m<pose_block_size ; m+=6){
    for (int r=0 ; r<m ; r+=6){
      H_pose_pose->block(r,m,6,6)=H_pose_pose->block(m,r,6,6).transpose();
    }
  }


}

void HessianAndB::updateHessianAndB(JacobiansAndError* jacobians_and_error ){

  assert(jacobians_and_error!=nullptr);

  Eigen::Matrix<float,1,6> J_r = *(jacobians_and_error->J_r);
  Eigen::Matrix<float,1,6> J_m = *(jacobians_and_error->J_m);
  float J_d = jacobians_and_error->J_d;
  Eigen::Matrix<float,6,1> J_r_transp = *(jacobians_and_error->J_r_transp);
  Eigen::Matrix<float,6,1> J_m_transp = *(jacobians_and_error->J_m_transp);
  float error = jacobians_and_error->error;
  float weight_total = jacobians_and_error->weight_total;

  int r = jacobians_and_error->active_pt->cam_->state_pose_block_idx_;
  int m = jacobians_and_error->cam_m->state_pose_block_idx_;
  int d = jacobians_and_error->active_pt->state_point_block_idx_;

  assert(d>-1 && d<point_block_size);
  assert(m>=-1 && m<pose_block_size);
  assert(r>=-1 && r<pose_block_size);

  // need for dealing with fixed first keyframe
  bool r_flag = r!=-1;
  bool m_flag = m!=-1;


  // ********** update H **********
  // pose pose block
  // O-
  // --
  if(m_flag)
    H_pose_pose->block(m,m,6,6)+=J_m_transp*weight_total*J_m;
  if(r_flag)
    H_pose_pose->block(r,r,6,6)+=J_r_transp*weight_total*J_r;
  if(m_flag && r_flag){
    H_pose_pose->block(m,r,6,6)+=J_m_transp*weight_total*J_r;
    // H_pose_pose->block(r,m,6,6)+=J_r_transp*weight_total*J_m;
  }


  // pose point block
  // -O
  // --
  if(m_flag)
    H_pose_point->block(m,d,6,1)+=J_m_transp*(weight_total*J_d);
  if(r_flag)
    H_pose_point->block(r,d,6,1)+=J_r_transp*(weight_total*J_d);
  // H_pose_point->block(m,d,6,1)+=J_m*(weight_total*J_d);
  // H_pose_point->block(r,d,6,1)+=J_r*(weight_total*J_d);



  // point point block
  // --
  // -O
  H_point_point->diagonal()[d]+=J_d*weight_total*J_d;

  // ********** update b **********
  // pose block
  if(m_flag)
    b_pose->segment(m,6)+=J_m*(weight_total*error);
  if(r_flag)
    b_pose->segment(r,6)+=J_r*(weight_total*error);

  // point block
  (*b_point)(d)+=J_d*(weight_total*error);

}

// void HessianAndB_Marg::updateHessianAndB_marg(JacobiansAndError* jacobians_and_error ){
//
//   assert(jacobians_and_error!=nullptr);
//   assert(jacobians_and_error->J_r==nullptr);
//   assert(jacobians_and_error->J_m!=nullptr);
//
//   Eigen::Matrix<float,1,6> J_m = *(jacobians_and_error->J_m);
//   float J_d = jacobians_and_error->J_d;
//   Eigen::Matrix<float,6,1> J_m_transp = *(jacobians_and_error->J_m_transp);
//   float error = jacobians_and_error->error;
//   float weight_total = jacobians_and_error->weight_total;
//
//   int m = jacobians_and_error->cam_m->state_pose_block_marg_idx_;
//   int d = jacobians_and_error->active_pt->state_point_block_marg_idx_;
//
//   // need for dealing with fixed first keyframe
//   assert(d>-1 && d<point_block_size);
//   assert(m>-1 && m<pose_block_size);
//
//   // ********** update H **********
//   // pose pose block
//   // O-
//   // --
//   H_pose_pose->block(m,m,6,6)+=J_m_transp*weight_total*J_m;
//   // std::cout <<
//   // std::cout << "aooo\n"<< J_r_transp*J_m << std::endl;
//
//   // pose point block
//   // -O
//   // --
//   H_pose_point->block(m,d,6,1)+=J_m_transp*(weight_total*J_d);
//   // H_pose_point->block(m,d,6,1)+=J_m*(weight_total*J_d);
//
//   // point point block
//   // --
//   // -O
//   H_point_point->diagonal()[d]+=J_d*weight_total*J_d;
//
//   // ********** update b **********
//   // pose block
//   b_pose->segment(m,6)+=J_m*(weight_total*error);
//
//   // point block
//   (*b_point)(d)+=J_d*(weight_total*error);
//
// }


float BundleAdj::getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m){
  float z = active_pt->intensity_;
  float z_hat = cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->evalPixelBilinear(pixel_m);

  // error
  // float error = (z_hat-z)/normalizer;
  float error = (z_hat-z);
  return error;
}

bool BundleAdj::getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, float& error){
  float z = active_pt->intensity_;
  float z_hat = cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->evalPixelBilinear(pixel_m);

  // error
  // float error = (z_hat-z)/normalizer;
  error = (z_hat-z);

  float error_eps = 0.00001;
  // // saturate error
  // if (error<error_eps)
  //   error=error_eps;

  // // add error eps
  // error+=error_eps;

  // do not consider measurement
  if (error<error_eps)
  // if (error==0)
    return false;

  return true;
}


float BundleAdj::getWeightTotal(float error){
  // robustifier
  float u = abs(error);
  float rho_der = huberNormDerivative(u,dtam_->parameters_->huber_threshold);
  float gamma=(1/(u+0.0001))*rho_der;


  // weight
  float variance = dtam_->parameters_->variance;
  int ni = dtam_->parameters_->robustifier_dofs;
  float weight = (ni+1.0)/(ni+(pow(error,2)/variance));

  float  weight_total = weight*gamma;

  // return weight_total;
  return 1;
}


JacobiansAndError* BundleAdj::getJacobiansAndError(ActivePoint* active_pt, CameraForMapping* cam_m, bool marg){


  Eigen::Matrix<float,1,3>* J_1;
  Eigen::Vector3f point_m_0;
  pxl pixel_m;
  J_1= getJfirst( active_pt, cam_m,  point_m_0, pixel_m);
  if(debug_optimization_){
    // Image<float>* img_r = active_pt->cam_->wavelet_dec_->vector_wavelets->at(active_pt->level_)->c;
    // Image<float>* img_m = cam_m->wavelet_dec_->vector_wavelets->at(active_pt->level_)->c;
    // img_r->showImgWithColoredPixel(active_pt->pixel_,pow(2,active_pt->level_+1), "img_r");
    // img_m->showImgWithColoredPixel(pixel_m,pow(2,active_pt->level_+1), "img_m");
    // cv::waitKey(0);
  }

  // if J_1 is nullptr, point is outside the frustum of cam m
  if (J_1==nullptr)
    return nullptr;

  float error = getError( active_pt, cam_m ,pixel_m );

  float  weight_total = getWeightTotal(error);

  Eigen::Matrix<float,1,6>* J_r = nullptr;
  if (!marg)
    J_r = getJr( active_pt, cam_m, J_1);

  Eigen::Matrix<float,1,6>* J_m = getJm( active_pt, cam_m, J_1, point_m_0);
  float J_d = getJd(active_pt, cam_m, J_1);

  // if(J_d*J_d<0.001)
  //   return nullptr;

  JacobiansAndError* jacobians = new JacobiansAndError(J_r,J_m,J_d,cam_m,active_pt, error, weight_total);

  return jacobians;

}

Eigen::DiagonalMatrix<float,Eigen::Dynamic>* HessianAndB_base::invertHPointPoint(){
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size);

  // threshold based on paper
  // "The Optimal Hard Threshold for Singular Values is 4/√3"
  float sv_sum = 0;
  for(int i=0; i<point_block_size; i++)
    sv_sum += H_point_point->diagonal()(i);
  float sv_average = sv_sum/point_block_size;

  float thresh = 2.858*sv_average;

  for(int i=0; i<point_block_size; i++){
    float val = H_point_point->diagonal()(i);
    if(val>thresh)
    // if(val!=0)
      H_point_point_inv->diagonal()(i)=1.0/val;
    else
      H_point_point_inv->diagonal()(i)=0;

  }
  return H_point_point_inv;
}

Eigen::DiagonalMatrix<float,Eigen::Dynamic>* HessianAndB_base::invertHPointPointDLS(float mu){
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size);
  for(int i=0; i<point_block_size; i++){
    float val = H_point_point->diagonal()(i);
    float val2 = val*val;
    if(val!=0)
      H_point_point_inv->diagonal()(i)=val/(val*val+mu);
  }
  return H_point_point_inv;
}


deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrements(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;

  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint();

  Eigen::MatrixXf Schur=(*H_pose_pose)-((*H_pose_point)*(*H_point_point_inv)*(*H_point_pose));
  // Eigen::MatrixXf Schur_inv=Schur.inverse();
  Eigen::MatrixXf Schur_inv = Schur.completeOrthogonalDecomposition().pseudoInverse();


  *dx_poses =  Schur_inv * ( -(*b_pose) + (*H_pose_point)*(*H_point_point_inv)*(*b_point) );
  *dx_points = (*H_point_point_inv)* ( -(*b_point) -( (*H_point_pose)*(*dx_poses)) );

  delete H_point_point_inv;

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}


deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrements_Slow(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;


  Eigen::MatrixXf H(pose_block_size+point_block_size,pose_block_size+point_block_size);
  H.block(0,0,pose_block_size,pose_block_size)=*H_pose_pose;
  H.block(0,pose_block_size,pose_block_size,point_block_size)=*H_pose_point;
  H.block(pose_block_size,0,point_block_size,pose_block_size)=*H_point_pose;
  H.block(pose_block_size,pose_block_size,point_block_size,point_block_size)=*H_point_point;

  Eigen::VectorXf b(pose_block_size+point_block_size);
  b.segment(0,pose_block_size)=*b_pose;
  b.segment(pose_block_size,point_block_size)=*b_point;

  Eigen::VectorXf d(pose_block_size+point_block_size);

  Eigen::MatrixXf Hpinv = H.completeOrthogonalDecomposition().pseudoInverse();

  // d=-(H.inverse())*b ;
  d=-Hpinv*b ;

  *dx_poses  =  d.segment(0,pose_block_size);
  *dx_points =  d.segment(pose_block_size,point_block_size);

  // std::cout << *dx_poses << std::endl;

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}


void HessianAndB::visualizeH(){
  Image<colorRGB>* img_H = new Image<colorRGB>("Hessian");
  int size = pose_block_size+point_block_size;
  // img_H->initImage(pose_block_size,pose_block_size);
  img_H->initImage(size,size);
  img_H->setAllPixels( white);

  // pose pose block
  for(int i=0; i<pose_block_size; i++){
    for(int j=0; j<pose_block_size; j++){
      if ((*H_pose_pose)(i,j)!=0){
        img_H->setPixel(i,j, red);
      }
      else{
        img_H->setPixel(i,j, white);

      }
    }
  }

  // pose point block
  for(int i=0; i<pose_block_size; i++){
    for(int j=0; j<point_block_size; j++){
      if ((*H_pose_point)(i,j)!=0){
        img_H->setPixel(i,pose_block_size+j, green);
      }
      else{
        img_H->setPixel(i,pose_block_size+j, white);

      }
    }
  }

  // point pose block
  for(int i=0; i<point_block_size; i++){
    for(int j=0; j<pose_block_size; j++){
      if ((*H_point_pose)(i,j)!=0){
        img_H->setPixel(i+pose_block_size,j, green);
      }
      else{
        img_H->setPixel(i+pose_block_size,j, white);

      }
    }
  }

  // point point block
  for(int i=0; i<point_block_size; i++){
    if ((H_point_point->diagonal())(i)!=0){
      img_H->setPixel(i+pose_block_size,i+pose_block_size, blue);
    }
    else{
      img_H->setPixel(i+pose_block_size,i+pose_block_size, white);

    }

  }

  img_H->show(1);
}




void BundleAdj::updateTangentSpace(){
  // fix tangent space for cameras
  for(int i=0; i<keyframe_vector_ba_->size() ; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->state_pose_block_idx_!=-1){
      // if active keyframe has link with marginalization prior, do not update tangent space
      if(keyframe->state_pose_block_marg_idx_!=-1){
        continue;
      }
      keyframe->assignPose0( *(keyframe->frame_camera_wrt_world_) );
      keyframe->delta_update_x_->setZero();
    }
  }

  // fix tangent space for points
  for(int i=0; i<keyframe_vector_ba_->size() ; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->state_pose_block_idx_!=-1){

      // iterate through all active points
      for(int j=0; j<keyframe->active_points_->size() ; j++){
        ActivePoint* active_pt = keyframe->active_points_->at(j);
        active_pt->invdepth_0_=active_pt->invdepth_;
        active_pt->delta_update_x_=0;
      }
    }
  }
}


void BundleAdj::updateDeltaUpdates(deltaUpdateIncrements* delta){
  // UPDATE CAMERAS
  if(delta->dx_poses!=nullptr)
    for(int i=0; i<keyframe_vector_ba_->size() ; i++){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
      if (keyframe->state_pose_block_idx_!=-1){
        // update delta update of keyframe
        // regular sum in tangent space
        // if(!keyframe->first_keyframe_){
          (*keyframe->delta_update_x_)+=delta->dx_poses->segment(keyframe->state_pose_block_idx_,6);
          // std::cout << "CHAIOHCAEO\n" << (*keyframe->delta_update_x_) << "\n" << std::endl;

          Eigen::Isometry3f frame_camera_wrt_world =(*(keyframe->frame_camera_wrt_world_0_))*v2t_inv(*(keyframe->delta_update_x_));
          keyframe->assignPose( frame_camera_wrt_world );
        // }

      }
    }

  // UPDATE POINTS
  // if(delta->dx_points!=nullptr){
    for(int i=0; i<keyframe_vector_ba_->size() ; i++){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
      if (keyframe->state_pose_block_idx_!=-1){

        // iterate through all active points
        for(int j=0; j<keyframe->active_points_->size() ; j++){
          ActivePoint* active_pt = keyframe->active_points_->at(j);
          // update delta update of active point
          // regular sum since invdepth is in R

          if(delta->dx_points!=nullptr){
            active_pt->delta_update_x_+=(*(delta->dx_points))(active_pt->state_point_block_idx_);
            // std::cout << active_pt->invdepth_0_ << " " << active_pt->delta_update_x_ << std::endl;
            active_pt->invdepth_=active_pt->invdepth_0_+active_pt->delta_update_x_;
            // active_pt->invdepth_=1.0/((1.0/active_pt->invdepth_0_)+active_pt->delta_update_x_);
          }

          // since change both poses and invdepths
          keyframe->pointAtDepthInCamFrame(active_pt->uv_, 1.0/active_pt->invdepth_, *(active_pt->p_incamframe_));
          *(active_pt->p_)=(*(keyframe->frame_camera_wrt_world_0_))*v2t_inv(*(keyframe->delta_update_x_))*(*(active_pt->p_incamframe_));
          // propagate uncertainty TODO
        }
      }
    }

}

void HessianAndB_Marg::curr2old(){
  delete hessian_b_marg_old;
  hessian_b_marg_old = new HessianAndB_base(this);
  deleteAllPtrs();
}


bool BundleAdj::updateOldHessianAndB(){

  // if i have no H and b, i have no old structure
  if(hessian_b_marg==nullptr)
    return false;

  // current become old H and b
  hessian_b_marg->curr2old();
  HessianAndB_base* hessian_b_old = hessian_b_marg->hessian_b_marg_old;

  std::vector<Eigen::VectorXf> pose_point_cols;
  std::vector<float> point_point_vals;
  std::vector<Matrix6f> pose_pose_blocks;

  // ************* set old cam marg idxs, get row to marg *************

  int poses_old_size = 0;
  int row_pose_to_be_marginalized = -1;
  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    int block_idx = keyframe->state_pose_block_marg_idx_;
    // if keyframe is not the one that is going to be marginalized, and keyframe has link with marg prior
    if (!keyframe->to_be_marginalized_ba_ && block_idx!=-1 ){
      Matrix6f pose_pose_block = hessian_b_old->H_pose_pose->block<6,6>(block_idx,block_idx);
      pose_pose_blocks.push_back(pose_pose_block);
      keyframe->state_pose_block_marg_idx_=poses_old_size;
      poses_old_size+=6;
    }
    // save initial row of pose block to marginalize
    else if(keyframe->to_be_marginalized_ba_){
      row_pose_to_be_marginalized = keyframe->state_pose_block_marg_idx_;
      keyframe->state_pose_block_marg_idx_=-1;
    }

  }

  assert(row_pose_to_be_marginalized!=-1);

  // ************* structure of old variables *************

  int n_points_old_size = 0;
  int old_point_block_size = hessian_b_old->point_block_size;
  int old_pose_block_size = hessian_b_old->pose_block_size;


  // iterate through old points (cols)
  for(int col=0; col<old_point_block_size; col++){
    // column to save has dimension of old pose block size - 6 for the marginalized cam
    Eigen::VectorXf column(old_pose_block_size-6);

    int vec_upper_part_size = row_pose_to_be_marginalized;
    int vec_lower_part_size = old_pose_block_size-row_pose_to_be_marginalized-6;

    column.head(vec_upper_part_size)= hessian_b_old->H_pose_point->block(0,col,vec_upper_part_size,1);
    column.tail(vec_lower_part_size)= hessian_b_old->H_pose_point->block(vec_upper_part_size+6,col,vec_lower_part_size,1);

    // if col is not zero
    if (!column.isZero()){
      pose_point_cols.push_back(column);  // push column
      point_point_vals.push_back(hessian_b_old->H_point_point->diagonal()[col]); // push same old point point diagonal value
      n_points_old_size++; // increment n old points size
    }
    // otherwise that marginalized point is automatically removed inside the factor graph
  }

  // ************* update old hessian and b *************

  // create new hessiand and b class and delete the old one
  delete hessian_b_old;
  hessian_b_old = new HessianAndB_base(pose_pose_blocks.size(),pose_point_cols.size());

  // update old pose pose block
  for (int i=0; i<pose_pose_blocks.size(); i++){
    Matrix6f pose_pose_block = pose_pose_blocks.at(i);
    hessian_b_old->H_pose_pose->block<6,6>(i*6,i*6);
  }

  // update old pose point block
  for (int i=0; i<pose_point_cols.size(); i++){
    Eigen::VectorXf col = pose_point_cols.at(i);
    hessian_b_old->H_pose_point->col(i)=col;
  }

  // update old point point block
  for (int i=0; i<point_point_vals.size(); i++){
    float val = point_point_vals.at(i);
    hessian_b_old->H_point_point->diagonal()[i]=val;
  }

  return true;
}

void BundleAdj::getDataForNewUpdate(){

  int new_points = 0;
  int poses_new_size = 0;

  int poses_old_size = hessian_b_marg->hessian_b_marg_old->H_pose_pose->rows();

  std::vector<JacobiansAndError*>* jacobians_and_error_vec = new std::vector<JacobiansAndError*>;

  // ********** collect Jacobians **********

  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    // iterate through points to be marginalized
    for( int j=0; j<keyframe->marginalized_points_->size(); j++){
      ActivePoint* pt_to_be_marg = keyframe->active_points_->at(j);
      bool point_taken = false;
      // iterate through keyframes on which the active point is projected
      for(int k=0; k<keyframe_vector_ba_->size() ; k++){

        // avoid self projection
        if(k==i)
          continue;

        CameraForMapping* keyframe_proj = dtam_->camera_vector_->at(keyframe_vector_ba_->at(k));

        if(keyframe_proj->to_be_marginalized_ba_){
          continue;
        }

        JacobiansAndError* jacobians = getJacobiansAndError(pt_to_be_marg, keyframe_proj, true);
        if (jacobians==nullptr){
          // n_suppressed_measurement++;
        }
        else{
          // if pose is new inside the marginalization part
          if(keyframe_proj->state_pose_block_marg_idx_==-1){
            keyframe_proj->state_pose_block_marg_idx_=poses_old_size+poses_new_size*6;
            poses_new_size++;
          }

          point_taken = true;
          jacobians_and_error_vec->push_back(jacobians);
        }
      }

      if(point_taken){
        new_points++;
      }

    }

  }

  // **********  **********


}

void BundleAdj::deleteMarginalizedPoints(){
  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // iterate through points to be marginalized
    for( int j=0; j<keyframe->marginalized_points_->size(); j++){
      ActivePoint* pt_to_be_marg = keyframe->active_points_->at(j);
    }
  }
}

void BundleAdj::initializeStateStructureMarg( int& n_cams, int& n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec, CameraForMapping* keyframe_to_marginalize ){
  // iterate through keyframes with active points (except last)
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // iterate through projected active points

    bool point_taken = false;
    // iterate through points to be marginalized
    for( int j=0; j<keyframe->marginalized_points_->size(); j++){
      ActivePoint* pt_to_be_marg = keyframe->active_points_->at(j);

      // iterate through keyframes on which the active point is projected
      for(int k=0; k<keyframe_vector_ba_->size() ; k++){

        // avoid self projection
        if(k==i)
          continue;

        CameraForMapping* keyframe_proj = dtam_->camera_vector_->at(keyframe_vector_ba_->at(k));

        if(keyframe_proj->to_be_marginalized_ba_){
          continue;
        }


        JacobiansAndError* jacobians = getJacobiansAndError(pt_to_be_marg, keyframe_proj, true);
        if (jacobians==nullptr){
          // n_suppressed_measurement++;
        }
        else{
          if(keyframe_proj->state_pose_block_marg_idx_==-1){
            keyframe_proj->state_pose_block_marg_idx_=n_cams*6;
            n_cams++;
          }

          point_taken=true;
          jacobians_and_error_vec->push_back(jacobians);
        }

      }
      if(point_taken){
        pt_to_be_marg->state_point_block_marg_idx_=n_points_marg;
        n_points_marg++;
      }
      else{
        pt_to_be_marg->remove();
        j--;
      }
    }

    if (keyframe->to_be_marginalized_ba_)
      keyframe_to_marginalize = keyframe;
  }
}


void priorMarg::build(int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec){

}

priorMarg* BundleAdj::updateMarginalizationPrior(int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec){
  priorMarg* marg_prior = new priorMarg(n_cams,n_points_marg);
  // marg_prior->build(n_cams,n_points_marg,jacobians_and_error_vec );

}


void BundleAdj::marginalize( ){

  int n_cams=0;
  int n_points_marg=0;
  std::vector<JacobiansAndError*>* jacobians_and_error_vec = new std::vector<JacobiansAndError*>;

  CameraForMapping* keyframe_to_marginalize = nullptr;
  initializeStateStructureMarg(n_cams,n_points_marg,jacobians_and_error_vec, keyframe_to_marginalize);

  // build prior
  // updateMarginalizationPrior(int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec);



  // // create Hessian and b vector
  // HessianAndB* hessian_b = new HessianAndB(n_cams*6, n_points);
  // // for each measurement update Hessian and b vector
  // for(int i=0; i<jacobians_and_error_vec->size(); i++){
  //   JacobiansAndError* jacobians_and_error = jacobians_and_error_vec->at(i);
  //   hessian_b->updateHessianAndB( jacobians_and_error );
  //
  //   chi+=(jacobians_and_error->error*jacobians_and_error->error);
  //
  //   delete jacobians_and_error;
  // }
  // hessian_b->mirrorTriangH();
  //
  // // std::cout << (*hessian_b->H_pose_point)-(hessian_b->H_point_pose->transpose()) << std::endl;
  // // std::cout << (*hessian_b->H_pose_point) << std::endl;
  // hessian_b->visualizeH();
  // cv::waitKey(0);
  // delete jacobians_and_error_vec;


  // remove keyframe to marginalize

  // compute deltas

  // remember to set all state_point_block_marg_idx_ to -1

}


void BundleAdj::initializeStateStructure( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec ){

  // iterate through keyframes with active points (except last)
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // if keyframe is going to be marginalized, this step needs to be performed in marginalization procedure
    if(keyframe->to_be_marginalized_ba_){
      continue;
    }
    bool cam_taken = false;
    // iterate through active points
    for( int j=0; j<keyframe->active_points_->size(); j++){
      ActivePoint* active_pt = keyframe->active_points_->at(j);

      bool point_taken = false;
      bool not_seen = false;
      // iterate through keyframes on which the active point is projected
      for(int k=0; k<keyframe_vector_ba_->size() ; k++){

        // avoid self projection
        if(k==i)
          continue;

        CameraForMapping* keyframe_proj = dtam_->camera_vector_->at(keyframe_vector_ba_->at(k));

        if(keyframe_proj->to_be_marginalized_ba_){
          continue;
        }



        JacobiansAndError* jacobians = getJacobiansAndError(active_pt, keyframe_proj);
        if (jacobians==nullptr){

        }
        else{
          point_taken=true;
          if(!cam_taken && !keyframe->first_keyframe_)
            cam_taken=true;
          jacobians_and_error_vec->push_back(jacobians);
        }
      }
      if(point_taken){
        active_pt->state_point_block_idx_=n_points;
        n_points++;
      }
      else{
        active_pt->state_point_block_idx_=-1;
      }
    }
    if (!cam_taken){
      keyframe->state_pose_block_idx_=-1;
    }
    else{
      keyframe->state_pose_block_idx_=n_cams*6;
      n_cams++;
    }
  }
  CameraForMapping* keyframe_last = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  keyframe_last->state_pose_block_idx_=n_cams*6;
  n_cams++;
}




float BundleAdj::optimizationStep( ){

  float chi = 0;

  int n_suppressed_measurement = 0;
  std::vector<JacobiansAndError*>* jacobians_and_error_vec = new std::vector<JacobiansAndError*> ;

  int n_cams = 0;
  int n_points = 0;
  initializeStateStructure( n_cams, n_points, jacobians_and_error_vec );

  // create Hessian and b vector
  HessianAndB* hessian_b = new HessianAndB(n_cams*6, n_points);
  // for each measurement update Hessian and b vector
  for(int i=0; i<jacobians_and_error_vec->size(); i++){
    JacobiansAndError* jacobians_and_error = jacobians_and_error_vec->at(i);
    hessian_b->updateHessianAndB( jacobians_and_error );

    chi+=(jacobians_and_error->error*jacobians_and_error->error);

    delete jacobians_and_error;
  }
  hessian_b->mirrorTriangH();

  // std::cout << (*hessian_b->H_pose_point)-(hessian_b->H_point_pose->transpose()) << std::endl;
  // std::cout << (*hessian_b->H_pose_point) << std::endl;
  hessian_b->visualizeH();
  cv::waitKey(0);
  delete jacobians_and_error_vec;


  // std::cout << "NUM SUPPRESSED MEASUREMENT: " << n_suppressed_measurement << std::endl;

  // get delta update
  deltaUpdateIncrements* delta = hessian_b->getDeltaUpdateIncrements();
  // deltaUpdateIncrements* delta = hessian_b->getDeltaUpdateIncrements_Slow();


  delete hessian_b;


  // update x in each cam and active point
  updateDeltaUpdates(delta);

  // update fixed points
  updateTangentSpace();

  return chi;
}

void BundleAdj::optimize(){

  double t_start=getTime();

  // marginalize
  // marginalize();

  // optimize
  // while(true){
  for(int i=0; i<6; i++){
    float chi = optimizationStep( );
    if(debug_optimization_){
      CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());

      last_keyframe->clearProjectedActivePoints();
      bool take_fixed_point = 1;
      projectActivePoints(take_fixed_point);
      last_keyframe->showProjActivePoints(1);
      std::cout << "chi: " << chi << std::endl;
      cv::waitKey(0);
    }
  }
  getCoarseActivePoints();

  // after optimization, remove added_ba_ flag on keyframe

}

ActivePointProjected* BundleAdj::projectActivePoint(ActivePoint* active_pt, CamCouple* cam_couple){

  Eigen::Vector3f p;
  Eigen::Vector2f uv;
  pxl pixel_coords;
  float depth_m;
  float depth_r;

  if(cam_couple->take_fixed_point_){
    depth_r= 1.0/active_pt->invdepth_0_;}
  else
    depth_r= 1.0/active_pt->invdepth_;

  cam_couple->getD2(active_pt->uv_.x(), active_pt->uv_.y(), depth_r, depth_m );
  cam_couple->getUv(active_pt->uv_.x(), active_pt->uv_.y(), depth_r, uv.x(), uv.y() );

  cam_couple->cam_m_->uv2pixelCoords( uv, pixel_coords, active_pt->level_);

  if (active_pt->cam_->wavelet_dec_->vector_wavelets->at(active_pt->level_)->c->pixelInRange(pixel_coords)){
    ActivePointProjected* active_point_proj = new ActivePointProjected(active_pt, pixel_coords, uv, 1.0/depth_m, cam_couple->cam_m_ );
    return active_point_proj;
  }
  return nullptr;


}

void BundleAdj::projectActivePoints_prepMarg(bool take_fixed_point){
  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  CameraForMapping* prev_last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->size()-2);

  // iterate through all keyframe (except the last)
  for (int i=0; i<keyframe_vector_ba_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    if (keyframe->to_be_marginalized_ba_){
      // iterate along all active points
      num_active_points_-=(keyframe->active_points_->size()-keyframe->num_marginalized_active_points_);
      for (ActivePoint* active_pt : *keyframe->active_points_){
        active_pt->marginalize();
      }
      // if(!keyframe->active_points_removed_ ){
      //   keyframe->active_points_removed_=true;
      // }
      continue;
    }

    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe,take_fixed_point);
    CamCouple* cam_couple_prev = new CamCouple(keyframe,prev_last_keyframe,take_fixed_point);

    // iterate along all active points
    for (ActivePoint* active_pt : *keyframe->active_points_){

      // project active point in new keyframe
      ActivePointProjected* active_point_proj = projectActivePoint(active_pt, cam_couple);
      // if active point is in frustum
      if (active_point_proj!=nullptr){
        // push active point projected
        active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);
      }
      // otherwise
      else{
        // project also in previous keyframe
        ActivePointProjected* active_point_proj_prev = projectActivePoint(active_pt, cam_couple_prev);
        if (active_point_proj==nullptr){
          active_pt->marginalize();
          num_active_points_--;
          keyframe->num_marginalized_active_points_++;
        }

      }

    }
  }
}


void BundleAdj::projectActivePoints(bool take_fixed_point){

  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());

  // iterate through all keyframe (except the last)
  for (int i=0; i<keyframe_vector_ba_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));


    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe,take_fixed_point);

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
        active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);
      }


    }
  }

}



void BundleAdj::marginalizeActivePoints(){

}
