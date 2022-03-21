#include "dtam.h"
#include "bundleadj.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include <assert.h>
#include "defs.h"
#include "plot_functions.h"
#include "parameters.h"
#include <Eigen/QR>
#include <Eigen/Eigenvalues>


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

Eigen::Matrix<float,1,2> BundleAdj::getImageJacobian(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, int image_id){

  Eigen::Matrix<float, 1,2> img_jacobian;

  if(image_id==INTENSITY_ID){
    float coeff = parameters_->intensity_coeff;
    img_jacobian << coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dx->evalPixelBilinear(pixel_m), coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c_dy->evalPixelBilinear(pixel_m);
  }
  else if(image_id==GRADIENT_ID){
    float coeff = parameters_->gradient_coeff;
    img_jacobian << coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->magn_cd_dx->evalPixelBilinear(pixel_m), coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->magn_cd_dy->evalPixelBilinear(pixel_m);
  }
  else if(image_id==PHASE_ID){
    float coeff = parameters_->phase_coeff;
    img_jacobian << coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->phase_cd_dx->evalPixelBilinear(pixel_m), coeff*cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->phase_cd_dy->evalPixelBilinear(pixel_m);
  }

  return img_jacobian;

}


Eigen::Matrix<float, 2,3>* BundleAdj::getJfirst_(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Vector3f& point_m_0, pxl& pixel_m){
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
    return nullptr;}

  if(!cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->pixelInRange(pixel_m)){
    return nullptr;}

  Eigen::Matrix<float,2,3>* J_first_ = new Eigen::Matrix<float,2,3>;

  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;

  proj_jacobian << 1./p_proj_0.z(), 0, -p_proj_0.x()/pow(p_proj_0.z(),2),
                   0, 1./p_proj_0.z(), -p_proj_0.y()/pow(p_proj_0.z(),2);


  *J_first_ = coeff*((proj_jacobian)*K);
  assert(J_first_->allFinite());

  // if(J_first_->norm()>10){
  //   std::cout << "JFIRSTTTTTT " << *J_first_ <<"\n\ncoeff " << coeff << "\n\nproj_jac " << proj_jacobian << "\n\nK " << K << std::endl;
  // }
  // assert(!J_first->isInf());

  return J_first_;
}

Eigen::Matrix<float,1,3> BundleAdj::getJfirst(Eigen::Matrix<float, 2,3>* Jfirst_, Eigen::Matrix<float,1,2> img_jacobian ){


  assert(Jfirst_->allFinite());
  assert(img_jacobian.allFinite());

  Eigen::Matrix<float,1,3> J_first;

  if(img_jacobian.isZero()){
    J_first.setZero();
    return J_first;
  }

  J_first = (img_jacobian)*(*Jfirst_);
  return J_first;

}




Eigen::Matrix<float,1,6> BundleAdj::getJm( Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,6> JSecond_jm){

  Eigen::Matrix<float,1,6> J_m;

  J_m=(J_first)*(JSecond_jm);
  assert(J_m.allFinite());

  return J_m;
}

Eigen::Matrix3f BundleAdj::getRelativeRotationMatrix(ActivePoint* active_pt, CameraForMapping* cam_m){
  CameraForMapping* cam_r = active_pt->cam_;
  Eigen::Matrix3f relative_rot_mat;
  relative_rot_mat = (cam_m->frame_world_wrt_camera_0_->linear())*(cam_r->frame_camera_wrt_world_0_->linear());
  return relative_rot_mat;
}

Eigen::Matrix<float,3,6> BundleAdj::getJSecondJr(ActivePoint* active_pt, Eigen::Matrix3f relative_rot_mat ){
  Eigen::Vector3f point_r_0 = *(active_pt->p_incamframe_0_);

  Eigen::Matrix<float, 3,6> JSecond_jr;
  Eigen::Matrix<float, 3,6> state_jacobian;

  state_jacobian << -1,  0,  0,  0             , -point_r_0.z()  ,  point_r_0.y(),
                     0, -1,  0,  point_r_0.z() ,  0              , -point_r_0.x(),
                     0,  0, -1, -point_r_0.y() ,  point_r_0.x()  ,  0            ;

  JSecond_jr = (relative_rot_mat)*(state_jacobian);

  return JSecond_jr;
}

Eigen::Matrix<float,3,6> BundleAdj::getJSecondJm( Eigen::Vector3f& point_m_0 ){
  Eigen::Matrix<float, 3,6> state_jacobian ;

  state_jacobian << 1, 0, 0,  0             ,  point_m_0.z()  , -point_m_0.y(),
                    0, 1, 0, -point_m_0.z() ,  0              ,  point_m_0.x(),
                    0, 0, 1,  point_m_0.y() , -point_m_0.x()  ,  0         ;

  return state_jacobian;
}

Eigen::Matrix<float,3,1> BundleAdj::getJSecondJd( ActivePoint* active_pt, Eigen::Matrix3f relative_rot_mat ){

  CameraForMapping* cam_r = active_pt->cam_;
  Eigen::Matrix3f Kinv = *(cam_r->Kinv_);

  // INVDEPTH_VS_DEPTH

  Eigen::Matrix<float, 3,1> invdepth_jacobian;
  invdepth_jacobian << -active_pt->uv_.x()/pow(active_pt->invdepth_,2),
                       -active_pt->uv_.y()/pow(active_pt->invdepth_,2),
                       -1/pow(active_pt->invdepth_,2);
  Eigen::Matrix<float, 3,1> JSecond_jd;
  JSecond_jd =(relative_rot_mat)*Kinv*invdepth_jacobian;

  // Eigen::Matrix<float, 3,1> depth_jacobian;
  // depth_jacobian << active_pt->uv_.x(),
  //                   active_pt->uv_.y(),
  //                   1;
  // Eigen::Matrix<float, 3,1> JSecond_jd;
  // JSecond_jd =(relative_rot_mat)*Kinv*depth_jacobian;



  return JSecond_jd;

}

Eigen::Matrix<float,1,6> BundleAdj::getJr(Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,6> JSecond_jr){

  Eigen::Matrix<float,1,6> J_r;

  J_r=(J_first)*(JSecond_jr);
  assert(J_r.allFinite());

  return J_r;
}

float BundleAdj::getJd( Eigen::Matrix<float,1,3> J_first, Eigen::Matrix<float,3,1> JSecond_jd ){

  float J_d = (J_first)*(JSecond_jd);

  // if (J_d>10)
  //   std::cout << "Jd " << J_d << "\n\nJSecond_jd " << JSecond_jd << "\n\nJ_first: " << J_first << "\n" << std::endl;

  assert((J_first.allFinite()));
  assert(JSecond_jd.allFinite());
  assert(std::isfinite(J_d));

  return J_d;
}

void HessianAndB_base::mirrorTriangH(bool pose_pose_not_diagonal){

  // mirror pose point
  *H_point_pose=H_pose_point->transpose();

  // mirror pose pose
  if(pose_pose_not_diagonal){
    for (int m=0 ; m<pose_block_size ; m+=6){
      for (int r=0 ; r<m ; r+=6){
        H_pose_pose->block(r,m,6,6)=H_pose_pose->block(m,r,6,6).transpose();
      }
    }
  }


}

bool HessianAndB::updateHessianAndB(JacobiansAndError* jacobians_and_error ){

  assert(jacobians_and_error!=nullptr);

  int d = jacobians_and_error->active_pt->state_point_block_idx_;
  int r = jacobians_and_error->active_pt->cam_->state_pose_block_idx_;

  int m = jacobians_and_error->cam_m->state_pose_block_idx_;

  Eigen::Matrix<float,1,6> J_r;
  Eigen::Matrix<float,6,1> J_r_transp;
  if(jacobians_and_error->J_r!=nullptr){
    J_r = *(jacobians_and_error->J_r);
    J_r_transp = *(jacobians_and_error->J_r_transp);
  }

  Eigen::Matrix<float,1,6> J_m = *(jacobians_and_error->J_m);
  Eigen::Matrix<float,6,1> J_m_transp = *(jacobians_and_error->J_m_transp);

  float J_d = jacobians_and_error->J_d;

  float error = jacobians_and_error->error;
  float weight_total = jacobians_and_error->weight_total;
  float omega = jacobians_and_error->omega;

  assert(d>-1 && d<point_block_size);
  assert(m>=-1 && m<pose_block_size);
  assert(r>=-1 && r<pose_block_size);

  // need for dealing with fixed first keyframe
  bool r_flag = (r!=-1);
  bool m_flag = (m!=-1);


  // ********** update H **********
  // pose pose block
  // O-
  // --
  if(m_flag){
    H_pose_pose->block<6,6>(m,m)+=J_m_transp*(omega*weight_total)*J_m;
  }
  if(r_flag){
    H_pose_pose->block<6,6>(r,r)+=J_r_transp*(omega*weight_total)*J_r;
  }
  if(m_flag && r_flag){
    H_pose_pose->block<6,6>(m,r)+=J_m_transp*(omega*weight_total)*J_r;
    // H_pose_pose->block(r,m,6,6)+=J_r_transp*weight_total*J_m;
  }


  // pose point block
  // -O
  // --
  if(m_flag){
    H_pose_point->block<6,1>(m,d)+=J_m_transp*(omega*weight_total*J_d);
  }
  if(r_flag){
    H_pose_point->block<6,1>(r,d)+=J_r_transp*(omega*weight_total*J_d);
  }


  // point point block
  // --
  // -O
  H_point_point->diagonal()[d]+=J_d*(omega*weight_total)*J_d;


  // ********** update b **********
  // pose block
  if(m_flag){
    b_pose->segment<6>(m)+=J_m_transp*(omega*weight_total*error);
  }
  if(r_flag){
    b_pose->segment<6>(r)+=J_r_transp*(omega*weight_total*error);
  }

  // point block
  (*b_point)(d)+=J_d*(omega*weight_total*error);

  return true;
}

void HessianAndB_base::LMDampening(Params* parameters){
  // damp pose

  // int size_pose = H_pose_pose->rows();
  // float sv_sum_pose = 0;
  // for(int i=0; i<size_pose; i++){
  //   sv_sum_pose += (*H_pose_pose)(i,i);
  // }
  //
  // float sv_sum_pt = 0;
  // for(int i=0; i<point_block_size; i++){
  //   sv_sum_pt += H_point_point->diagonal()[i];
  // }
  // float sv_average = (sv_sum_pose+sv_sum_pt)/(H_pose_pose->rows()+point_block_size);

  float max_eig = H_pose_pose->operatorNorm();

  // damp point
  // float damp_coeff = parameters->damp_point_invdepth;
  float damp_coeff = max_eig*1e-3;
  // float damp_coeff = 1000;
  for(int i=0; i<point_block_size; i++){
    H_point_point->diagonal()[i]+=damp_coeff;
    // H_point_point->diagonal()[i]+=damp_coeff;
  }
  // for(int i=0; i<pose_block_size; i++){
  //   H_pose_pose->diagonal()[i]+=damp_coeff;
  //   // H_point_point->diagonal()[i]+=damp_coeff;
  // }
}

void HessianAndB_Marg::updateHessianAndB_marg(JacobiansAndError* jacobians_and_error ){

  assert(jacobians_and_error!=nullptr);
  assert(jacobians_and_error->J_r==nullptr);
  assert(jacobians_and_error->J_m!=nullptr);
  // assert(jacobians_and_error->J_d!=0);

  Eigen::Matrix<float,1,6> J_m = *(jacobians_and_error->J_m);
  float J_d = jacobians_and_error->J_d;
  Eigen::Matrix<float,6,1> J_m_transp = *(jacobians_and_error->J_m_transp);
  float error = jacobians_and_error->error;
  float weight_total = jacobians_and_error->weight_total;

  int m = jacobians_and_error->cam_m->state_pose_block_marg_idx_;
  int d = jacobians_and_error->active_pt->state_point_block_marg_idx_;

  // need for dealing with fixed first keyframe
  assert(d>-1 && d<point_block_size);
  assert(m>-1 && m<pose_block_size);

  // ********** update H **********
  // pose pose block
  // O-
  // --
  H_pose_pose->block(m,m,6,6)+=J_m_transp*weight_total*J_m;

  // pose point block
  // -O
  // --
  H_pose_point->block(m,d,6,1)+=J_m_transp*(weight_total*J_d);

  // point point block
  // --
  // -O
  H_point_point->diagonal()[d]+=J_d*weight_total*J_d;

  // ********** update b **********
  // pose block
  b_pose->segment(m,6)+=J_m_transp*(weight_total*error);

  // point block
  (*b_point)(d)+=J_d*(weight_total*error);

}


float BundleAdj::getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, int image_id){
  float z, z_hat;
  float error;
  if(image_id==INTENSITY_ID){
    z = active_pt->intensity_;
    z_hat = cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->c->evalPixelBilinear(pixel_m);
    error = parameters_->intensity_coeff*(z_hat-z);
  }
  else if(image_id==GRADIENT_ID){
    z = active_pt->grad_magnitude_;
    z_hat = cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->magn_cd->evalPixelBilinear(pixel_m);
    error = parameters_->gradient_coeff*(z_hat-z);
  }
  else if(image_id==PHASE_ID){
    z = active_pt->grad_phase_;
    z_hat = cam_m->wavelet_dec_->getWavLevel(active_pt->level_)->phase_cd->evalPixelBilinear(pixel_m);
    error = parameters_->phase_coeff*(radiansSub(z_hat,z));
  }

  // error
  // float error = (z_hat-z)/normalizer;
  return error;
}


float BundleAdj::getWeightTotal(float error){
  float weight_total;

  // huber robustifier
  if(opt_norm_==HUBER){
    float huber_threshold=dtam_->parameters_->huber_threshold;
    float u = abs(error);

    if (u<=huber_threshold){
      weight_total=1/huber_threshold;
    }
    else{
      // float rho_der = huberNormDerivative(error,dtam_->parameters_->huber_threshold);
      float rho_der = huberNormDerivative(u,dtam_->parameters_->huber_threshold);
      float gamma=(1/u)*rho_der;
      weight_total=gamma;
    }

  }
  // least square without robustifier
  else if (opt_norm_==QUADRATIC){
    weight_total=1;
  }
  else{
    throw std::invalid_argument( "optimization norm has wrong value" );
  }
  assert(!std::isnan(weight_total));
  assert(!std::isinf(weight_total));


  // // weight
  // float variance = dtam_->parameters_->variance;
  // int ni = dtam_->parameters_->robustifier_dofs;
  // float weight = (ni+1.0)/(ni+(pow(error,2)/variance));
  //
  // float  weight_total = weight*gamma;

  // return weight_total;
  return weight_total;
  // return 1;
}


std::vector<int> BundleAdj::collectImageIds(){
  std::vector<int> image_id_vec;
  if(image_id_==INTENSITY_ID){
    image_id_vec.push_back(INTENSITY_ID);
  }
  else if(image_id_==GRADIENT_ID){
    image_id_vec.push_back(INTENSITY_ID);
    image_id_vec.push_back(GRADIENT_ID);
  }
  else if(image_id_==PHASE_ID){
    image_id_vec.push_back(INTENSITY_ID);
    image_id_vec.push_back(GRADIENT_ID);
    image_id_vec.push_back(PHASE_ID);
  }
  else
  {}

  return image_id_vec;
}


JacobiansAndError* BundleAdj::getJacobiansAndError(ActivePoint* active_pt, CameraForMapping* cam_m, bool no_r){


  Eigen::Vector3f point_m_0;
  pxl pixel_m;
  Eigen::Matrix<float,2,3>* J_1_;
  J_1_ = getJfirst_( active_pt, cam_m, point_m_0, pixel_m);

  if (J_1_==nullptr)
    return nullptr;

  Eigen::Matrix3f relative_rot_mat = getRelativeRotationMatrix( active_pt,cam_m);
  Eigen::Matrix<float,3,6> JSecond_jr = getJSecondJr( active_pt, relative_rot_mat );
  Eigen::Matrix<float,3,6> JSecond_jm = getJSecondJm( point_m_0 );
  Eigen::Matrix<float,3,1> JSecond_jd = getJSecondJd( active_pt, relative_rot_mat );

  Eigen::Matrix<float,1,6>* J_r = nullptr;
  if (!no_r){
    J_r = new Eigen::Matrix<float,1,6>;
    J_r->setZero();
  }

  Eigen::Matrix<float,1,6>* J_m = new Eigen::Matrix<float,1,6>;
  J_m->setZero();
  float J_d = 0;
  float error = 0;

  // collect list
  std::vector<int> image_id_vec = collectImageIds();
  for(int image_id : image_id_vec ){

    Eigen::Matrix<float,1,2> img_jacobian = getImageJacobian( active_pt, cam_m, pixel_m, image_id);

    Eigen::Matrix<float,1,3> J_1= getJfirst( J_1_, img_jacobian);

    if (!no_r){
      Eigen::Matrix<float,1,6> J_r_ = getJr( J_1, JSecond_jr);
      *J_r+=J_r_;
    }

    Eigen::Matrix<float,1,6> J_m_ = getJm( J_1, JSecond_jm);
    *J_m+=J_m_;

    float J_d_ = getJd( J_1, JSecond_jd);
    J_d+=J_d_;


    float error_ = getError( active_pt, cam_m ,pixel_m, image_id );
    error += error_;
  }

  if(J_m->isZero() || J_d==0 || (!no_r && J_r->isZero()) ){
    return nullptr;
  }

  float var = active_pt->invdepth_var_;
  // float omega = 100000.0;
  float omega = 1;
  // float omega = 1.0/var;
  // float omega = J_d*J_d*(1.0/var);

  float chi = getChi(error);
  // float chi = getChi(error,omega);
  float  weight_total = getWeightTotal(error);


  // float omega_pose_pose = parameters_->omega_pose_pose;
  // float omega_point_point = parameters_->omega_point_point;
  // float omega_pose_point = 1/((sqrt(1/omega_pose_pose_))*(sqrt(1/omega_point_point_)));

  // float omega = 0.01;
  // float omega = 0.001;  float sd_p = parameters_->sd_pose_position;

  // float omega_point_point = 1.0/(parameters_->sd_invdepth_*parameters_->sd_invdepth_);
  // // Eigen::Matrix6f sd_pose_pose;
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>sd_pose_pose;
  //
  // float p = parameters_->sd_pose_position;
  // float a = parameters_->sd_pose_angles_;
  // sd_pose_pose << p,0,0,0,0,0,
  //                 0,p,0,0,0,0,
  //                 0,0,p,0,0,0,
  //                 0,0,0,a,0,0,
  //                 0,0,0,0,a,0
  //                 0,0,0,0,0,a;
  //
  // Eigen::Matrix6f omega_pose_pose;
  // omega_pose_pose << (sd_pose_pose*sd_pose_pose).inverse()


  JacobiansAndError* jacobians = new JacobiansAndError(J_r,J_m,J_d,cam_m,active_pt, error, chi, weight_total, omega );

  return jacobians;

}

Eigen::DiagonalMatrix<float,Eigen::Dynamic>* HessianAndB_base::invertHPointPoint(float thresh){

  // if (H_point_point_inv!=nullptr){
  //   return H_point_point_inv;
  // }

  H_point_point_inv = new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size);


  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size);

  int count = 0;
  for(int i=0; i<point_block_size; i++){
    float val = H_point_point->diagonal()[i];
    assert(val>=0);
    if(val>thresh)
    // if(val!=0)
      H_point_point_inv->diagonal()[i]=(1.0/val);
    else{
      H_point_point_inv->diagonal()[i]=0;
      count++;
    }
    // std::cout << 1./val << std::endl;
  }
  // std::cout << "discarded: " << count << " out of " << point_block_size << std::endl;

  // *H_point_point_inv=H_point_point->inverse();

  return H_point_point_inv;
}

Eigen::DiagonalMatrix<float,Eigen::Dynamic>* HessianAndB_base::invertHPointPointDLS(float mu){
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size);
  for(int i=0; i<point_block_size; i++){
    float val = H_point_point->diagonal()[i];
    float val2 = val*val;
    if(val!=0)
      H_point_point_inv->diagonal()[i]=val/(val*val+mu);
  }
  return H_point_point_inv;
}


deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrements(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;
  dx_poses->setZero();
  dx_points->setZero();

  float thresh = getPinvThreshold(H_point_point->diagonal());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint(thresh);

  Eigen::MatrixXf Schur=(*H_pose_pose)-((*H_pose_point)*(*H_point_point_inv)*(*H_point_pose));
  // Eigen::MatrixXf Schur_inv=Schur.inverse();


  Eigen::MatrixXf Schur_inv = Schur.completeOrthogonalDecomposition().pseudoInverse();

  *dx_poses =  (Schur_inv) * ( -(*b_pose) + (*H_pose_point)*(*H_point_point_inv)*(*b_point) );
  *dx_points = (*H_point_point_inv)* ( -(*b_point) -( (*H_point_pose)*(*dx_poses)) );
  // *dx_points = (*H_point_point_inv)* ( -(*b_point) );

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}

deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrementsProva(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;

  // std::cout << H_point_point->diagonal() << std::endl;
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint();


  // float thresh_pt = 400*((pose_block_size/6));
  // float thresh_pt = 150*((pose_block_size/6));
  // float thresh_pt = H_point_point->diagonal().maxCoeff()*1e-07;
  // float thresh_pt = getPinvThreshold(H_point_point->diagonal());
  // float thresh_pt = 100;
  // float thresh_pose = H_pose_pose->operatorNorm()*7.15256e-07*(pose_block_size+point_block_size);
  // float thresh_pose = H_pose_pose->operatorNorm()*1e-07;
  // float thresh_pose = H_pose_pose->operatorNorm()*1e-06;
  // float thresh_pose = getPinvThreshold(H_pose_pose->diagonal());
  // thresh *= (pose_block_size/6)-1;
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint(1);
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv_pt = invertHPointPoint(thresh_pt);
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv_pose = invertHPointPoint(thresh_pose);
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv_pose = invertHPointPoint(thresh_pose);
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv_pt = invertHPointPointDLS(thresh_pt*thresh_pt);
  // Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv_pose = invertHPointPointDLS(thresh_pt*thresh_pose);


  Eigen::MatrixXf Schur=(*H_pose_pose)-((*H_pose_point)*(*H_point_point_inv)*(*H_point_pose));
  Eigen::MatrixXf Schur_inv = Schur.completeOrthogonalDecomposition().pseudoInverse();


  *dx_poses =  (Schur_inv) * ( -(*b_pose) + (*H_pose_point)*(*H_point_point_inv)*(*b_point) );
  // *dx_poses =  (H_pose_pose->completeOrthogonalDecomposition().pseudoInverse()) * ( -(*b_pose)  );

  // *dx_points = (*H_point_point_inv)* ( -(*b_point) -( (*H_point_pose)*(*dx_poses)) );
  // *dx_points = (*H_point_point_inv)* (-(*b_point)) ;
  dx_points->setZero();

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}


deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrementsOnlyPoints(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;

  float thresh = getPinvThreshold(H_point_point->diagonal());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint(thresh);

  dx_poses->setZero();
  *dx_points = (*H_point_point_inv)* ( -(*b_point) );

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}

deltaUpdateIncrements* HessianAndB::getDeltaUpdateIncrementsOnlyPoses(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;
  Eigen::VectorXf* dx_points = new Eigen::VectorXf(point_block_size) ;

  Eigen::MatrixXf H_pose_pose_inv = H_pose_pose->completeOrthogonalDecomposition().pseudoInverse();


  *dx_poses = (H_pose_pose_inv)* ( -(*b_pose) );
  dx_points->setZero();

  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);

  return delta;
}

Eigen::VectorXf* HessianAndB_Marg::getDeltaUpdateIncrementsMarg(){
  Eigen::VectorXf* dx_poses = new Eigen::VectorXf(pose_block_size) ;

  float thresh = getPinvThreshold(H_point_point->diagonal());
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point_inv = invertHPointPoint(thresh);

  Eigen::MatrixXf Schur=(*H_pose_pose)-((*H_pose_point)*(*H_point_point_inv)*(*H_point_pose));
  // Eigen::MatrixXf Schur_inv=Schur.inverse();


  Eigen::MatrixXf Schur_inv = Schur.completeOrthogonalDecomposition().pseudoInverse();
  *dx_poses =  (Schur_inv) * ( -(*b_pose) + (*H_pose_point)*(*H_point_point_inv)*(*b_point) );

  // Eigen::MatrixXf* Schur_inv = pinv(Schur);
  // *dx_poses =  (*Schur_inv) * ( -(*b_pose) + (*H_pose_point)*(*H_point_point_inv)*(*b_point) );

  return dx_poses;
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

  // Eigen::MatrixXf Hpinv = H.completeOrthogonalDecomposition().pseudoInverse();
  auto a = H.completeOrthogonalDecomposition();
  std::cout << "nonzeroPivots: " << a.nonzeroPivots() << ", rank: " << a.rank() << ", size: " << pose_block_size+point_block_size;

  // d=-(H.inverse())*b ;
  // d=-Hpinv*b ;
  d=a.solve(-b);

  *dx_poses  =  d.segment(0,pose_block_size);
  *dx_points =  d.segment(pose_block_size,point_block_size);


  deltaUpdateIncrements* delta = new deltaUpdateIncrements(dx_poses,dx_points);
  return delta;
}


bool HessianAndB_base::visualizeH( const std::string& name){
  Image<colorRGB>* img_H = new Image<colorRGB>(name);
  int size = pose_block_size+point_block_size;
  if (size == 0)
    return false;
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
    if ((H_point_point->diagonal())[i]!=0){
      img_H->setPixel(i+pose_block_size,i+pose_block_size, blue);
    }
    else{
      img_H->setPixel(i+pose_block_size,i+pose_block_size, white);

    }

  }

  img_H->show(1);
  cv::waitKey(0);
  delete img_H;
}


bool HessianAndB_Marg::visualizeHMarg( const std::string& name){
  Image<colorRGB>* img_H = new Image<colorRGB>(name);
  int size = pose_block_size+point_block_size;
  if (size == 0)
    return false;
  // img_H->initImage(pose_block_size,pose_block_size);
  img_H->initImage(size,size);
  img_H->setAllPixels( white);

  // pose pose block
  for(int i=0; i<pose_block_size; i++){
    for(int j=0; j<pose_block_size; j++){
      colorRGB color = red;
      if (i<hessian_b_marg_old->pose_block_size && j<hessian_b_marg_old->pose_block_size)
        color = magenta;
      if ((*H_pose_pose)(i,j)!=0){
        img_H->setPixel(i,j, color);
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
        colorRGB color = green;
        if (i<hessian_b_marg_old->pose_block_size && j<hessian_b_marg_old->point_block_size)
          color = yellow;
        img_H->setPixel(i,pose_block_size+j, color);
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
        colorRGB color = green;
        if (j<hessian_b_marg_old->pose_block_size && i<hessian_b_marg_old->point_block_size)
          color = yellow;
        img_H->setPixel(i+pose_block_size,j, color);
      }
      else{
        img_H->setPixel(i+pose_block_size,j, white);

      }
    }
  }

  // point point block
  for(int i=0; i<point_block_size; i++){
    if ((H_point_point->diagonal())[i]!=0){
      colorRGB color = blue;
      if ( i<hessian_b_marg_old->point_block_size)
        color = cyan;
      img_H->setPixel(i+pose_block_size,i+pose_block_size, color);
    }
    else{
      img_H->setPixel(i+pose_block_size,i+pose_block_size, white);

    }

  }

  img_H->show(0.5);
  cv::waitKey(0);
  delete img_H;
}



void BundleAdj::updateTangentSpace(bool with_marg){
  // fix tangent space for cameras
  for(int i=0; i<keyframe_vector_ba_->size() ; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->state_pose_block_idx_!=-1 ){
      assert(!keyframe->first_keyframe_);
      // if active keyframe has link with marginalization prior, do not update tangent space

      // if(keyframe->state_pose_block_marg_idx_!=-1 && with_marg){
      //   continue;
      // }
      keyframe->assignPose0( *(keyframe->frame_camera_wrt_world_) );
      keyframe->delta_update_x_->setZero();
    }
  }

  // fix tangent space for points
  for(int i=0; i<keyframe_vector_ba_->size() ; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->state_pose_block_idx_!=-1 || keyframe->first_keyframe_){

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
  if(delta->dx_poses!=nullptr){
    for(int i=0; i<keyframe_vector_ba_->size() ; i++){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
      if (keyframe->state_pose_block_idx_!=-1 ){
        assert(!keyframe->first_keyframe_);
        assert(!keyframe->to_be_marginalized_ba_);

        // update delta update of keyframe
        // regular sum in tangent space
        // if(!keyframe->first_keyframe_){
          (*keyframe->delta_update_x_)+=delta->dx_poses->segment(keyframe->state_pose_block_idx_,6);

          Eigen::Isometry3f frame_camera_wrt_world =(*(keyframe->frame_camera_wrt_world_0_))*v2t_inv(*(keyframe->delta_update_x_));
          keyframe->assignPose( frame_camera_wrt_world );
        // }

      }
    }
  }

  // UPDATE POINTS
  // if(delta->dx_points!=nullptr){
    for(int i=0; i<keyframe_vector_ba_->size() ; i++){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
      if (keyframe->state_pose_block_idx_!=-1 || keyframe->first_keyframe_){
        assert(!keyframe->to_be_marginalized_ba_);


        // iterate through all active points
        for(int j=0; j<keyframe->active_points_->size() ; j++){
          ActivePoint* active_pt = keyframe->active_points_->at(j);

          if(active_pt->state_point_block_idx_==-1)
            continue;

          // update delta update of active point
          // regular sum since invdepth is in R
          if(delta->dx_points!=nullptr){


            active_pt->delta_update_x_+=(*(delta->dx_points))(active_pt->state_point_block_idx_);

            // INVDEPTH_VS_DEPTH
            float new_invdepth = active_pt->invdepth_0_+active_pt->delta_update_x_;
            // float new_invdepth = 1.0/((1.0/active_pt->invdepth_0_)+active_pt->delta_update_x_);

            float new_depth = 1.0/new_invdepth;
            float min_depth = dtam_->camera_vector_->at(0)->cam_parameters_->min_depth;
            float max_depth = dtam_->camera_vector_->at(0)->cam_parameters_->max_depth;
            if(new_depth>min_depth && new_depth<max_depth){
              active_pt->invdepth_=new_invdepth;
              // since change both poses and invdepths
              keyframe->pointAtDepthInCamFrame(active_pt->uv_, 1.0/active_pt->invdepth_, *(active_pt->p_incamframe_));
              *(active_pt->p_)=(*(keyframe->frame_camera_wrt_world_0_))*v2t_inv(*(keyframe->delta_update_x_))*(*(active_pt->p_incamframe_));
            }

          }


        }
      }
    }

}


void BundleAdj::updateInvdepthVars(HessianAndB* hessian_and_b){

  // UPDATE POINTS
  // if(delta->dx_points!=nullptr){
  for(int i=0; i<keyframe_vector_ba_->size() ; i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->state_pose_block_idx_!=-1 || keyframe->first_keyframe_){

      // iterate through all active points
      for(int j=0; j<keyframe->active_points_->size() ; j++){
        ActivePoint* active_pt = keyframe->active_points_->at(j);
        if(active_pt->state_point_block_idx_==-1)
          continue;

        if(hessian_and_b->H_point_point_inv!=nullptr){
          float omega = hessian_and_b->H_point_point->diagonal()[active_pt->state_point_block_idx_];
          // float variance=(1/omega);
          float variance=(1/omega)/keyframe_vector_ba_->size();
          // float variance = hessian_and_b->H_point_point_inv->diagonal()[active_pt->state_point_block_idx_];
          // if(variance!=0){  // if val in H_point_point_inv is 0, there is no variance propagation
            active_pt->invdepth_var_=variance+1;
            // active_pt->invdepth_var_=omega;
            // active_pt->invdepth_var_=std::max((variance)+0.01,100.0);
            // if certainty very close to 0 (variance very high), variance saturate close to
            // if certainty is very large, variance saturate close to 0.001
          // }
        }
      }
    }
  }

}

void HessianAndB_Marg::curr2old(){
  delete hessian_b_marg_old;
  hessian_b_marg_old = new HessianAndB_base(this);
  // deleteAllPtrs();
}


bool BundleAdj::updateOldMargHessianAndB(){

  // current become old H and b
  hessian_b_marg->curr2old();

  // if i have no H and b, i have no old structure
  // and so i have no old prior to update (just at first frame)
  if(hessian_b_marg->hessian_b_marg_old->isNull())
    return false;

  HessianAndB_base* hessian_b_old = hessian_b_marg->hessian_b_marg_old;

  std::vector<Eigen::VectorXf> pose_point_cols;
  std::vector<float> point_point_vals;
  std::vector<Matrix6f> pose_pose_blocks;
  std::vector<Vector6f> b_pose_segments;
  std::vector<float> b_point_vals;


  // ************* set old cam marg idxs, get row to marg *************

  int max_poses_old_size = hessian_b_old->pose_block_size;
  int max_point_old_size = hessian_b_old->point_block_size;
  int poses_old_size = 0;
  int row_pose_to_be_marginalized = -1;


  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    int block_idx = keyframe->state_pose_block_marg_idx_;
    // if keyframe is not the one that is going to be marginalized, and keyframe has link with marg prior
    if (!keyframe->to_be_marginalized_ba_ && block_idx!=-1 ){
      // hessian_b_old->visualizeH("ao=?");
      // cv::waitKey(0);
      Matrix6f pose_pose_block = hessian_b_old->H_pose_pose->block<6,6>(block_idx,block_idx);
      pose_pose_blocks.push_back(pose_pose_block);
      Vector6f b_pose_segment = hessian_b_old->b_pose->segment<6>(block_idx);
      b_pose_segments.push_back(b_pose_segment);
      keyframe->state_pose_block_marg_idx_=poses_old_size;
      poses_old_size+=6;
    }
    // save initial row of pose block to marginalize
    else if(keyframe->to_be_marginalized_ba_){
      row_pose_to_be_marginalized = keyframe->state_pose_block_marg_idx_;
      keyframe->state_pose_block_marg_idx_=-1;
    }

  }

  // if row_pose_to_be_marginalized==-1, poses are not marginalized
  // and so there is no need to update old prior
  if(row_pose_to_be_marginalized==-1)
    return false;

  // ************* structure of old variables *************

  int points_old_size = 0;
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
      b_point_vals.push_back((*hessian_b_old->b_point)[col]);
      points_old_size++; // increment n old points size
    }
    // otherwise that marginalized point is automatically removed inside the factor graph
  }

  // ************* update old hessian and b *************

  // create new hessiand and b
  // hessian_b_old->initHessianAndB(pose_pose_blocks.size(),pose_point_cols.size());
  hessian_b_old->initHessianAndB(poses_old_size, points_old_size);

  // update old pose pose block
  for (int i=0; i<pose_pose_blocks.size(); i++){
    Matrix6f pose_pose_block = pose_pose_blocks.at(i);
    hessian_b_old->H_pose_pose->block<6,6>(i*6,i*6)=pose_pose_block;
  }

  // update old pose point block
  for (int i=0; i<pose_point_cols.size(); i++){
    Eigen::VectorXf col = pose_point_cols.at(i);
    hessian_b_old->H_pose_point->col(i)=col;
  }

  // update old point point block
  for (int i=0; i<point_point_vals.size(); i++){
    float point_point_val = point_point_vals.at(i);
    hessian_b_old->H_point_point->diagonal()[i]=point_point_val;
  }

  // update old b pose segment
  for (int i=0; i<b_pose_segments.size(); i++){
    Vector6f b_pose_segment = b_pose_segments.at(i);
    hessian_b_old->b_pose->segment<6>(i*6)=b_pose_segment;
  }

  // update old b point segment
  for (int i=0; i<b_point_vals.size(); i++){
    float b_point_val = b_point_vals.at(i);
    (*hessian_b_old->b_point)[i]=b_point_val;
  }



  hessian_b_old->mirrorTriangH(true);

  return true;
}

void BundleAdj::getJacobiansForNewUpdate(int& new_points, int& poses_new_size, std::vector<JacobiansAndError*>* jacobians_and_error_vec ){

  assert(new_points == 0);
  assert(poses_new_size == 0);
  assert(jacobians_and_error_vec->empty());

  int poses_old_size = hessian_b_marg->hessian_b_marg_old->pose_block_size;
  int points_old_size = hessian_b_marg->hessian_b_marg_old->point_block_size;


  // ********** collect Jacobians **********

  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    // iterate through points to be marginalized
    for( int j=0; j<keyframe->marginalized_points_->size(); j++){
      ActivePoint* pt_to_be_marg = keyframe->marginalized_points_->at(j);
      bool point_taken = false;

      // iterate through keyframes on which the active point is projected
      // exclute last keyframe cause optimization has not been done (it's added just now)
      for(int k=0; k<keyframe_vector_ba_->size()-1 ; k++){

        // avoid self projection
        if(k==i)
          continue;

        CameraForMapping* keyframe_proj = dtam_->camera_vector_->at(keyframe_vector_ba_->at(k));

        if(keyframe_proj->to_be_marginalized_ba_){
          continue;
        }

        if(keyframe_proj->first_keyframe_){
          continue;
        }

        JacobiansAndError* jacobians = getJacobiansAndError(pt_to_be_marg, keyframe_proj, true);
        if (jacobians==nullptr){
          // n_suppressed_measurement++;
        }
        //occlusion detection
        else if (jacobians->chi > parameters_->chi_occlusion_threshold ){
          // active_pt->marginalize();
          // j--;
          // point_taken=false;
          // cam_taken=false;
          // for(int count =0; count<n_jac_added; count++){
          //   jacobians_and_error_vec->pop_back();
          // }
          // break;
        }
        else{
          // if pose is new inside the marginalization part
          if(keyframe_proj->state_pose_block_marg_idx_==-1){
            keyframe_proj->state_pose_block_marg_idx_=poses_old_size+poses_new_size;
            poses_new_size+=6;
          }

          point_taken = true;
          jacobians_and_error_vec->push_back(jacobians);
        }
      }

      if(point_taken){
          pt_to_be_marg->state_point_block_marg_idx_=points_old_size+new_points;
          new_points++;

      }
    }
  }

}

void BundleAdj::deleteMarginalizedPoints(){
  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // iterate through points to be marginalized
    for( int j=0; j<keyframe->marginalized_points_->size(); j++){
      ActivePoint* pt_to_be_marg = keyframe->marginalized_points_->at(j);
      pt_to_be_marg->remove();
    }
  }
}


void BundleAdj::removeMarginalizedKeyframe(){

  // for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
  //   CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
  //   // std::cout << keyframe->name_ << " ";
  // }
  // iterate along active keyframes
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    if (keyframe->to_be_marginalized_ba_){
      keyframe_vector_ba_->erase(keyframe_vector_ba_->begin() + i);
      std::cout << keyframe->name_ << " erased";

    }
  }

  // for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){
  //   CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
  //   // std::cout << keyframe->name_ << " ";
  // }
}


void BundleAdj::updateMargHessianAndB(int new_points, int poses_new_size, std::vector<JacobiansAndError*>* jacobians_and_error_vec){
  HessianAndB_base* hessian_b_old = hessian_b_marg->hessian_b_marg_old;

  // initialize structure
  int poses_old_size = hessian_b_old->pose_block_size;
  int points_old_size = hessian_b_old->point_block_size;

  int poses_size = poses_old_size + poses_new_size;
  int points_size = points_old_size + new_points;


  hessian_b_marg->initHessianAndB(poses_size, points_size);

  // assign old blocks
  if(poses_old_size!=0 && points_old_size!=0){
    hessian_b_marg->H_pose_pose->block(0,0,poses_old_size,poses_old_size)=(*hessian_b_old->H_pose_pose);
    hessian_b_marg->H_pose_point->block(0,0,poses_old_size,points_old_size)=(*hessian_b_old->H_pose_point);
    hessian_b_marg->H_point_point->diagonal().segment(0,points_old_size)=hessian_b_old->H_point_point->diagonal();
    hessian_b_marg->b_pose->segment(0,poses_old_size)=(*hessian_b_old->b_pose);
    hessian_b_marg->b_point->segment(0,points_old_size)=(*hessian_b_old->b_point);
  }

  // update new blocks
  for(int i=0; i<jacobians_and_error_vec->size(); i++){
    JacobiansAndError* jacobians_and_error = jacobians_and_error_vec->at(i);
    hessian_b_marg->updateHessianAndB_marg( jacobians_and_error );
    delete jacobians_and_error;
  }
  delete jacobians_and_error_vec;

  hessian_b_marg->mirrorTriangH(true);

}

bool BundleAdj::marginalization( ){

  updateOldMargHessianAndB();

  int new_points = 0;
  int poses_new_size = 0;
  std::vector<JacobiansAndError*>* jacobians_and_error_vec = new std::vector<JacobiansAndError*>;

  getJacobiansForNewUpdate(new_points,poses_new_size,jacobians_and_error_vec);

  updateMargHessianAndB(new_points, poses_new_size, jacobians_and_error_vec);

  deleteMarginalizedPoints();
  removeMarginalizedKeyframe();
  //
  // // debug
  if(debug_optimization_){
    // hessian_b_marg->visualizeHMarg("Hessian marginalization");
    // hessian_b_marg->hessian_b_marg_old->visualizeH("Hessian marginalization OLD");
  }

  return true;
}


void BundleAdj::initializeStateStructure( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec ){

  n_cams=hessian_b_marg->pose_block_size/6;
  int occlusions = 0;
  int active_points = 0;
  // iterate through keyframes with active points (except last)
  for(int i=0; i<keyframe_vector_ba_->size()-1; i++ ){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    // std::cout << keyframe->name_ << std::endl;
    // if keyframe is going to be marginalized, this step needs to be performed in marginalization procedure
    if(keyframe->to_be_marginalized_ba_){
      keyframe->state_pose_block_idx_=-1;
      continue;
    }
    bool cam_taken = false;

    active_points+=keyframe->active_points_->size();
    // iterate through active points
    for( int j=0; j<keyframe->active_points_->size(); j++){
      int invalid_projections = 0;

      ActivePoint* active_pt = keyframe->active_points_->at(j);

      bool point_taken = false;
      int n_jac_added = 0;
      // iterate through keyframes on which the active point is projected
      for(int k=keyframe_vector_ba_->size()-1; k>=0 ; k--){


        // avoid self projection
        if(k==i)
          continue;

        CameraForMapping* keyframe_proj = dtam_->camera_vector_->at(keyframe_vector_ba_->at(k));

        if(keyframe_proj->to_be_marginalized_ba_){
          continue;
        }

        // JacobiansAndError* jacobians = getJacobiansAndError(active_pt, keyframe_proj,keyframe->first_keyframe_ );
        JacobiansAndError* jacobians = getJacobiansAndError(active_pt, keyframe_proj );
        if (jacobians==nullptr){
          invalid_projections++;
        }
        //occlusion detection
        else if (jacobians->chi > parameters_->chi_occlusion_threshold ){
          occlusions++;
          invalid_projections++;
        }
        else{
          point_taken=true;

          jacobians_and_error_vec->push_back(jacobians);
          n_jac_added++;
        }
      }
      // if(invalid_projections>(keyframe_vector_ba_->size()+1)/2){
      if(invalid_projections>0){
        active_pt->state_point_block_idx_=-1;
        active_pt->remove();
        num_active_points_--;
        j--;
        point_taken=false;
        for(int count =0; count<n_jac_added; count++){
          jacobians_and_error_vec->pop_back();
        }

        // break;
      }

      // if(point_taken){
      if(point_taken){
        if(!cam_taken && !keyframe->first_keyframe_)
          cam_taken=true;
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
      if(keyframe->state_pose_block_marg_idx_!=-1){
        keyframe->state_pose_block_idx_=keyframe->state_pose_block_marg_idx_;
        // std::cout << "FROM MARG " << keyframe->state_pose_block_idx_ << " " << keyframe->name_  << std::endl;
      }
      else{
        keyframe->state_pose_block_idx_=n_cams*6;
        // std::cout << "NEW " << n_cams*6 << " " << keyframe->name_  << std::endl;

        n_cams++;
      }
    }
  }

  CameraForMapping* keyframe_last = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  if(keyframe_last->state_pose_block_marg_idx_!=-1){
    keyframe_last->state_pose_block_idx_=keyframe_last->state_pose_block_marg_idx_;
    // std::cout << "FROM MARG " << keyframe_last->state_pose_block_idx_ << " " << keyframe_last->name_  << std::endl;
  }
  else{
    keyframe_last->state_pose_block_idx_=n_cams*6;
    // std::cout << "NEW " << n_cams*6 << " " << keyframe_last->name_  << std::endl;

    n_cams++;
  }
  // keyframe_last->state_pose_block_idx_=n_cams*6;
  // n_cams++;
  // std::cout << "Num occlusions: " << occlusions << " out of " << active_points << " "<< num_active_points_ << std::endl;
}

Eigen::VectorXf* BundleAdj::getDeltaForMargFromOpt(deltaUpdateIncrements* delta){


  Eigen::VectorXf* dx_poses_marg = new Eigen::VectorXf(hessian_b_marg->pose_block_size) ;

  // iterate through all keyframes
  for (int i=0; i<keyframe_vector_ba_->size(); i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    int block_idx_marg = keyframe->state_pose_block_marg_idx_;
    // if keyframe has a link with prior
    if(block_idx_marg!=-1){
      Vector6f segment = delta->dx_poses->segment<6>(keyframe->state_pose_block_idx_);
      dx_poses_marg->segment<6>(block_idx_marg)=segment;
    }
  }

  return dx_poses_marg;
}

void HessianAndB_Marg::updateBFromDelta(Eigen::VectorXf* dx_poses_marg){
  (*b_pose)+=((*H_pose_pose)*(*dx_poses_marg));
  (*b_point)+=((*H_point_pose)*(*dx_poses_marg));
}


bool BundleAdj::updateDeltaUpdateIncrementsMarg(deltaUpdateIncrements* delta){

  if (hessian_b_marg->pose_block_size==0)
    return false;

  // get delta in marg from delta given by optimization
  Eigen::VectorXf* dx_poses_marg_from_opt = getDeltaForMargFromOpt(delta);

  // update b
  hessian_b_marg->updateBFromDelta(dx_poses_marg_from_opt);

  // get delta from marginalization (solve the marg linear system)
  Eigen::VectorXf* dx_poses_marg_new = hessian_b_marg->getDeltaUpdateIncrementsMarg();

  // update b again for next time
  hessian_b_marg->updateBFromDelta(dx_poses_marg_new);

  // update deltaUpdates
  // iterate through all keyframes
  for (int i=0; i<keyframe_vector_ba_->size(); i++){
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));
    int block_idx_marg = keyframe->state_pose_block_marg_idx_;
    // if keyframe has a link with prior
    if(block_idx_marg!=-1){
      // add marg delta inside total delta
      delta->dx_poses->segment<6>(keyframe->state_pose_block_idx_)+=dx_poses_marg_new->segment<6>(block_idx_marg);
    }
  }

  return true;

}

float BundleAdj::getChi(float error, float omega){
  assert(!std::isnan(error));
  assert(!std::isinf(error));
  float chi=0;
  if(opt_norm_==HUBER){
    // chi=huberNormWithOmega(error,parameters_->huber_threshold,omega);
    chi=huberNormWithOmega(error,parameters_->huber_threshold,1);
  }
  else if (opt_norm_==QUADRATIC){
    chi=error*omega*error;
  }
  else{
    throw std::invalid_argument( "optimization norm has wrong value" );
  }
  return chi;
}

float BundleAdj::optimizationStep(bool with_marg){

  float chi = 0;
  int n_suppressed_measurement = 0;
  std::vector<JacobiansAndError*>* jacobians_and_error_vec = new std::vector<JacobiansAndError*> ;

  int n_cams = 0;
  int n_points = 0;
  initializeStateStructure( n_cams, n_points, jacobians_and_error_vec );
  int n_jacs = jacobians_and_error_vec->size();

  // create Hessian and b vector
  HessianAndB* hessian_b = new HessianAndB(n_cams*6, n_points);
  // for each measurement update Hessian and b vector
  for(int i=0; i<n_jacs; i++){
    JacobiansAndError* jacobians_and_error = jacobians_and_error_vec->at(i);
    hessian_b->updateHessianAndB( jacobians_and_error );

    chi+=jacobians_and_error->chi;
    delete jacobians_and_error;
  }
  hessian_b->LMDampening(parameters_);
  hessian_b->mirrorTriangH();


  if(debug_optimization_){
    // hessian_b->visualizeH("Hessian");
  }
  delete jacobians_and_error_vec;


  deltaUpdateIncrements* delta;
  // get delta update
  if(test_single_==TEST_ALL){
    // delta = hessian_b->getDeltaUpdateIncrements();
    delta = hessian_b->getDeltaUpdateIncrementsProva();
    // delta = hessian_b->getDeltaUpdateIncrements_Slow();
  }
  else if(test_single_==TEST_ONLY_POSES){
    delta = hessian_b->getDeltaUpdateIncrementsOnlyPoses();
  }
  else if(test_single_==TEST_ONLY_POINTS){
    delta = hessian_b->getDeltaUpdateIncrementsOnlyPoints();
  }



  // update x in each cam and active point
  updateDeltaUpdates(delta);
  updateInvdepthVars(hessian_b);

  // update fixed points
  updateTangentSpace(with_marg);


  delete hessian_b;
  // delete delta;
  return chi/n_jacs;
}


void BundleAdj::optimize(){

  double t_start=getTime();

  // // marginalize
  marginalization();

  bool with_marg = false ;

  if(debug_optimization_){
    // if(keyframe_vector_ba_->size()>=parameters_->num_active_keyframes || !test_marginalization_){
      CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
      last_keyframe->clearProjectedActivePoints();
      projectActivePoints(0);
      last_keyframe->showProjActivePoints(1);
      PoseNormError* poses_norm_error_tot = dtam_->getTotalPosesNormError();
      float points_norm_error_tot = dtam_->getTotalPointsNormError();
      std::cout << "\npoints norm error tot: " << points_norm_error_tot <<  std::endl;
      poses_norm_error_tot->print();
      std::cout << std::endl;
      cv::waitKey(0);
    // }
  }

  // optimize
  // while(true){
  for(int i=0; i<60; i++){
    float chi = optimizationStep(with_marg );

    if(debug_optimization_){
      // if(keyframe_vector_ba_->size()>=parameters_->num_active_keyframes || !test_marginalization_){

        std::cout << "Iteration " << i << std::endl;
        CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());

        last_keyframe->clearProjectedActivePoints();
        bool take_fixed_point = 0;
        projectActivePoints(take_fixed_point);
        last_keyframe->showProjActivePoints(1);

        PoseNormError* poses_norm_error_tot = dtam_->getTotalPosesNormError();
        float points_norm_error_tot = dtam_->getTotalPointsNormError();
        std::cout << "\nchi: " << chi << std::endl;
        std::cout << "points norm error tot: " << points_norm_error_tot << std::endl;
        poses_norm_error_tot->print();
        std::cout << std::endl;

        chi_history->push_back(chi);
        pose_angle_error_history->push_back(poses_norm_error_tot->angle);
        pose_position_error_history->push_back(poses_norm_error_tot->position_norm);
        points_error_history->push_back(points_norm_error_tot);
        cv::waitKey(0);
      // }
    }
  }
  if(debug_optimization_ ){
    // if(keyframe_vector_ba_->size()>=parameters_->num_active_keyframes || !test_marginalization_){

      showLine(*chi_history, "chi_history");
      showLine(*pose_angle_error_history, "pose_angle_error_history");
      showLine(*pose_position_error_history, "pose_position_error_history");
      showLine(*points_error_history, "points_error_history");
      chi_history->clear();
      pose_angle_error_history->clear();
      pose_position_error_history->clear();
      points_error_history->clear();
    // }
  }

  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  last_keyframe->clearProjectedActivePoints();
  bool take_fixed_point = 0;
  projectActivePoints(take_fixed_point);
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

bool BundleAdj::projectActivePoints_prepMarg(bool take_fixed_point){
  assert(dtam_->camera_vector_->size() > 1);
  assert(keyframe_vector_ba_->size() > 1);
  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());
  CameraForMapping* prev_last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(keyframe_vector_ba_->size()-2));


  bool keyframe_marginalized = false;
  // iterate through all keyframe (except the last two )
  for (int i=0; i<keyframe_vector_ba_->size()-2; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));

    if (keyframe->to_be_marginalized_ba_){
      keyframe_marginalized=true;
      // iterate along all active points
      num_active_points_-=(keyframe->active_points_->size()-keyframe->num_marginalized_active_points_);

      for (int j=0; j<keyframe->active_points_->size(); j++){
        ActivePoint* active_pt_ = keyframe->active_points_->at(j);
        // active_pt_->marginalize();
        active_pt_->remove();
      }

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
          keyframe->num_marginalized_active_points_++;
          num_active_points_--;
        }

      }

    }

    delete cam_couple;
    delete cam_couple_prev;
  }

  return keyframe_marginalized;
}


void BundleAdj::projectActivePoints(bool take_fixed_point){

  CameraForMapping* last_keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->back());

  // iterate through all keyframe (except the last)
  for (int i=0; i<keyframe_vector_ba_->size()-1; i++){

    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_vector_ba_->at(i));


    CamCouple* cam_couple = new CamCouple(keyframe,last_keyframe,take_fixed_point);

    // iterate along all active points
    for (ActivePoint* active_pt : *keyframe->active_points_){

      if(active_pt->to_marginalize_){
        continue;
      }

      // project active point in new keyframe
      ActivePointProjected* active_point_proj = projectActivePoint(active_pt, cam_couple);
      // if active point is in frustum
      if (active_point_proj!=nullptr){
        // push active point projected
        active_point_proj->cam_->regions_projected_active_points_->pushProjActivePoint(active_point_proj);
      }


    }

    delete cam_couple;
  }

}
