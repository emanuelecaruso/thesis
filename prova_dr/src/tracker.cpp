#include "dtam.h"
#include "tracker.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

void Tracker::trackGroundtruth(){
  const std::vector<Camera*>* cam_vec_env = dtam_->environment_->camera_vector_;

  Camera* cam_gt = cam_vec_env->at(dtam_->frame_current_);
  CameraForMapping* cam = dtam_->camera_vector_->at(dtam_->frame_current_);

  *(cam->frame_world_wrt_camera_)=*(cam_gt->frame_world_wrt_camera_);
  *(cam->frame_camera_wrt_world_)=*(cam_gt->frame_camera_wrt_world_);
  *(cam->frame_world_wrt_camera_0_)=*(cam_gt->frame_world_wrt_camera_);
  *(cam->frame_camera_wrt_world_0_)=*(cam_gt->frame_camera_wrt_world_);

  sharedCoutDebug("   - Frame tracked (groundtruth)");
}

void Tracker::trackLS(bool track_candidates, int guess_type, bool debug_tracking){

  double t_start=getTime();

  CameraForMapping* last_cam = dtam_->getLastCamera();
  CameraForMapping* curr_cam = dtam_->getCurrentCamera();

  Eigen::Isometry3f initial_guess = computeInitialGuess( guess_type );
  // Eigen::Isometry3f initial_guess = computeInitialGuess( VELOCITY_CONSTANT );
  // Eigen::Isometry3f initial_guess = computeInitialGuessGT( );

  Eigen::Isometry3f final_guess = doLS(initial_guess, track_candidates, debug_tracking);

  // final guess = curr_T_last -> pose = w_T_last * last_T_curr
  Eigen::Isometry3f frame_camera_wrt_world_ = final_guess.inverse();

  curr_cam->assignPose(frame_camera_wrt_world_);
  curr_cam->assignPose0(frame_camera_wrt_world_);

  double t_end=getTime();
  int deltaTime=(t_end-t_start);
  sharedCoutDebug("   - Frame tracked, time: "+ std::to_string(deltaTime)+" ms");

  // Camera* cam_curr_gt = dtam_->environment_->camera_vector_->at(dtam_->frame_current_);
  // Eigen::Isometry3f gt = *(cam_curr_gt->frame_world_wrt_camera_);
  // std::cout << gt.translation() << "\n and \n " << initial_guess.translation() << std::endl;


}

void Tracker::collectCandidatesInCoarseRegions(){

  CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->frame_current_);

  // iterate along all coarser levels
  for(int i=1; i<=dtam_->parameters_->coarsest_level; i++){

    RegionsWithCandidates* coarse_regions = keyframe->regions_coarse_cands_vec_->at(i-1);

    // for each candidate of keyframe
    for(Candidate* cand : *(keyframe->candidates_)){
      // if level of candidate is less than current coarse level
      // and if candidate has one min
      if(cand->level_<i ){

        int level_diff = i-cand->level_;
        // from pixel of candidate find pixel at level i
        int coarse_pxl_x = cand->pixel_.x()/(pow(2,level_diff));
        int coarse_pxl_y = cand->pixel_.y()/(pow(2,level_diff));
        int idx = coarse_regions->xyToIdx(coarse_pxl_x,coarse_pxl_y);
        // push candidate inside region
        coarse_regions->region_vec_->at(idx)->cands_vec_->push_back(cand);

        // save coarse region inside candidate
        cand->regions_coarse_->push_back(coarse_regions->region_vec_->at(idx));
      }
    }
  }

}


void Tracker::collectActivePointsInCoarseRegions(){

  CameraForMapping* keyframe = dtam_->camera_vector_->at(dtam_->frame_current_);
  // iterate along all coarser levels
  for(int i=1; i<=dtam_->parameters_->coarsest_level; i++){

    RegionsWithActivePoints* coarse_regions = keyframe->regions_coarse_active_pts_vec_->at(i-1);

    // for each active points of keyframe
    for(ActivePoint* active_pt : *(keyframe->active_points_)){
      // if level of active points is less than current coarse level
      // and if active points has one min
      if(active_pt->level_<i ){

        int level_diff = i-active_pt->level_;
        // from pixel of active points find pixel at level i
        int coarse_pxl_x = active_pt->pixel_.x()/(pow(2,level_diff));
        int coarse_pxl_y = active_pt->pixel_.y()/(pow(2,level_diff));
        int idx = coarse_regions->xyToIdx(coarse_pxl_x,coarse_pxl_y);
        // push active points inside region
        coarse_regions->region_vec_->at(idx)->active_pts_vec_->push_back(active_pt);

        // save coarse region inside active points
        active_pt->regions_coarse_->push_back(coarse_regions->region_vec_->at(idx));
      }
    }
  }

}


void Tracker::collectCoarseCandidates(CameraForMapping* keyframe){

  // clear coarse candidate vec
  for(std::vector<Candidate*>* v : *(keyframe->candidates_coarse_)){
    for (Candidate* cand : *v)
      delete cand;
    v->clear();
  }
  // create candidates at coarser levels

  // iterate along all coarser levels
  for(int i=1; i<=dtam_->parameters_->coarsest_level; i++){

    RegionsWithCandidates* coarse_regions = keyframe->regions_coarse_cands_vec_->at(i-1);

    // iterate along all regions
    for ( RegionWithCandidates* reg : *(coarse_regions->region_vec_)){
      // if region is not empty
      if(!reg->cands_vec_->empty()){

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


        // iterate along collected candidates
        float invdepth, invdepth_var;
        float d_over_v_sum = 0, inv_v_sum = 0;
        int num_cands = 0;

        for(Candidate* cand : *(reg->cands_vec_)){
          if(!cand->one_min_){
            continue;
          }

          num_cands++;
          // update d_over_v_sum
          d_over_v_sum+= cand->invdepth_/cand->invdepth_var_;
          // update inv_v_sum
          inv_v_sum+= 1./cand->invdepth_var_;

          // d_over_v_sum = cand->invdepth_;
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

          // create coarse candidate
          Candidate* candidate_coarse = new Candidate(i,pixel, uv, keyframe,
                                                      c,
                                                      magn_cd,
                                                      phase_cd,
                                                      // c_dx, c_dy,
                                                      // magn_cd_dx, magn_cd_dy,
                                                      // phase_cd_dx, phase_cd_dy,
                                                      invdepth,invdepth_var,
                                                      p,p_incamframe);
          // push candidate inside coarse candidates
          keyframe->candidates_coarse_->at(i-1)->push_back(candidate_coarse);
        }
      }

    }
  }
}




bool Tracker::updateLS(Matrix6f& H, Vector6f& b, float& chi, Eigen::Matrix<float, 2,6>& jacobian_to_mul, Eigen::Matrix<float, 2,1>& jacobian_to_mul_normalizer, pixelIntensity z, pixelIntensity z_hat, Eigen::Matrix<float, 1,2>& img_jacobian, float ni, float variance, float coeff, float invdepth_var ){


  float normalizer = img_jacobian*jacobian_to_mul_normalizer;
  normalizer*=coeff; // get d r/d invdepth
  normalizer=abs(normalizer);
  normalizer *= invdepth_var; // multiply with variance
  normalizer+=0.1; // add variance on img

  // float normalizer = 1;

  // error
  float error = (z_hat-z)/normalizer;
  // float error = (z_hat-z);

  // robustifier
  float u = abs(error);
  float rho_der = huberNormDerivative(u,dtam_->parameters_->huber_threshold);
  float gamma=(1/(u+0.0001))*rho_der;


  // weight
  float weight = (ni+1.0)/(ni+(pow(error,2)/variance));


  // jacobian
  Eigen::Matrix<float, 1, 6> J; // 1 row, 6 cols
  Eigen::Matrix<float, 6, 1> Jtransp; // 1 row, 6 cols

  J=(coeff*(img_jacobian*jacobian_to_mul))/normalizer;
  // J=coeff*(img_jacobian*jacobian_to_mul);
  // J=coeff*(img_jacobian*jacobian_to_mul);
  Jtransp = J.transpose();
  // update
  H+=Jtransp*gamma*weight*J;
  b+=Jtransp*gamma*weight*error;
  chi+=error*error;
}

bool Tracker::iterationLSCands(Matrix6f& H, Vector6f& b, float& chi, Candidate* cand, CameraForMapping* frame_new, Eigen::Isometry3f& current_guess ){

  float pixels_meter_ratio = dtam_->camera_vector_->at(0)->cam_parameters_->resolution_x/dtam_->camera_vector_->at(0)->cam_parameters_->width;
  Eigen::Matrix3f K = *(frame_new->K_);
  Eigen::Matrix3f Kinv = *(frame_new->Kinv_);
  float variance = dtam_->parameters_->variance;
  int ni = dtam_->parameters_->robustifier_dofs;
  float coeff = pixels_meter_ratio/pow(2,cand->level_+1);

  // variables
  Eigen::Vector2f uv_newframe;
  Eigen::Vector2i pixel_newframe;
  Eigen::Vector3f point_newframe;
  Eigen::Vector3f* point = cand->p_;
  Eigen::Vector3f* point_incamframe = cand->p_incamframe_;
  float invdepth = cand->invdepth_;
  float invdepth_var = cand->invdepth_var_;

  point_newframe= current_guess*(*point);

  Eigen::Vector3f p_proj = K*point_newframe;
  // return false if the projected point is behind the camera
  if (p_proj.z()<frame_new->cam_parameters_->lens)
    return false;
  uv_newframe = p_proj.head<2>()*(1./p_proj.z());

  frame_new->projectPointInCamFrame( point_newframe, uv_newframe );
  frame_new->uv2pixelCoords(uv_newframe, pixel_newframe, cand->level_);

  if(!frame_new->wavelet_dec_->getWavLevel(cand->level_)->c->pixelInRange(pixel_newframe))
    return false;

  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 3,6> state_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;
  Eigen::Matrix<float, 3,1> invdepth_jacobian;

  proj_jacobian << 1./p_proj.z(), 0, -p_proj.x()/pow(p_proj.z(),2),
                   0, 1./p_proj.z(), -p_proj.y()/pow(p_proj.z(),2);

  state_jacobian << 1, 0, 0,  0                   ,  point_newframe.z()  , -point_newframe.y(),
                    0, 1, 0, -point_newframe.z() ,  0                    ,  point_newframe.x(),
                    0, 0, 1,  point_newframe.y() , -point_newframe.x()  ,  0         ;

  invdepth_jacobian << -cand->uv_.x()/pow(invdepth,2),
                         -cand->uv_.y()/pow(invdepth,2),
                         -1/pow(invdepth,2);

  jacobian_to_mul_normalizer = proj_jacobian*(K*(current_guess.linear()*(cand->cam_->frame_camera_wrt_world_->linear()*invdepth_jacobian)));

  jacobian_to_mul = (proj_jacobian*K)*state_jacobian;

  Eigen::Matrix<float, 1,2> img_jacobian;
  pixelIntensity z, z_hat;

  z = cand->intensity_;
  z_hat = frame_new->wavelet_dec_->getWavLevel(cand->level_)->c->evalPixel(pixel_newframe);
  img_jacobian << frame_new->wavelet_dec_->getWavLevel(cand->level_)->c_dx->evalPixel(pixel_newframe), frame_new->wavelet_dec_->getWavLevel(cand->level_)->c_dy->evalPixel(pixel_newframe);
  updateLS( H, b, chi, jacobian_to_mul, jacobian_to_mul_normalizer, z, z_hat, img_jacobian, ni, variance, coeff, invdepth_var );


  // z = cand->grad_magnitude_;
  // z_hat = frame_new->wavelet_dec_->getWavLevel(cand->level_)->magn_cd->evalPixel(pixel_newframe);
  // img_jacobian << frame_new->wavelet_dec_->getWavLevel(cand->level_)->magn_cd_dx->evalPixel(pixel_newframe), frame_new->wavelet_dec_->getWavLevel(cand->level_)->magn_cd_dy->evalPixel(pixel_newframe);
  // updateLS( H, b, chi, jacobian_to_mul, jacobian_to_mul_normalizer, z, z_hat, img_jacobian, ni, variance, coeff, invdepth_var );



  // // DEBUG
  // // show images
  // cand->cam_->wavelet_dec_->getWavLevel(cand->level_)->c->showImgWithCircledPixel(cand->pixel_,2*(cand->level_+1),cand->cam_->name_,2,1);
  // frame_new->wavelet_dec_->getWavLevel(cand->level_)->c->showImgWithCircledPixel(pixel_newframe,2*(cand->level_+1),frame_new->name_,2,1);
  // std::cout << "error " << error << std::endl;
  // cv::waitKey(0);
  return true;
}


bool Tracker::iterationLS(Matrix6f& H, Vector6f& b, float& chi, ActivePoint* active_pt, CameraForMapping* frame_new, Eigen::Isometry3f& current_guess ){

  float pixels_meter_ratio = dtam_->camera_vector_->at(0)->cam_parameters_->resolution_x/dtam_->camera_vector_->at(0)->cam_parameters_->width;
  Eigen::Matrix3f K = *(frame_new->K_);
  Eigen::Matrix3f Kinv = *(frame_new->Kinv_);
  float variance = dtam_->parameters_->variance;
  int ni = dtam_->parameters_->robustifier_dofs;
  float coeff = pixels_meter_ratio/pow(2,active_pt->level_+1);

  // variables
  Eigen::Vector2f uv_newframe;
  Eigen::Vector2i pixel_newframe;
  Eigen::Vector3f point_newframe;
  Eigen::Vector3f* point = active_pt->p_; // 3D point wrt world frame
  Eigen::Vector3f* point_incamframe = active_pt->p_incamframe_; // 3D point wrt cam frame
  float invdepth = active_pt->invdepth_;
  float invdepth_var = active_pt->invdepth_var_;

  point_newframe= current_guess*(*point);

  Eigen::Vector3f p_proj = K*point_newframe;
  // return false if the projected point is behind the camera
  if (p_proj.z()<frame_new->cam_parameters_->lens)
    return false;
  uv_newframe = p_proj.head<2>()*(1./p_proj.z());

  frame_new->projectPointInCamFrame( point_newframe, uv_newframe );
  frame_new->uv2pixelCoords(uv_newframe, pixel_newframe, active_pt->level_);

  if(!frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->c->pixelInRange(pixel_newframe))
    return false;

  Eigen::Matrix<float, 2,3> proj_jacobian;
  Eigen::Matrix<float, 3,6> state_jacobian;
  Eigen::Matrix<float, 2,6> jacobian_to_mul;
  Eigen::Matrix<float, 2,1> jacobian_to_mul_normalizer;
  Eigen::Matrix<float, 3,1> invdepth_jacobian;

  proj_jacobian << 1./p_proj.z(), 0, -p_proj.x()/pow(p_proj.z(),2),
                   0, 1./p_proj.z(), -p_proj.y()/pow(p_proj.z(),2);

  state_jacobian << 1, 0, 0,  0                  ,  point_newframe.z()  , -point_newframe.y(),
                    0, 1, 0, -point_newframe.z() ,  0                   ,  point_newframe.x(),
                    0, 0, 1,  point_newframe.y() , -point_newframe.x()  ,  0         ;

  invdepth_jacobian << -active_pt->uv_.x()/pow(invdepth,2),
                         -active_pt->uv_.y()/pow(invdepth,2),
                         -1/pow(invdepth,2);

  jacobian_to_mul_normalizer = proj_jacobian*(K*(current_guess.linear()*(active_pt->cam_->frame_camera_wrt_world_->linear()*invdepth_jacobian)));

  jacobian_to_mul = (proj_jacobian*K)*state_jacobian;

  Eigen::Matrix<float, 1,2> img_jacobian;
  pixelIntensity z, z_hat;

  z = active_pt->intensity_;
  z_hat = frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->c->evalPixel(pixel_newframe);
  img_jacobian << frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->c_dx->evalPixel(pixel_newframe), frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->c_dy->evalPixel(pixel_newframe);
  updateLS( H, b, chi, jacobian_to_mul, jacobian_to_mul_normalizer, z, z_hat, img_jacobian, ni, variance, coeff, invdepth_var );


  // z = active_pt->grad_magnitude_;
  // z_hat = frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->magn_cd->evalPixel(pixel_newframe);
  // img_jacobian << frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->magn_cd_dx->evalPixel(pixel_newframe), frame_new->wavelet_dec_->getWavLevel(cand->level_)->magn_cd_dy->evalPixel(pixel_newframe);
  // updateLS( H, b, chi, jacobian_to_mul, jacobian_to_mul_normalizer, z, z_hat, img_jacobian, ni, variance, coeff, invdepth_var );



  // // DEBUG
  // // show images
  // active_pt->cam_->wavelet_dec_->getWavLevel(active_pt->level_)->c->showImgWithCircledPixel(active_pt->pixel_,2*(active_pt->level_+1),active_pt->cam_->name_,2,1);
  // frame_new->wavelet_dec_->getWavLevel(active_pt->level_)->c->showImgWithCircledPixel(pixel_newframe,2*(active_pt->level_+1),frame_new->name_,2,1);
  // std::cout << "error " << error << std::endl;
  // cv::waitKey(0);
  return true;
}

void Tracker::showProjectCandsWithCurrGuess(Eigen::Isometry3f& current_guess, int level){

  CameraForMapping* frame_new = dtam_->getCurrentCamera();

  Image< colorRGB >* show_image = frame_new->wavelet_dec_->getWavLevel(level)->c->returnColoredImgFromIntensityImg("project curr guess");

  // for each active keyframe
  for(int j=0; j<dtam_->keyframe_vector_->size()-1; j++){

    int keyframe_idx = dtam_->keyframe_vector_->at(j);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);

    std::vector<Candidate*>* v;
    if(level>0){
      v= keyframe->candidates_coarse_->at(level-1);
    }
    else
      v= keyframe->candidates_;


    // for each candidate
    for(Candidate* cand : *v){
      if (cand->one_min_){
        Eigen::Vector2f uv_newframe;
        Eigen::Vector2i pixel_newframe;
        Eigen::Vector3f point_newframe;
        Eigen::Vector3f* point = cand->p_;

        point_newframe= current_guess*(*point);
        float invdepth_proj = 1.0/point_newframe.z();
        frame_new->projectPointInCamFrame( point_newframe, uv_newframe );
        frame_new->uv2pixelCoords(uv_newframe, pixel_newframe, cand->level_);

        colorRGB color = frame_new->invdepthToRgb(invdepth_proj);
        show_image->setPixel( pixel_newframe, color);
      }
    }

  }

  show_image->show(2*pow(2,level));
  cv::waitKey(0);
}


void Tracker::showProjectActivePtsWithCurrGuess(Eigen::Isometry3f& current_guess, int level){

  CameraForMapping* frame_new = dtam_->getCurrentCamera();

  Image< colorRGB >* show_image = frame_new->wavelet_dec_->getWavLevel(level)->c->returnColoredImgFromIntensityImg("project curr guess, "+frame_new->name_);

  // for each active keyframe
  for(int j=0; j<dtam_->keyframe_vector_->size()-1; j++){

    int keyframe_idx = dtam_->keyframe_vector_->at(j);
    CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);

    std::vector<ActivePoint*>* v;
    if(level>0){
      v= keyframe->active_points_coarse_->at(level-1);
    }
    else
      v= keyframe->active_points_;


    // for each active point
    for(ActivePoint* active_pt : *v){

      Eigen::Vector2f uv_newframe;
      Eigen::Vector2i pixel_newframe;
      Eigen::Vector3f point_newframe;
      Eigen::Vector3f* point = active_pt->p_0_;

      point_newframe= current_guess*(*point);
      float invdepth_proj = 1.0/point_newframe.z();
      frame_new->projectPointInCamFrame( point_newframe, uv_newframe );
      frame_new->uv2pixelCoords(uv_newframe, pixel_newframe, active_pt->level_);

      colorRGB color = frame_new->invdepthToRgb(invdepth_proj);
      show_image->setPixel( pixel_newframe, color);

    }

  }

  show_image->show(2*pow(2,level));
  cv::waitKey(0);
}


void Tracker::filterOutOcclusionsGT(){


    CameraForMapping* frame_new = dtam_->getCurrentCamera();
    Eigen::Isometry3f* framenew_grountruth_pose = frame_new->grountruth_camera_->frame_world_wrt_camera_;

    // for each active keyframe
    for(int j=0; j<dtam_->keyframe_vector_->size()-1; j++){

      int keyframe_idx = dtam_->keyframe_vector_->at(j);
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);

      std::vector<Candidate*>* v;
      v= keyframe->candidates_;

      // for each candidate
      for(int i=v->size()-1; i>=0; i--){

        Candidate* cand = v->at(i);
        if (cand->one_min_){
          Eigen::Vector2f uv_newframe;
          Eigen::Vector2i pixel_newframe;
          Eigen::Vector3f point_newframe;
          Eigen::Vector3f* point = cand->p_;

          point_newframe= (*framenew_grountruth_pose)*(*point);
          frame_new->projectPointInCamFrame( point_newframe, uv_newframe );
          frame_new->uv2pixelCoords(uv_newframe, pixel_newframe);
          float invdepth_val = frame_new->grountruth_camera_->invdepth_map_->evalPixel(pixel_newframe);

          float invdepth_proj = 1.0/point_newframe.z();
          float invdepth_gt = invdepth_val/frame_new->cam_parameters_->min_depth;

          if (abs(invdepth_gt-invdepth_proj)>0.01){
            cand->marginalize();
          }

        }
      }

    }


}

void Tracker::trackWithActivePoints(Eigen::Isometry3f& current_guess, bool debug_tracking, CameraForMapping* frame_new){

  Matrix6f H;
  Vector6f b;
  float chi;

  // for each coarse level
  for(int i=dtam_->parameters_->coarsest_level; i>=0; i--){

    int iterations = 0;

    if(debug_tracking)
      std::cout << "\nLevel: " << i << std::endl;

    std::vector<float> chi_vec;
    float first_chi_der = 0;

    while(iterations<dtam_->parameters_->max_iterations_ls){

      H.setZero();
      b.setZero();
      chi=0;

      //DEBUG
      if(debug_tracking){
        showProjectActivePtsWithCurrGuess(current_guess, i);
      }


      // for each active keyframe
      for(int j=0; j<dtam_->keyframe_vector_->size()-1; j++){

        int keyframe_idx = dtam_->keyframe_vector_->at(j);
        // std::cout << "keyframe " << keyframe_idx << std::endl;

        CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);

        std::vector<ActivePoint*>* v;
        if(i>0){
          v= keyframe->active_points_coarse_->at(i-1);
        }
        else
          v= keyframe->active_points_;

        // for each candidate
        for(ActivePoint* active_pt : *v){
          iterationLS( H, b, chi, active_pt, frame_new, current_guess );
        }

      }
      Vector6f dx=-H.inverse()*b;
      Eigen::Isometry3f new_guess;
      new_guess=v2t(dx)*current_guess;
      chi_vec.push_back(chi);

      // DEBUG
      if(debug_tracking){
        std::cout << "chi " << chi << std::endl;
      }


      if (chi_vec.size()<=2){
        // first_chi_der=chi_vec.at(0)-chi_vec.at(1);
        current_guess=new_guess;
      }
      // if (iterations<dtam_->parameters_->max_iterations_ls/2){
      //   current_guess=new_guess;
      // }
      else {
        if( (chi_vec.at(chi_vec.size()-2)-chi)<0  ){
          break;
        }
        current_guess=new_guess;
      }

      iterations++;


    }
    // update initial guess for next level
    // std::cout << "H: " << H << std::endl;
     // break;
     // std::cout << "ao? " << std::endl;
  }

}

void Tracker::trackWithCandidates(Eigen::Isometry3f& current_guess, bool debug_tracking, CameraForMapping* frame_new){

    Matrix6f H;
    Vector6f b;
    float chi;

    // for each coarse level
    for(int i=dtam_->parameters_->coarsest_level; i>=0; i--){

      int iterations = 0;

      if(debug_tracking)
        std::cout << "\nLevel: " << i << std::endl;

      std::vector<float> chi_vec;
      float first_chi_der = 0;

      while(iterations<dtam_->parameters_->max_iterations_ls){

        H.setZero();
        b.setZero();
        chi=0;

        //DEBUG
        if(debug_tracking)
          showProjectCandsWithCurrGuess(current_guess, i);


        // for each active keyframe
        for(int j=0; j<dtam_->keyframe_vector_->size()-1; j++){

          int keyframe_idx = dtam_->keyframe_vector_->at(j);
          // std::cout << "keyframe " << keyframe_idx << std::endl;

          CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);

          std::vector<Candidate*>* v;
          if(i>0){
            v= keyframe->candidates_coarse_->at(i-1);
          }
          else
            v= keyframe->candidates_;

          // for each candidate
          for(Candidate* cand : *v){

            if (cand->one_min_){
              iterationLSCands( H, b, chi, cand, frame_new, current_guess );
            }

          }

        }
        Vector6f dx=-H.inverse()*b;
        Eigen::Isometry3f new_guess;
        new_guess=v2t(dx)*current_guess;
        chi_vec.push_back(chi);

        // DEBUG
        if(debug_tracking){
          std::cout << "chi " << chi << std::endl;
        }


        if (chi_vec.size()<=2){
          // first_chi_der=chi_vec.at(0)-chi_vec.at(1);
          current_guess=new_guess;
        }
        // if (iterations<dtam_->parameters_->max_iterations_ls/2){
        //   current_guess=new_guess;
        // }
        else {
          if( (chi_vec.at(chi_vec.size()-2)-chi)<0  ){
            break;
          }
          current_guess=new_guess;
        }

        iterations++;


      }
      // update initial guess for next level
      // std::cout << "H: " << H << std::endl;
       // break;
       // std::cout << "ao? " << std::endl;
    }

}

Eigen::Isometry3f Tracker::doLS(Eigen::Isometry3f& initial_guess, bool track_candidates, bool debug_tracking){

  // get new frame
  CameraForMapping* frame_new = dtam_->getCurrentCamera();

  Eigen::Isometry3f current_guess = initial_guess;


  if(track_candidates){

    // filterOutOcclusionsGT();
    for(int keyframe_idx : *(dtam_->keyframe_vector_)){
      CameraForMapping* keyframe = dtam_->camera_vector_->at(keyframe_idx);
      collectCoarseCandidates(keyframe);
      // for (int i=keyframe->candidates_coarse_->size(); i>0; i--){
      //   keyframe->showCoarseCandidates(i,1);
      // }
      // showProjectCandsWithCurrGuess(initial_guess, 0);
      // // filterOutOcclusionsGT();
      // // showProjectCandsWithCurrGuess(initial_guess, 0);
      // // keyframe->showCandidates(2);
      // cv::waitKey(0);
    }

    trackWithCandidates(current_guess, debug_tracking, frame_new);
  }
  else{

    trackWithActivePoints(current_guess, debug_tracking, frame_new);
  }

  return current_guess;
}

void Tracker::trackCam(bool takeGtPoses, bool track_candidates, int guess_type, bool debug_tracking){
  if(takeGtPoses){
    trackGroundtruth();
  }
  else{
    trackLS(track_candidates, guess_type, debug_tracking);
  }
}

Eigen::Isometry3f Tracker::computeInitialGuess( int guess_type){
  Eigen::Isometry3f pose_initial_guess;

  if( guess_type==POSE_CONSTANT)
    pose_initial_guess = poseConstantModel();
  else if ( guess_type==VELOCITY_CONSTANT)
    pose_initial_guess = velocityConstantModel();


  return pose_initial_guess;
}

Eigen::Isometry3f Tracker::computeInitialGuessGT( ){
  Eigen::Isometry3f pose_initial_guess;

  pose_initial_guess = *(dtam_->environment_->camera_vector_->at(dtam_->frame_current_)->frame_world_wrt_camera_);


  return pose_initial_guess;
}

Eigen::Isometry3f Tracker::poseConstantModel(){

  CameraForMapping* last_cam = dtam_->getLastCamera();

  Eigen::Isometry3f* last_T_w = last_cam->frame_world_wrt_camera_;

  Eigen::Isometry3f curr_T_w =  *last_T_w;

  return curr_T_w;
}

Eigen::Isometry3f Tracker::velocityConstantModel(){

  CameraForMapping* last_cam = dtam_->getLastCamera();
  CameraForMapping* prev_cam = dtam_->getSecondLastCamera();
  CameraForMapping* lastkeyframe_cam = dtam_->getLastKeyframe();

  Eigen::Isometry3f* last_T_w = last_cam->frame_world_wrt_camera_;
  Eigen::Isometry3f* w_T_prev = prev_cam->frame_camera_wrt_world_;

  Eigen::Isometry3f last_T_prev = (*last_T_w)*(*w_T_prev);

  Eigen::Isometry3f* w_T_lastkeyframe = lastkeyframe_cam->frame_camera_wrt_world_;

  // we assume that the relative pose between last and previous frame
  // is the same between current and last (to track) frame (constant velocity model)
  // so last_T_prev = curr_T_last

  // the initial guess is the pose of the last keyframe wrt current camera
  // so iitial_guess = curr_T_lastkeyframe = curr_T_last*last_T_lastkeyframe
  // so curr_T_w = curr_T_last*last_T_w
  // since curr_T_last = last_T_prev -> curr_T_w = last_T_prev*last_T_w
  Eigen::Isometry3f curr_T_w =  last_T_prev*(*last_T_w);

  return curr_T_w;
}
