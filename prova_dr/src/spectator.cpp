#include "spectator.h"
#include <thread>
#include <vector>
#include <mutex>

CamParameters* Spectator::initCamParams(Params* parameters){
  CamParameters* spectator_params = new CamParameters(
  parameters->spec_resolution_x, parameters->spec_resolution_y, parameters->spec_width,
  parameters->spec_lens, parameters->spec_min_depth, parameters->spec_max_depth);
  return spectator_params;
}

Camera* Spectator::initCam(Params* parameters){
  Camera* spectator_cam = new Camera("Spectator", spectator_params_, nullptr);
  return spectator_cam;
}

Image<colorRGB>* Spectator::initImage(Params* parameters){
  Image<colorRGB>* spectator_image = new Image<colorRGB> ("Spectator");
  spectator_image->initImage(parameters->spec_resolution_y, parameters->spec_resolution_x);
  spectator_image->setAllPixels(background_color_);
  return spectator_image;
}

void Spectator::spectateDso(){
  dtam_->waitForNewFrame();

  while(true){
    renderState();
    showSpectator();
  }
}

void Spectator::renderState(){
  reinitSpectator();
  renderPoints();
  renderCamsAndKFs();
  renderPoints();
}

void Spectator::showSpectator(){
  spectator_image_->show(1);
  cv::waitKey(1);

}

void Spectator::reinitSpectator(){
  spectator_image_->setAllPixels(background_color_);
  Eigen::Isometry3f pose = getSpectatorPose();
  spectator_cam_->assignPose(pose);
}

void Spectator::renderPoints(){
  // iterate through all cameras
  for ( int i = 0; i<dtam_->camera_vector_->size(); i++){
    CameraForMapping* cam = dtam_->camera_vector_->at(i);
    if (cam->keyframe_){
      // iterate through all marginalized points
      for ( int j = 0; j<cam->marginalized_points_->size(); j++){
        ActivePoint* marg_pt = cam->marginalized_points_->at(j);
        plotPt(marg_pt, black);
      }

      std::vector<int>* v = dtam_->keyframe_vector_;
      if (std::count(v->begin(), v->end(), i)) {
        for ( int j = 0; j<cam->active_points_->size(); j++){
          ActivePoint* act_pt = cam->active_points_->at(j);
          plotPt(act_pt, red);
        }
      }
    }
  }

}

void Spectator::renderCamsAndKFs(){
  // iterate through all cameras
  for ( int i = 0; i<dtam_->camera_vector_->size(); i++){
    CameraForMapping* cam = dtam_->camera_vector_->at(i);
    if (cam->keyframe_){
      std::vector<int>* v = dtam_->keyframe_vector_;
      if (std::count(v->begin(), v->end(), i)) {
        plotCam(cam, red);
      }
      else{
        plotCam(cam, green);
      }
    }
    else{
      plotCam(cam, blue);
    }
  }
}


Eigen::Isometry3f Spectator::getSpectatorPose(){
  // get first active keyframe
  CameraForMapping* first_keyframe = dtam_->camera_vector_->at(dtam_->keyframe_vector_->back());
  // distance of spectator wrt first keyframe
  float spec_distance = dtam_->parameters_->spec_distance;
  // spec wrt first keyframe
  Eigen::Isometry3f kf1_T_spec;
  kf1_T_spec.linear().setIdentity();
  kf1_T_spec.translation() << 0,0,-spec_distance;

  Eigen::Isometry3f pose =  (*(first_keyframe->frame_camera_wrt_world_))*kf1_T_spec;

  return pose;

}

bool Spectator::plotPt(ActivePoint* pt, const colorRGB& color){
  plotPt(*(pt->p_), color);
}

bool Spectator::plotPt(Eigen::Vector3f& pt, const colorRGB& color){
  Eigen::Vector2f uv;
  float depth;
  bool pt_in_front = spectator_cam_->projectPoint(pt,uv,depth);
  if(!pt_in_front)
    return false;

  pxl pixel_coords;
  spectator_cam_->uv2pixelCoords(uv, pixel_coords);

  if(spectator_image_->pixelInRange(pixel_coords)){
    spectator_image_->drawCircle( color, pixel_coords);
    return true;
  }
  return false;

}

bool Spectator::plotCam(CameraForMapping* cam, const colorRGB& color){
  // w_t_cam cam_t_angle t+Rt
	// plotPt( cam→t, color )
	// tl_corn = cam->t+cam->R*(x+y+depth)
	// tr_corn
	// ...
	// plotPt( cam→( tl_corn ) , color )
	// plotPt( cam→( tr_corn , color )
	// ...
	// plotLine(tl_corn, tr_corn)
  Eigen::Matrix3f R = cam->frame_camera_wrt_world_->linear();
  Eigen::Vector3f t = cam->frame_camera_wrt_world_->translation();
  float delta = dtam_->parameters_->rendered_cams_size;
  // Eigen::Vector3f _tl = t+R*
  plotPt( t , color );

}
