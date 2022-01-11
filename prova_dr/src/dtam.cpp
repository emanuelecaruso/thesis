#include "mapper.h"
#include "dtam.h"
#include "json.hpp"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>
#include <fstream>
using json = nlohmann::json;



void Dtam::debugAllCameras(bool show_imgs){

  sharedCoutDebug("DEBUGGING ALL CAMERAS:");
  sharedCoutDebug("camera vector size: " + std::to_string(camera_vector_->size()));

  for(Camera* camera : *camera_vector_){
    camera->printMembers();
    if (show_imgs){
      camera->showRGB();
      camera->showDepthMap();
    }
  }
  cv::waitKey(0);
}

CameraForMapping* Dtam::getCurrentCamera() {
  return camera_vector_->at(frame_current_);
}

CameraForMapping* Dtam::getPreviousCamera() {
  return camera_vector_->at(frame_current_-1);
}

void Dtam::addCamera(int counter){

  // CameraForStudy* env_cam=environment_->camera_vector_->at(counter);
  Camera* env_cam=environment_->camera_vector_->at(counter);
  CameraForMapping* new_cam=new CameraForMapping (env_cam, parameters_);
  new_cam->regions_->collectCandidates();
  camera_vector_->push_back(new_cam);
}



void Dtam::waitForNewFrame(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  frame_updated_.wait(locker);
  locker.unlock();
}

void Dtam::waitForTrackedCandidates(){
  std::unique_lock<std::mutex> locker(mu_candidate_tracking_);
  cand_tracked_.wait(locker);
  locker.unlock();
}

void Dtam::waitForInitialization(){
  std::unique_lock<std::mutex> locker(mu_initialization_);
  initialization_done_.wait(locker);
  locker.unlock();
}

void Dtam::doInitialization(bool initialization_loop){
  bool initialization_done = false;

  while(true){
    if(frame_current_==camera_vector_->size()-1)
      waitForNewFrame();

    double t_start=getTime();

    frame_current_++;

    int frame_delay = camera_vector_->size()-frame_current_-1;
    sharedCoutDebug("\nINITIALIZATION of Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));

    if(frame_current_==0){
      tracker_->trackCam(true);
      keyframe_handler_->addKeyframe(true);
      initializer_->compute_cv_K();
      initializer_->extractCorners();
      mapper_->updateRotationalInvariantGradients();
      mapper_->selectNewCandidates();
    }
    else{
      initializer_->trackCornersLK();
      // if pose is found ...
      if(initializer_->findPose()){

        if(!initialization_loop){
          // ... add last keyframe
          keyframe_handler_->addKeyframe(true);
          // start initializing the model
          mapper_->updateRotationalInvariantGradients();
          mapper_->trackExistingCandidates();
          mapper_->selectNewCandidates();
          initialization_done=true;
        }

      }
    }

    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("   - INITIALIZATION of frame "+std::to_string(frame_current_)+", computation time: "+ std::to_string(deltaTime)+" ms");

    if(initialization_done){
      initialization_done_.notify_all();
      sharedCoutDebug("\nINITIALIZATION ENDED");
      break;
    }

  }
}

void Dtam::doFrontEndPart(bool all_keyframes, bool wait_for_initialization, bool take_gt_poses,bool const_acc){
  if(wait_for_initialization)
    waitForInitialization();

  while( true ){

    if(frame_current_==camera_vector_->size()-1)
      waitForNewFrame();

    double t_start=getTime();

    frame_current_++;

    int frame_delay = camera_vector_->size()-frame_current_-1;
    sharedCoutDebug("\nFRONT END for Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));

    if(frame_current_==0){
      tracker_->trackCam(true);
      keyframe_handler_->addKeyframe(true);
      mapper_->updateRotationalInvariantGradients();
      mapper_->selectNewCandidates();
      continue;
    }

    // sharedCoutDebug("Front end part of frame: "+std::to_string(frame_current_)+" ...");

    tracker_->trackCam(take_gt_poses);
    if(keyframe_handler_->addKeyframe(all_keyframes)){

      mapper_->updateRotationalInvariantGradients();
      // bundle_adj_->projectAndMarginalizeActivePoints();
      mapper_->trackExistingCandidates();
      cand_tracked_.notify_all();
      mapper_->selectNewCandidates();
      // bundle_adj_->activateNewPoints(); in other thread
      // bundle_adj_->optimize(); in other thread
    }
    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("FRONT END part of frame: "+std::to_string(frame_current_)+", time: "+ std::to_string(deltaTime)+" ms");

  }

}

void Dtam::doOptimization(bool active_all_candidates){
  while(frame_current_<camera_vector_->size() || !end_flag_ ){

    bundle_adj_->activateNewPoints();
  }
}


void Dtam::doTracking(){

}

void Dtam::updateCamerasFromEnvironment(){

  float fps=environment_->fps_;
  int counter=0;

  while(counter< environment_->camera_vector_->size()){

    std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    // sharedCout("\nFrame: "+ std::to_string(counter));
    addCamera(counter);

    frame_updated_.notify_all();

    double t_end=getTime();
    locker.unlock();

    int deltaTime=(t_end-t_start);
    // sharedCoutDebug("\nAdd Frame "+std::to_string(counter)+", computation time: "+ std::to_string(deltaTime)+" ms");
    long int waitDelay=deltaTime*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    if(time_to_wait>0)
      std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
    // else
      // sharedCoutDebug("Delay accumulated! : "+ std::to_string(-time_to_wait)+" ms");


    counter++;

  }
  end_flag_=true;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  sharedCout("\nVideo stream ended");

}

void Dtam::testRotationalInvariance(){

  CameraForMapping* cam0=camera_vector_->at(0);
  for( int i=1; i<camera_vector_->size(); i++)
  {
    CameraForMapping* cam=camera_vector_->at(i);

    Image<pixelIntensity>* dv_ = new Image<pixelIntensity>("dh");
    Image<pixelIntensity>* dh_ = new Image<pixelIntensity>("dv");
    Image<pixelIntensity>* dv_inv = new Image<pixelIntensity>("dh inv");
    Image<pixelIntensity>* dh_inv = new Image<pixelIntensity>("dv inv");
    dv_->image_=cam->wavelet_dec_->vector_wavelets->at(0)->dv->image_.clone();
    dh_->image_=cam->wavelet_dec_->vector_wavelets->at(0)->dh->image_.clone();
    dv_inv->image_=cam->wavelet_dec_->vector_wavelets->at(0)->dv_robust->image_.clone();
    dh_inv->image_=cam->wavelet_dec_->vector_wavelets->at(0)->dh_robust->image_.clone();

    Eigen::Isometry3f T = (*(cam->frame_camera_wrt_world_));
    Eigen::Matrix3f R=T.linear();

    // float rollAngle_= -atan2(-R(0,1),R(0,0));
    // float rollAngleCam0= -atan2(-Rcam0(0,1),Rcam0(0,0));
    float rollAngle= -atan2(R(1,0),R(1,1));

    float c=cos(rollAngle);
    float s=sin(rollAngle);

    cv::multiply(dh_->image_, cv::Scalar(1./8.,1./8.,1./8.), dh_->image_);
    cv::multiply(dv_->image_, cv::Scalar(1./8.,1./8.,1./8.), dv_->image_);
    cv::add(dh_->image_, cv::Scalar(0.5,0.5,0.5), dh_->image_);
    cv::add(dv_->image_, cv::Scalar(0.5,0.5,0.5), dv_->image_);
    dv_->show(2,"dv");
    dh_->show(2,"dh");


    cv::multiply(dh_inv->image_, cv::Scalar(1./8.,1./8.,1./8.), dh_inv->image_);
    cv::multiply(dv_inv->image_, cv::Scalar(1./8.,1./8.,1./8.), dv_inv->image_);
    cv::add(dh_inv->image_, cv::Scalar(0.5,0.5,0.5), dh_inv->image_);
    cv::add(dv_inv->image_, cv::Scalar(0.5,0.5,0.5), dv_inv->image_);

    dh_inv->show(2,"dh_inv");
    dv_inv->show(2,"dv_inv");

    // cam->wavelet_dec_->vector_wavelets->at(0)->magnitude_img->show();
    cv::waitKey(0);
  }
  cv::waitKey(0);
}

void Dtam::eval_initializer(){

  bool initialization_loop=true;

  std::thread initialization_thread_(&Dtam::doInitialization, this, initialization_loop);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);

  update_cameras_thread_.join();
  initialization_thread_.detach();

  // initializer_->showCornersRef();
  initializer_->showCornersTrack();
  cv::waitKey(0);

}

void Dtam::test_mapping(){

  bool take_gt_poses=true;
  bool const_acc=true;
  bool all_keyframes=true;
  bool wait_for_initialization=false;
  // bool active_all_candidates=true;

  // std::thread optimization_thread(&Dtam::doOptimization, this, active_all_candidates);
  std::thread frontend_thread_(&Dtam::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, const_acc);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);


  // optimization_thread.detach();
  frontend_thread_.detach();
  update_cameras_thread_.join();

  // debugAllCameras();


  // camera_vector_->at(0)->image_intensity_->show(2);
  // camera_vector_->at(0)->wavelet_dec_->vector_wavelets->at(2)->c->show(2);
  // camera_vector_->at(0)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(2);
  // camera_vector_->at(1)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(4,"1");
  // camera_vector_->at(0)->showCandidates_1(2);
  // camera_vector_->at(0)->showCandidates_2(2);
  // camera_vector_->at(camera_vector_->size()-2)->showProjCandidates_2(2);
  camera_vector_->at(0)->showCandidates_2(2);
  // camera_vector_->at(0)->showCandidates_2(2);
  // camera_vector_->at(7)->showProjCandidates_2(2);
  camera_vector_->at(5)->showProjCandidates_2(2);
  // camera_vector_->at(1)->showProjCandidates_2(2);
  // camera_vector_->at(keyframe_vector_->back())->regions_->region_vec_->at(1)->showRegion(2);

  // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(camera_vector_->size()-2));
  makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));

  // testRotationalInvariance();
  cv::waitKey(0);

}

void Dtam::test_dso(){

  bool initialization_loop=false;
  bool take_gt_poses=true;
  bool all_keyframes=true;
  bool const_acc=true;
  bool wait_for_initialization=true;
  // bool active_all_candidates=true;

  std::thread frontend_thread_(&Dtam::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, const_acc);
  std::thread initialization_thread_(&Dtam::doInitialization, this, initialization_loop);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);

  frontend_thread_.join();
  update_cameras_thread_.join();
  initialization_thread_.detach();

  // initializer_->showCornersRef();
  // initializer_->showCornersTrack();
  // cv::waitKey(0);

  // bool all_keyframes=true;
  //
  // // std::thread frontend_thread(&Dtam::doFrontEndPart, this, all_keyframes);
  // std::thread initialization_thread_(&Dtam::doInitialization, this);
  // std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);
  //
  // // frontend_thread.join();
  // update_cameras_thread_.join();
  // initialization_thread_.detach();
  //
  //
  // cv::waitKey(0);

}


// TODO remove
bool Dtam::makeJsonForCands(const std::string& path_name, CameraForMapping* camera){


    const char* path_name_ = path_name.c_str(); // dataset name
    struct stat info;
    if( stat( path_name_, &info ) != 0 )
    { }
    else if( info.st_mode & S_IFDIR )
    {
      // isdir
      return 0;
    }
    else
    {
      // printf( "%s is not a directory\n", path_name );
      std::string st = "rm " + path_name;
      const char *str = st.c_str();

    }

    std::string st = "touch "+path_name;
    const char *str = st.c_str();

    json j;

    j["cameras"][camera->name_];
    int count=0;
    for(RegionWithProjCandidates* reg : *(camera->regions_projected_cands_->region_vec_) ){
      for(CandidateProjected* cand : *(reg->cands_vec_)){
        int level = cand->level_;
        Eigen::Vector2f uv = cand->uv_ ;
        Eigen::Vector3f p;
        camera->pointAtDepth( uv, cand->depth_, p);
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << count;
        std::string idx = ss.str();
        j["cameras"][camera->name_]["p"+idx] = {
          {"level", level},
          {"depth_var", cand->depth_var_},
          {"position", {p[0],p[1],p[2]}}
        };
        count++;
      }

    }
    // write prettified JSON to another file
    std::ofstream o(path_name);
    o << std::setw(4) << j << std::endl;
    o.close();

    return 1;
    // int rows=camera->parameters_->resolution_y;
    // int cols=camera->parameters_->resolution_x;
    // int n_pixels=rows*cols;
    //
    // for
    //
    // for (int i=0; i<n_pixels; i++){
    //   Cp_gpu cp= camera->cp_array_[i];
    //   if (cp.valid){
    //     std::stringstream ss;
    //     ss << std::setw(6) << std::setfill('0') << i;
    //     std::string idx = ss.str();
    //     j["cameras"][camera->name_]["p"+idx] = {
    //       {"color", {cp.color[0],cp.color[1],cp.color[2]}},
    //       {"position", {cp.point[0],cp.point[1],cp.point[2]}}
    //     };
    //   }
    // }
    // // write prettified JSON to another file
    // std::ofstream o(path_name);
    // o << std::setw(4) << j << std::endl;
    // o.close();
    //
    // double t_e=getTime();
    // double delta=t_e-t_s;
    // return delta;


}
