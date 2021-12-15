#include "mapper.h"
#include "dtam.h"
#include "json.hpp"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>
#include <fstream>.
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

void Dtam::addCamera(int counter){

  CameraForStudy* env_cam=environment_->camera_vector_->at(counter);
  CameraForMapping* new_cam=new CameraForMapping (env_cam, parameters_);
  new_cam->regions_->collectCandidates();
  camera_vector_->push_back(new_cam);
}

int Dtam::getFrameCurrent(){
  return frame_current_;
}

void Dtam::waitForNewFrame(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  frame_updated_.wait(locker);
  locker.unlock();
}

void Dtam::waitForTrackedCandidates(){
  std::unique_lock<std::mutex> locker(mu_frame_);
  cand_tracked_.wait(locker);
  locker.unlock();
}

void Dtam::doInitialization(bool all_keyframes, bool take_gt_poses){
  while( true ){

    if(frame_current_==camera_vector_->size()-1)
      waitForNewFrame();

    double t_start=getTime();

    frame_current_++;
    sharedCoutDebug("Frame current: "+std::to_string(frame_current_));

    if(frame_current_==0){
      tracker_->trackCam(take_gt_poses);
      keyframe_handler_->addKeyframe(all_keyframes);
      mapper_->selectNewCandidates();
      continue;
    }

    sharedCoutDebug("Initialization of frame: "+std::to_string(frame_current_)+" ...");

    tracker_->trackCam(take_gt_poses);
    if(keyframe_handler_->addKeyframe(all_keyframes)){
      // bundle_adj_->projectAndMarginalizeActivePoints();
      mapper_->trackExistingCandidates();
      cand_tracked_.notify_all();
      mapper_->selectNewCandidates();
      // bundle_adj_->activateNewPoints(); in other thread
      // bundle_adj_->optimize(); in other thread
    }
    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("Initialization of frame: "+std::to_string(frame_current_)+", time: "+ std::to_string(deltaTime)+" ms");

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

    sharedCout("\nFrame: "+ std::to_string(counter));
    addCamera(counter);

    frame_updated_.notify_all();

    double t_end=getTime();
    locker.unlock();

    int deltaTime=(t_end-t_start);
    sharedCoutDebug("Add camera computation time: "+ std::to_string(deltaTime)+" ms");
    long int waitDelay=deltaTime*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    if(time_to_wait>0)
      std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
    else
      sharedCoutDebug("Delay accumulated! : "+ std::to_string(-time_to_wait)+" ms");


    counter++;
  }
  end_flag_=true;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  sharedCout("\nVideo stream ended");

}

void Dtam::test_mapping(){

  bool take_gt_poses=true;
  bool all_keyframes=true;
  bool active_all_candidates=true;

  std::thread optimization_thread(&Dtam::doOptimization, this, active_all_candidates);
  std::thread initialization_thread(&Dtam::doInitialization, this, all_keyframes, take_gt_poses);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);


  optimization_thread.join();
  initialization_thread.join();
  update_cameras_thread_.join();

  // debugAllCameras();


  // camera_vector_->at(0)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(4,"0");
  // camera_vector_->at(1)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(4,"1");
  // camera_vector_->at(0)->showCandidates_1(2);
  // camera_vector_->at(0)->showCandidates_2(2);
  // camera_vector_->at(camera_vector_->size()-2)->showProjCandidates_2(2);
  camera_vector_->at(1)->showProjCandidates_2(2);
  // camera_vector_->at(1)->showProjCandidates_2(2);
  // camera_vector_->at(keyframe_vector_->back())->regions_->region_vec_->at(1)->showRegion(2);

  // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(camera_vector_->size()-2));
  makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(1));

  cv::waitKey(0);


}

void Dtam::testFeatures(){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  float size=1;
  // float size=1.3;image_
  // float size=2;

  for (int i=1; i<environment_->camera_vector_->size(); i++){
    showFeatures(i, size);
    cv::waitKey(0);
  }
};

void Dtam::showFeatures(int idx, float size=1){
  CameraForStudy* cam_1=environment_->camera_vector_->at(0);
  CameraForStudy* cam_2=environment_->camera_vector_->at(idx);
  // cam_1->image_rgb_->showWithOtherImage(cam_2->image_rgb_,"image comparison",size);
  cam_2->wavelet_dec_->showWaveletDec(size);
  // cam_1->wavelet_dec_->compareThreshold(0.1,size);
  // cam_1->curvature_->showWithOtherImage(cam_2->curvature_,"curvature comparison",size);
  // cam_1->grad_intensity_->showWithOtherImage(cam_2->grad_intensity_,"grad intensity comparison",size);
  // cam_1->grad_x_->showWithOtherImage(cam_2->grad_x_,"grad x comparison",size);
  // cam_1->grad_x_->getChannel(2)->showWithOtherImage(cam_2->grad_x_->getChannel(2),"r grad x comparison",size);
  // cam_1->grad_x_->getChannel(1)->showWithOtherImage(cam_2->grad_x_->getChannel(1),"g grad x comparison",size);
  // cam_1->grad_x_->getChannel(0)->showWithOtherImage(cam_2->grad_x_->getChannel(0),"b grad x comparison",size);
  // cam_1->grad_y_->showWithOtherImage(cam_2->grad_y_,"grad y comparison",size);
  // cam_1->grad_robust_x_->showWithOtherImage(cam_2->grad_robust_x_,"grad x rob comparison",size);


};

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
      // std::string
      system(str);
    }

    std::string st = "touch "+path_name;
    const char *str = st.c_str();
    system(str);

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
