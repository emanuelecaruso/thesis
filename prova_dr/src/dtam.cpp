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

CameraForMapping* Dtam::getLastCamera() {
  return camera_vector_->at(frame_current_-1);
}

CameraForMapping* Dtam::getSecondLastCamera() {
  return camera_vector_->at(frame_current_-2);
}

CameraForMapping* Dtam::getLastKeyframe() {
  return camera_vector_->at(keyframe_vector_->back());
}

void Dtam::addCamera(int counter){

  // CameraForStudy* env_cam=environment_->camera_vector_->at(counter);
  Camera* env_cam=environment_->camera_vector_->at(counter);
  CameraForMapping* new_cam=new CameraForMapping (env_cam, parameters_);
  // new_cam->regions_sampling_->collectCandidates(parameters_->wavelet_levels);
  new_cam->regions_sampling_->collectCandidates(1);
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

void Dtam::waitForPointActivation(){
  std::unique_lock<std::mutex> locker(mu_point_activation_);
  points_activated_.wait(locker);
  locker.unlock();
}

void Dtam::waitForOptimization(){
  std::unique_lock<std::mutex> locker(mu_optimization_);
  optimization_done_.wait(locker);
  locker.unlock();
}

void Dtam::doInitialization(bool initialization_loop, bool debug_initialization, bool debug_mapping, bool track_candidates, bool take_gt_points){
  bool initialization_done = false;

  while( true ){

    if(frame_current_==int(camera_vector_->size())-1){
      waitForNewFrame();
    }


    // if still frame current is last camera, new frame is the end signal
    if(frame_current_==int(camera_vector_->size())-1)
      break;

    double t_start=getTime();

    frame_current_++;

    int frame_delay = camera_vector_->size()-frame_current_-1;
    sharedCoutDebug("\nINITIALIZATION of Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));

    if(frame_current_==0){
      tracker_->trackCam(true);
      keyframe_handler_->addFirstKeyframe();
      initializer_->extractCorners();
      if(debug_initialization)
        initializer_->showCornersTrackCurr();
      mapper_->selectNewCandidates();

      if(track_candidates)
        tracker_->collectCandidatesInCoarseRegions();

    }
    else{
      initializer_->trackCornersLK();

      // if a good pose is found ...
      if(initializer_->findPose()){
        sharedCoutDebug("   - Pose found");
        if(debug_initialization)
          initializer_->showCornersTrackCurr();

        if(!initialization_loop){
          // ... add last keyframe
          keyframe_handler_->addKeyframe(true);

          // start initializing the model
          // mapper_->trackExistingCandidates(false,debug_mapping);
          mapper_->trackExistingCandidates(take_gt_points,debug_mapping);

          cand_tracked_.notify_all(); // after candidates are tracked notify for ba


          mapper_->selectNewCandidates();

          if(track_candidates)
            tracker_->collectCandidatesInCoarseRegions();

          initialization_done=true;
        }

      }
    }

    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("   - INITIALIZATION of frame "+std::to_string(frame_current_)+", computation time: "+ std::to_string(deltaTime)+" ms");

    if(initialization_done){
      sharedCoutDebug("\nINITIALIZATION ENDED");
      initialization_done_.notify_all();
      break;
    }

  }

}

void Dtam::doFrontEndPart(bool all_keyframes, bool wait_for_initialization, bool take_gt_poses, bool take_gt_points, bool track_candidates, int guess_type, bool debug_mapping, bool debug_tracking){
  if(wait_for_initialization)
    waitForInitialization();
  while( true ){

    // if the frame is the latest
    if(frame_current_==int(camera_vector_->size())-1){

      //if cameras are ended
      if(update_cameras_thread_finished_){
        break;
      }
      else{
        // std::cout << "FRONT END WAIT NEW FRAME " << std::endl;
        waitForNewFrame();
        // std::cout << "FRONT END, NEW FRAME ARRIVED " << std::endl;

      }
    }

    // // if still frame current is last camera, new frame is the end signal
    // if(frame_current_==int(camera_vector_->size())-1){
    //   break;
    // }

    if (!track_candidates){
      std::cout << "FRONT END WAIT FOR OPTIMIZATION " << std::endl;
      waitForOptimization();
      std::cout << "FRONT END OPTIMIZATION DONE " << std::endl;

    }

    double t_start=getTime();

    frame_current_++;

    int frame_delay = camera_vector_->size()-frame_current_-1;
    sharedCoutDebug("\nFRONT END for Frame "+std::to_string(frame_current_)+" ("+camera_vector_->at(frame_current_)->name_+") , frame delay: "+std::to_string(frame_delay));

    if(frame_current_<=1){


      tracker_->trackCam(true);
      keyframe_handler_->addFirstKeyframe();
      if(frame_current_==1){
        mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
        // mapper_->trackExistingCandidates(true,debug_mapping);
        cand_tracked_.notify_all(); // after candidates are tracked notify for ba
      }
      mapper_->selectNewCandidates();

      if (track_candidates)
        tracker_->collectCandidatesInCoarseRegions();

      continue;
    }


    tracker_->trackCam(take_gt_poses,track_candidates,guess_type,debug_tracking);

    if(keyframe_handler_->addKeyframe(all_keyframes)){

      mapper_->trackExistingCandidates(take_gt_points,debug_mapping);
      // mapper_->trackExistingCandidates(true,debug_mapping);
      cand_tracked_.notify_all(); // after candidates are tracked notify for ba
      mapper_->selectNewCandidates();
      if (track_candidates)
        tracker_->collectCandidatesInCoarseRegions();

    }
    double t_end=getTime();
    int deltaTime=(t_end-t_start);
    sharedCoutDebug("FRONT END part of frame: "+std::to_string(frame_current_)+", time: "+ std::to_string(deltaTime)+" ms");

  }

  frontend_thread_finished_=true;
  cand_tracked_.notify_all();

}

void Dtam::setOptimizationFlags( bool debug_optimization){
  bundle_adj_->debug_optimization_=debug_optimization;
}

void Dtam::doOptimization(bool active_all_candidates, bool debug_optimization){

  setOptimizationFlags(debug_optimization);

  while( true ){

    if(!frontend_thread_finished_){
      // std::cout << "OPTIMIZATION WAIT, NEW TRACK CANDS " << std::endl;
      waitForTrackedCandidates();
      // std::cout << "OPTIMIZATION STARTS " << std::endl;
    }
    else
      break;


    bundle_adj_->frame_current_ba=frame_current_;

    // project active points (and marginalize points 2 times outside the frustum)
    // take fixed point
    bool take_fixed_point = 1;
    bundle_adj_->projectActivePoints(take_fixed_point);

    // std::cout << "OPTIMIZATION 1 " << std::endl;

    // activate new points
    bundle_adj_->activateNewPoints();

    // std::cout << "OPTIMIZATION 2 " << std::endl;

    // collect coarse active points
    bundle_adj_->collectCoarseActivePoints();

    // std::cout << "OPTIMIZATION 3 " << std::endl;

    // DEBUG
    if(bundle_adj_->debug_optimization_){
      // cam on which active points are projected
      CameraForMapping* last_keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->back());
      last_keyframe->showProjActivePoints(1);
      // last_keyframe->showProjCandidates(1);


      // iterate along all cameras
      // for (int j=0; j<int(bundle_adj_->keyframe_vector_ba_->size())-1; j++){
      //     CameraForMapping* keyframe = camera_vector_->at(bundle_adj_->keyframe_vector_ba_->at(j));
      //     for (int i=keyframe->candidates_coarse_->size(); i>0; i--){
      //         // std::cout << "CAMERA " << keyframe->name_ << std::endl;
      //         keyframe->showCoarseActivePoints(i,1);}
      //     }
      //
        cv::waitKey(0);
        cv::destroyAllWindows();
      }

    // optimize
    bundle_adj_->optimize();

    // after optimization
    // std::cout << "OPTIMIZATION WAIT NEW FRAME " << std::endl;
    if(!update_cameras_thread_finished_ )
      waitForNewFrame();
    // std::cout << "OPTIMIZATION, NEW FRAME ARRIVED " << std::endl;


    // time guard
    std::this_thread::sleep_for(std::chrono::microseconds(1000));


    // notify all threads that optimization has been done
    optimization_done_.notify_all();

  }
}



void Dtam::updateCamerasFromEnvironment(){

  float fps=environment_->fps_;
  int counter=0;

  while(counter< environment_->camera_vector_->size()){

    // std::unique_lock<std::mutex> locker(mu_frame_);

    double t_start=getTime();

    // sharedCout("\nFrame: "+ std::to_string(counter));
    addCamera(counter);

    // locker.unlock();
    // std::cout << "CAM UPDATED" << std::endl;
    frame_updated_.notify_all();

    double t_end=getTime();

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
  update_cameras_thread_finished_=true;
  frame_updated_.notify_all();
  sharedCout("\nVideo stream ended");

}


void Dtam::eval_initializer(){

  // bool debug_initialization=true;
  // bool debug_mapping=false;
  // bool track_candidates=false;
  //
  // bool initialization_loop=true;
  //
  // std::thread initialization_thread_(&Dtam::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates );
  // std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);
  //
  // update_cameras_thread_.join();
  // initialization_thread_.join();
  //
  // // initializer_->showCornersRef();
  // initializer_->showCornersTrackSequence();
  // cv::waitKey(0);

}

void Dtam::test_mapping(){

  bool debug_mapping=true;
  bool debug_tracking=false;

  bool take_gt_poses=true;
  bool take_gt_points=false;
  bool track_candidates=true;

  int guess_type=VELOCITY_CONSTANT;

  bool all_keyframes=true;
  bool wait_for_initialization=false;
  // bool active_all_candidates=true;

  // std::thread optimization_thread(&Dtam::doOptimization, this, active_all_candidates);
  std::thread frontend_thread_(&Dtam::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);


  // optimization_thread.join();
  frontend_thread_.join();
  update_cameras_thread_.join();

  // debugAllCameras();


  // camera_vector_->at(0)->image_intensity_->show(2);
  // camera_vector_->at(0)->wavelet_dec_->vector_wavelets->at(2)->c->show(2);
  // camera_vector_->at(0)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(2);
  // camera_vector_->at(1)->wavelet_dec_->vector_wavelets->at(2)->magnitude_img->show(4,"1");
  // camera_vector_->at(0)->showCandidates_1(2);
  // camera_vector_->at(0)->showCandidates(2);
  // camera_vector_->at(camera_vector_->size()-2)->showProjCandidates(2);
  // camera_vector_->at(0)->showCandidates(2);
  // camera_vector_->at(0)->showCandidates(2);
  // camera_vector_->at(7)->showProjCandidates(2);
  // camera_vector_->at(1)->showProjCandidates(2);
  // camera_vector_->at(5)->showProjCandidates(2);

  // camera_vector_->at(keyframe_vector_->back())->regions_sampling_->region_vec_->at(1)->showRegion(2);

  // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));

  // testRotationalInvariance();
  cv::waitKey(0);

}


void Dtam::test_tracking(){

  bool debug_mapping=false;
  bool debug_tracking=true;

  // tracking flags
  bool take_gt_poses=false;
  bool take_gt_points=true;
  bool track_candidates=true;
  int guess_type=POSE_CONSTANT;

  bool all_keyframes=true;
  bool wait_for_initialization=false;

  std::thread frontend_thread_(&Dtam::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);

  // optimization_thread.join();
  frontend_thread_.join();
  update_cameras_thread_.join();


  cv::waitKey(0);

}

void Dtam::test_dso(){

  bool debug_initialization=false;
  bool debug_mapping=false;
  bool debug_tracking=false;
  bool debug_optimization= true;

  bool initialization_loop=false;
  bool take_gt_poses=false;
  bool take_gt_points=false;

  bool track_candidates=false;
  // int guess_type=POSE_CONSTANT;
  int guess_type=VELOCITY_CONSTANT;

  bool all_keyframes=true;
  bool wait_for_initialization=true;
  bool active_all_candidates=true;

  std::thread frontend_thread_(&Dtam::doFrontEndPart, this, all_keyframes, wait_for_initialization, take_gt_poses, take_gt_points, track_candidates, guess_type, debug_mapping, debug_tracking);
  std::thread initialization_thread_(&Dtam::doInitialization, this, initialization_loop, debug_initialization, debug_mapping, track_candidates, take_gt_points);
  std::thread update_cameras_thread_(&Dtam::updateCamerasFromEnvironment, this);

  if(!track_candidates){
    std::thread optimization_thread(&Dtam::doOptimization, this, active_all_candidates, debug_optimization);
    optimization_thread.join();
  }

  initialization_thread_.join();
  update_cameras_thread_.join();
  frontend_thread_.join();

  // makeJsonForCands("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));
  makeJsonForActivePts("./dataset/"+environment_->dataset_name_+"/state.json", camera_vector_->at(5));


}


// TODO remove
bool Dtam::makeJsonForCands(const std::string& path_name, CameraForMapping* camera){

  std::cout << "creating json (cands)" << std::endl;

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
    for(RegionWithProjCandidates* reg_proj : *(camera->regions_projected_cands_->region_vec_) ){


      for(int i=0; i<reg_proj->cands_proj_vec_->size(); i++){

        CandidateProjected* cand_proj = reg_proj->cands_proj_vec_->at(i);

        int level = cand_proj->level_;
        Eigen::Vector2f uv = cand_proj->uv_ ;
        Eigen::Vector3f p;
        camera->pointAtDepth( uv, 1.0/cand_proj->invdepth_, p);
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << count;
        std::string idx = ss.str();
        j["cameras"][camera->name_]["p"+idx] = {
          {"level", level},
          {"invdepth_var", cand_proj->cand_->invdepth_var_},
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


}


// TODO remove
bool Dtam::makeJsonForActivePts(const std::string& path_name, CameraForMapping* camera){

  std::cout << "creating json (active pts)" << std::endl;

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
    for(RegionWithProjActivePoints* reg_proj : *(camera->regions_projected_active_points_->region_vec_) ){


      for(int i=0; i<reg_proj->active_pts_proj_vec_->size(); i++){

        ActivePointProjected* active_pt_proj = reg_proj->active_pts_proj_vec_->at(i);

        int level = active_pt_proj->level_;
        Eigen::Vector2f uv = active_pt_proj->uv_ ;
        Eigen::Vector3f p;
        camera->pointAtDepth( uv, 1.0/active_pt_proj->invdepth_, p);
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << count;
        std::string idx = ss.str();
        j["cameras"][camera->name_]["p"+idx] = {
          {"level", level},
          {"invdepth_var", active_pt_proj->active_point_->invdepth_var_},
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


}
