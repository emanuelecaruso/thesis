#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <thread>
#include <chrono>
#include <stdlib.h>

void Dtam::debugAllCameras(bool show_imgs){

  std::cout << "DEBUGGING ALL CAMERAS:" << std::endl;
  std::cout << "camera vector size: " << camera_vector_->size() << std::endl;

  for(Camera* camera : *camera_vector_){
    camera->printMembers();
    if (show_imgs){
      camera->showRGB();
      camera->showDepthMap();
    }
  }
  cv::waitKey(0);
}

void Dtam::addCamera(bool takeGtPoses){
  Camera* new_camera=environment_->camera_vector_->at(frame_current_)->clone();
  if(!takeGtPoses){
    new_camera->frame_camera_wrt_world_->linear().setIdentity();
    new_camera->frame_camera_wrt_world_->translation()= Eigen::Vector3f(0,0,0);
    new_camera->frame_world_wrt_camera_->linear().setIdentity();
    new_camera->frame_world_wrt_camera_->translation()= Eigen::Vector3f(0,0,0);
  }
  camera_vector_->push_back(new_camera);
}

bool Dtam::getCurrentFrame(){

  // int fps=environment_->fps_;
  // int frame_new = int((getTime()-t_start_-waitDelay_)/((1.0/fps)*1000));
  //
  //   int frames_delta=frame_new-frame_current_;
  //   if (frames_delta>1)
  //     std::cout << frames_delta << " frames has been skipped!" << std::endl;
  //
  // if (frame_new>=frame_current_)
  //   if(frame_new>=environment_->camera_vector_->size()){
  //     return false;
  //   frame_current_=frame_new;
  //   std::cout << "\nFrame n: " << frame_new << std::endl;
  //   return true;
  // }
  // else
  //   return false;

}


void Dtam::updateCamerasFromVideostream(bool takeGtPoses){

  float fps=environment_->fps_;

  while(true){

    double t_start=getTime();

    if (frame_current_>= environment_->camera_vector_->size()){
      std::cout << "Video stream ended" << std::endl;
      break;
    }
    std::cout << "Frame: " << frame_current_ << std::endl;
    addCamera(takeGtPoses);
    frame_current_++;

    double t_end=getTime();

    long int waitDelay=(t_end-t_start)*1000;

    long int time_to_wait=(1.0/fps)*1000000-waitDelay;
    std::this_thread::sleep_for(std::chrono::microseconds(time_to_wait));
  }
}

void Dtam::test_mapping(){

  bool frame_available=true;
  bool set_reference=true;
  bool init=true;
  int it=0;
  bool takeGtPoses=true;

  std::thread update_cameras_thread_(&Dtam::updateCamerasFromVideostream, this, takeGtPoses);
  update_cameras_thread_.join();

  // debugAllCameras(true);
  // t_start_=getTime();

  // while (true){
  //
  //   bool takeGtPoses=true;
  //   updateWithNewCamera(takeGtPoses);
    // camera_cpu->setGroundtruthPose();

  //   int current_frame=int((getTime()-t_start-waitKeyDelay)/((1.0/fps)*1000));
  //   int frames_delta=current_frame-frames_computed;
  //   if(frames_delta>=0){
  //     frame_available=true;
  //     if(frames_computed>=environment->camera_vector_cpu_.size()){
  //       break;
  //     }
  //     Camera_cpu* camera_cpu=environment->camera_vector_cpu_[frames_computed];
  //     Camera_gpu* camera_gpu=camera_cpu->camera_gpu_;
  //     // set groundtruth pose
  //     camera_cpu->setGroundtruthPose();
  //     // load camera
  //     addCamera(camera_cpu);
  //     frames_computed+=(frames_delta+1);
  //     if (frames_delta>0)
  //       std::cout << frames_delta+1 << " frames has been skipped!" << std::endl;
  //
  //     std::cout << "\nFrame n: " << frames_computed-1 << std::endl;
  //
  //   }
  //
  //   if (frame_available){
  //
  //     if (set_reference){
  //       setReferenceCamera(frames_computed-1);
  //       set_reference=false;
  //     }
  //     else{
  //       int index_m=frames_computed-1;
  //       // printf("%i\n", index_m);
  //
  //       mapper_->UpdateCostVolume(index_m,index_m<2 );
  //
  //       if(index_m==(index_r_+1)){
  //         mapper_->ComputeCostVolumeMin();
  //
  //         init = false;
  //       }
  //       else if(index_m>(index_r_+2)){
  //
  //         mapper_->UpdateDepthmap();
  //         mapper_->PopulateState();
  //
  //       }
  //
  //     }
  //
  //
  //     frame_available=false;
  //   }
  //
  //   else if(!init){
  //
  //     // float cr=0.484; float rr=0.465;  //occlusion
  //     float cr=0.51; float rr=0.98;  //strange down
  //     // float cr=0.61; float rr=0.53;  //hightex1
  //     // float cr=0.61; float rr=0.53;  //hightex2
  //     // float cr=0.95; float rr=0.87;  //corner dr
  //     // float cr=0.95; float rr=0.08;  //corner ur
  //     // float cr=0.5; float rr=0.9;  //hightex cube
  //
  //     int index_m=frames_computed-1;
  //     int col=cr*camera_vector_cpu_[0]->resolution_;
  //     int row=rr*camera_vector_cpu_[0]->resolution_/camera_vector_cpu_[0]->aspect_;
  //     // waitKeyDelay+=mapper_->StudyCostVolumeMin(index_m, row, col, true);
  //
  //
  //     if (!mapper_->Regularize())
  //       init=true;
  //
  //     waitKeyDelay+=showImgs(640);
  //
  //   }
  //
  //
  // }
  // waitKeyDelay+=environment->saveState("./dataset/"+environment->dataset_name_+"/state.json", environment->camera_vector_cpu_[index_r_]);


}
