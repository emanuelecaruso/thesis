#include "dtam_cuda.cuh"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include "cuda_utils.cuh"



void Dtam::addCamera(Camera_cpu* camera_cpu){
  camera_vector_cpu_.push_back(camera_cpu);
  mapper_->camera_vector_cpu_.push_back(camera_cpu);
  tracker_->camera_vector_cpu_.push_back(camera_cpu);
}


bool Dtam::setReferenceCamera(int index_r){

  int num_cameras = camera_vector_cpu_.size();

  if (index_r<0 || index_r>=num_cameras)
    return false;

  index_r_ = index_r;
  mapper_->index_r_ = index_r;
  tracker_->index_r_ = index_r;


  return true;

}

double Dtam::showImgs(int scale){

  double t_s=getTime();

  int resolution=camera_vector_cpu_[index_r_]->resolution_;

  cv::Mat_< float > gt;
  mapper_->depth_groundtruth_.download(gt);
  cv::Mat_< float > resized_image_gt;
  cv::resize(gt, resized_image_gt, cv::Size(), scale/resolution, scale/resolution, cv::INTER_NEAREST );
  cv::imshow("comparison", resized_image_gt);


  cv::Mat_< float > state;
  (camera_vector_cpu_[index_r_]->invdepth_map_gpu_).download(state);
  cv::Mat_< float > resized_image_state;
  cv::resize(state, resized_image_state, cv::Size(), scale/resolution, scale/resolution, cv::INTER_NEAREST );
  cv::imshow("state", resized_image_state);

  cv::waitKey(0);

  double t_e=getTime();
  double delta=t_e-t_s;
  return delta;
}


void Dtam::test_mapping(Environment_gpu* environment){

  double t_start;  // time start for computing computation time
  double waitKeyDelay=0;

  bool frame_available=true;
  bool set_reference=true;
  bool init=true;
  int it=0;
  int& frames_computed = mapper_->frames_computed_;

  mapper_->depthSamplingInit(environment);

  t_start=getTime();
  while (true){

    // considering 30 fps camera
    float fps=3;
    int current_frame=int((getTime()-t_start-waitKeyDelay)/((1.0/fps)*1000));
    int frames_delta=current_frame-frames_computed;
    if(frames_delta>=0){
      frame_available=true;
      if(frames_computed>=environment->camera_vector_cpu_.size()){
        break;
      }
      Camera_cpu* camera_cpu=environment->camera_vector_cpu_[frames_computed];
      Camera_gpu* camera_gpu=camera_cpu->camera_gpu_;
      // set groundtruth pose
      camera_cpu->setGroundtruthPose();
      // load camera
      addCamera(camera_cpu);
      frames_computed+=(frames_delta+1);
      if (frames_delta>0)
        std::cout << frames_delta+1 << " frames has been skipped!" << std::endl;

      std::cout << "\nFrame n: " << frames_computed-1 << std::endl;

    }

    if (frame_available){

      if (set_reference){
        setReferenceCamera(frames_computed-1);
        set_reference=false;
      }
      else{
        int index_m=frames_computed-1;

        // mapper_->UpdateCostVolume(index_m,index_m<2 );
        //
        // mapper_->ComputeCostVolumeMin();


        // waitKeyDelay+=showImgs(640);

      }


      frame_available=false;
    }


  }
  // waitKeyDelay+=environment->saveState("./dataset/"+environment->dataset_name_+"/state.json", environment->camera_vector_cpu_[index_r_]);


}


void Dtam::test_tracking(Environment_gpu* environment){

  // double t_start;  // time start for computing computation time
  // double waitKeyDelay=0;
  //
  // bool frame_available=true;
  // bool set_reference=true;
  // bool init=true;
  // int it=0;
  // int frames_computed=0;
  //
  // t_start=getTime();
  // while (true){
  //
  //   // considering 30 fps camera
  //   float fps=3;
  //   int current_frame=int((getTime()-t_start-waitKeyDelay)/((1.0/fps)*1000));
  //   int frames_delta=current_frame-frames_computed;
  //   if(frames_delta>=0){
  //     frame_available=true;
  //     if(frames_computed>=environment->camera_vector_cpu_.size()){
  //       break;
  //     }
  //     Camera_cpu* camera_cpu=environment->camera_vector_cpu_[frames_computed];
  //     Camera_gpu* camera_gpu=camera_cpu->camera_gpu_;
  //     // load camera
  //     addCamera(camera_cpu);
  //
  //
  //     frames_computed+=(frames_delta+1);
  //     if (frames_delta>0)
  //       std::cout << frames_delta+1 << " frames has been skipped!" << std::endl;
  //     std::cout << "\nFrame n: " << frames_computed-1 << std::endl;
  //   }
  //
  //   if (frame_available){
  //
  //     if (set_reference){
  //       setReferenceCamera(frames_computed-1);
  //
  //       Camera_cpu* camera_cpu=environment->camera_vector_cpu_[index_r_];
  //       if (index_r_==0)
  //         camera_cpu->setGroundtruthPose();
  //       // camera_cpu->printMembers();  //for debug
  //
  //       mapper_->StateFromGt();
  //       mapper_->PopulateState();
  //       waitKeyDelay+=environment->saveState("./dataset/"+environment->dataset_name_+"/state.json", environment->camera_vector_cpu_[index_r_]);
  //       set_reference=false;
  //
  //     }
  //     else{
  //       int index_m=frames_computed-1;
  //       // tracker_->printPoseComparison(index_m);
  //       init=false;
  //     }
  //
  //
  //
  //     frame_available=false;
  //   }
  //   else if(!init){
  //     // waitKeyDelay+=showImgs(640);
  //     init=true;
  //   }
  //
  //
  // }

}
