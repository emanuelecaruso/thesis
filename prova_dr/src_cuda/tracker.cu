#pragma once
#include "tracker.cuh"

void Tracker::printPoseComparison(int index_m){
  Camera_cpu* camera_cpu= camera_vector_cpu_[index_m];
  std::cout << "frame_world_wrt_camera LINEAR:\n" << (*(camera_cpu->frame_world_wrt_camera_)).linear() << std::endl;
  std::cout << "frame_world_wrt_camera TRANSL:\n" << (*(camera_cpu->frame_world_wrt_camera_)).translation() << std::endl;
  std::cout << "frame_world_wrt_camera_gt LINEAR:\n" << (*(camera_cpu->frame_world_wrt_camera_gt_)).linear() << std::endl;
  std::cout << "frame_world_wrt_camera_gt TRANSL:\n" << (*(camera_cpu->frame_world_wrt_camera_gt_)).translation() << std::endl;
}

void computeCameraPose(int index_m){
  
}
