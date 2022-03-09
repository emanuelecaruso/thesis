#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <unistd.h>
#include "opencv2/opencv.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <mutex>
#include "defs.h"

// candidate selection
static int coarsest_level_= 3; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
static int reg_level_=2;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
static float grad_threshold_=0.05;
static int num_candidates_=2000;

// mapping
static float cost_threshold_=0.07;
static float cost_grad_threshold_=0.1;
static float cost_grad_threshold_DSO_=0.01; // for DSO
static int max_num_mins_ = 3;

// optimization
static int max_num_active_points_=2*num_candidates_;
static int num_active_keyframes_=6;
static float huber_threshold_=0.02;
static float chi_occlusion_threshold_=(0.1-huber_threshold_/2);
static float intensity_coeff_ = 1;
static float gradient_coeff_ = 1;
static float phase_coeff_ = 1;

// tracking
static int max_iterations_ls_=20;
static float variance_ = 0.1;
static int robustifier_dofs_=2;
static float ratio_for_convergence_ = 0.001;

//  video streaming
static int end_frame_=5;
static int fps_=30;

// initializer parameters
static int n_corners_ = 1000;
static float quality_level_ = 0.01;
static float min_distance_ = 10;
static float err_threshold_ = 5;
static int size_window_ = 21;
static float confidence_ = 0.999;
static float ransacReprojThreshold_ = 1;

struct Params{

  int coarsest_level=coarsest_level_;
  int reg_level=reg_level_;
  float grad_threshold=grad_threshold_;
  int num_candidates=num_candidates_;

  float cost_threshold=cost_threshold_;
  float cost_grad_threshold=cost_grad_threshold_;
  float cost_grad_threshold_DSO=cost_grad_threshold_DSO_;
  int max_num_active_points=max_num_active_points_;
  int num_active_keyframes=num_active_keyframes_;
  int max_num_mins=max_num_mins_;
  int end_frame=end_frame_;
  int fps=fps_;
  float huber_threshold=huber_threshold_;
  float chi_occlusion_threshold=chi_occlusion_threshold_;

  float intensity_coeff=intensity_coeff_;
  float gradient_coeff=gradient_coeff_;
  float phase_coeff=phase_coeff_;

  int max_iterations_ls=max_iterations_ls_;
  float variance=variance_;
  int robustifier_dofs=robustifier_dofs_;
  int ratio_for_convergence=ratio_for_convergence_;
  int n_corners=n_corners_;
  float quality_level=quality_level_;
  float min_distance=min_distance_;
  float err_threshold=err_threshold_;
  float size_window=size_window_;
  float confidence=confidence_;
  float ransacReprojThreshold=ransacReprojThreshold_;

  Params(){
    // if (coarsest_level<reg_level){
    //   std::cout << "Error in parameters: coarsest level less than reg_level!" << std::endl;
    //   exit(1);
    // }
  };


};
