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

// coarsest level
static int coarsest_level_= 4; // e.g. level = 3 -> 0,1,2,*3* (fourth level)
// levels for regions
// static int wavelet_levels_=3; // e.g. 3 levels -> lev 0,1,2
static int wavelet_levels_=1; // e.g. 3 levels -> lev 0,1,2
static int reg_level_=2;     // e.g. level = 3 -> 0,1,2,*3* (fourth level)
static float grad_threshold_=0.1;
static float grad_perc_threshold_=0.75;
// static float cost_threshold_=0.7;
static float cost_threshold_=0.4;
static int num_candidates_=2000;
static int max_num_active_points_=num_candidates_;
static int num_active_keyframes_=5;
static int max_num_mins_ = 5;
static float max_invdepth_var_=0.05;
static int end_frame_=5;
static int fps_=30;
// tracker parameters
static int max_iterations_ls_=10;
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
  int wavelet_levels=wavelet_levels_;
  int reg_level=reg_level_;
  float grad_threshold=grad_threshold_;
  float grad_perc_threshold=grad_perc_threshold_;
  float cost_threshold=cost_threshold_;
  int num_candidates=num_candidates_;
  int max_num_active_points=max_num_active_points_;
  int num_active_keyframes=num_active_keyframes_;
  int max_num_mins=max_num_mins_;
  float max_invdepth_var=max_invdepth_var_;
  int end_frame=end_frame_;
  int fps=fps_;
  int max_iterations_ls=max_iterations_ls_;
  int n_corners=n_corners_;
  float quality_level=quality_level_;
  float min_distance=min_distance_;
  float err_threshold=err_threshold_;
  float size_window=size_window_;
  float confidence=confidence_;
  float ransacReprojThreshold=ransacReprojThreshold_;

  Params(){
    if (coarsest_level<wavelet_levels){
      std::cout << "Error in parameters: coarsest_level less than wavelet_levels!" << std::endl;
      exit(1);
    }
    if (wavelet_levels>reg_level+1){
      std::cout << "Error in parameters: reg_level+1 less than wavelet_levels!" << std::endl;
      exit(1);
    }
  };

};
