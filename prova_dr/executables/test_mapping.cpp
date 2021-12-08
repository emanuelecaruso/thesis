#include "environment.h"
#include "dtam.h"
#include <stdio.h>

using namespace std;
using namespace pr;


int main (int argc, char * argv[]) {


  // read arguments
  const std::string dataset_name = argv[1]; // dataset name
  const std::string path_name = "./dataset/"+dataset_name;

  // parameters
  int wavelet_levels=3; // ex: 3 levels -> lev 0,1,2
  int reg_level=2;  // ex reg level = 3 -> 0,1,2,*3* (fourth level)
  float grad_threshold = 0.7;
  // float grad_threshold = 1.5;
  float grad_perc_threshold = 0.8;
  float cost_threshold = 0.2;
  int num_candidates = 4000;
  int num_active_points = num_candidates;
  // int num_candidates = INT_MAX;
  int num_active_keyframes = 7;
  float max_depth_var = 0.05; // maximum depth variance for the initilizer


  // initialization
  Params* parameters = new Params(wavelet_levels ,reg_level, grad_threshold, grad_perc_threshold,
                                  cost_threshold, num_candidates, num_active_points,
                                  num_active_keyframes, max_depth_var);

  Environment* environment = new Environment(path_name, dataset_name, wavelet_levels);

  Dtam* dtam = new Dtam(environment, parameters); // dense mapper and tracker

  //############################################################################
  // compute depth map
  //############################################################################

  dtam->test_mapping();

  // dtam->testFeatures();
  // cv::waitKey(0);

  cout << "\nPress Enter to exit"<< endl;
  cin.ignore();
  // // --------------------------------------
  return 1;
}
