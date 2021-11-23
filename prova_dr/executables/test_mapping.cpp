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
  int wavelet_levels=4;
  float gradient_threshold = 0.5;
  // int num_candidates = 0;
  int num_candidates = 2000;
  // int num_candidates = 10000;
  // int num_candidates = INT_MAX;
  int num_active_keyframes = 7;

  // initialization
  Environment* environment = new Environment(path_name,
                    dataset_name, wavelet_levels); // environment generator object (pointer)
  Dtam* dtam = new Dtam(environment, gradient_threshold,
            num_candidates,num_active_keyframes,wavelet_levels); // dense mapper and tracker

  //############################################################################
  // compute depth map
  //############################################################################

  dtam->test_mapping();

  // dtam->testFeatures();
  // cv::waitKey(0);


  // // --------------------------------------
  return 1;
}
