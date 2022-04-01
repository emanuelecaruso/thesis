// #include "environment.h"
#include "dtam.h"
#include "parameters.h"
#include <stdio.h>
#include <cassert>

using namespace std;
using namespace pr;


int main (int argc, char * argv[]) {


  // read arguments
  const std::string dataset_name = argv[1]; // dataset name
  const std::string path_name = "./dataset/"+dataset_name;

  // initialization
  Params* parameters = new Params();

  Environment* environment = new Environment(path_name, dataset_name, parameters);

  Dtam* dtam = new Dtam(environment, parameters); // dense mapper and tracker

  //############################################################################
  // compute depth map
  //############################################################################

  // dtam->test_dso();
  dtam->test_dso_sequential();
  // dtam->test_optimization_pose();
  // dtam->test_optimization_points();
  // dtam->test_mapping();
  // dtam->test_tracking();

  // dtam->testFeatures();
  // cv::waitKey(0);

  cout << "\nPress Enter to exit"<< endl;
  cin.ignore();
  // // --------------------------------------
  return 1;
}
