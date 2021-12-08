#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration


class BundleAdj{

  public:
    BundleAdj(Dtam* dtam, Params* parameters):
    dtam_(dtam),
    parameters_(parameters),
    active_points_vec_(new std::vector<KFPoint*>),
    frame_current_ba(0){};

    void projectAndMarginalizeActivePoints();
    void activateNewPoints();
    void optimize();
  private:

    int frame_current_ba;
    Dtam* const dtam_;
    Params* const parameters_;
    std::vector<KFPoint*>* active_points_vec_;

    void projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe);

    void sortRegions();
    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void selectNewActivePoints();



};
