#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration


class BundleAdj{

  public:
    BundleAdj(Dtam* dtam):
    dtam_(dtam),
    frame_current_ba(0){};

    void projectAndMarginalizeActivePoints();
    void activateNewPoints(bool active_all_candidates);
    void optimize();
  private:

    int frame_current_ba;
    Dtam* const dtam_;
    void projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void sortRegions();
    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void selectNewActivePoints();
    void selectNewActivePointsAll();
    void activateNewPointsAllCand();
    void activateNewPoints();



};
