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
    num_active_points_(0),
    frame_current_ba(0){};

    void projectAndMarginalizeActivePoints();
    void activateNewPoints();
    void optimize();
  private:

    int frame_current_ba;
    Dtam* const dtam_;
    Params* const parameters_;
    int num_active_points_;

    void projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void activateCandidate(const Candidate* cand);

    void sortRegions();
    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void selectNewActivePoints();



};
