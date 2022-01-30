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
    num_points_to_marginalize_(0),
    frame_current_ba(-1){};

    void projectAndMarginalizeActivePoints();
    void activateNewPoints();
    void optimize();
    inline int getFrameCurrentBA(){
      return frame_current_ba;
    }
  private:

    int frame_current_ba;
    Dtam* const dtam_;
    Params* const parameters_;
    int num_active_points_;
    int num_points_to_marginalize_;

    void projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    void activateCandidate(CandidateProjected* cand_proj);

    void sortRegions();
    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    int selectNewActivePoints();



};
