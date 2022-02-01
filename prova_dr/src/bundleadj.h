#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration


class BundleAdj{

  public:
    BundleAdj(Dtam* dtam, Params* parameters):
    debug_optimization_(false),
    dtam_(dtam),
    parameters_(parameters),
    frame_current_ba(-1),
    num_active_points_(0),
    num_points_to_marginalize_(0),
    min_num_of_active_pts_per_region_(INT_MAX)
    {};


    void projectAndMarginalizeActivePoints();
    void activateNewPoints();
    void optimize();


    inline int getFrameCurrentIdxBA(){
      return frame_current_ba;
    }

    CameraForMapping* getFrameCurrentBA();
    bool debug_optimization_;

  private:

    Dtam* const dtam_;
    Params* const parameters_;
    int frame_current_ba;
    int num_active_points_;
    int num_points_to_marginalize_;
    int min_num_of_active_pts_per_region_;

    void projectActivePoints(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    bool activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs);

    void sortRegions();
    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    int selectNewActivePoints();



};
