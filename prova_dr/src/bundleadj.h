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
    keyframe_vector_ba_(new std::vector<int>),
    frame_current_ba(-1),
    num_active_points_(0),
    num_points_to_marginalize_(0),
    min_num_of_active_pts_per_region_(INT_MAX)
    {};


    void projectActivePoints();
    void marginalizeActivePoints();
    void activateNewPointsAndGetCoarseActivePoints();
    void collectCoarseActivePoints();


    void optimize();

    inline void addKeyframe(int idx){
      keyframe_vector_ba_->push_back(idx);
    }

    inline int getFrameCurrentIdxBA(){
      return frame_current_ba;
    }

    CameraForMapping* getFrameCurrentBA();
    bool debug_optimization_;
    std::vector<int>* keyframe_vector_ba_;
    int frame_current_ba;

  private:

    Dtam* const dtam_;
    Params* const parameters_;
    int num_active_points_;
    int num_points_to_marginalize_;
    int min_num_of_active_pts_per_region_;


    ActivePoint* activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs);
    void addCoarseActivePointInRegion(ActivePoint* active_pt);

    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    int selectNewActivePoints();



};
