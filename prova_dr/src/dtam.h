#pragma once
#include "camera.h"
#include "parameters.h"
#include "keyframe.h"
#include "mapper.h"
#include "tracker.h"
#include "bundleadj.h"
#include "initializer.h"
#include "environment.h"
#include <thread>
#include <mutex>
#include <condition_variable>



class Dtam{
  public:
    Dtam(Environment* environment, Params* parameters):

    // Dtam(Environment* environment, float grad_threshold, float cost_threshold,
    //       int num_candidates, int num_active_keyframes, int wavelet_levels) :
    environment_(environment),
    keyframe_handler_(new KeyframeHandler(this, parameters->num_active_keyframes)),
    mapper_(new Mapper(this,parameters)),
    tracker_(new Tracker(this)),
    bundle_adj_(new BundleAdj(this, parameters)),
    initializer_(new Initializer(this, parameters)),
    parameters_(parameters),
    camera_vector_(new std::vector<CameraForMapping*>),
    keyframe_vector_(new std::vector<int>),
    frame_current_(-1),
    end_flag_(false)

    { };


    void test_mapping();
    void test_tracking();
    void eval_initializer();
    void test_dso();

    void debugAllCameras(bool show_imgs=false);
    void waitForNewFrame();
    void waitForTrackedCandidates();
    void waitForInitialization();

    CameraForMapping* getCurrentCamera();
    CameraForMapping* getPreviousCamera();


  private:
    const Environment* environment_;
    KeyframeHandler* const keyframe_handler_;
    Mapper* const mapper_;
    Tracker* const tracker_;
    BundleAdj* const bundle_adj_;
    Initializer* const initializer_;
    Params* const parameters_;
    std::vector<CameraForMapping*>* camera_vector_;
    std::vector<int>* keyframe_vector_;
    int frame_current_;
    bool end_flag_;

    friend class Initializer;
    friend class BundleAdj;
    friend class KeyframeHandler;
    friend class Mapper;
    friend class Tracker;

    std::mutex mu_frame_;
    std::mutex mu_candidate_tracking_;
    std::mutex mu_initialization_;
    std::condition_variable frame_updated_;
    std::condition_variable cand_tracked_;
    std::condition_variable initialization_done_;

    std::thread update_cameras_thread_;
    std::thread mapping_thread_;
    std::thread tracking_thread_;


    void addCamera(int counter);

    void updateCamerasFromEnvironment();
    void updateCamerasFromVideostream();
    void doMapping();
    void doInitialization(bool initialization_loop=false);
    void doFrontEndPart(bool all_keyframes=false, bool wait_for_initialization=true,  bool take_gt_poses=false, bool const_acc=true);
    void doOptimization(bool active_all_candidates=false);
    void doTracking();

    bool makeJsonForCands(const std::string& path_name, CameraForMapping* camera);

    void testRotationalInvariance();
};
