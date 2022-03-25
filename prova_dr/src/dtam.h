#pragma once
#include "camera.h"
#include "defs.h"
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

class PoseNormError;

class Dtam{
  public:
    Dtam(Environment* environment, Params* parameters):

    // Dtam(Environment* environment, float grad_threshold, float cost_threshold,
    //       int num_candidates, int num_active_keyframes, int wavelet_levels) :
    environment_(environment),
    keyframe_handler_(new KeyframeHandler(this)),
    mapper_(new Mapper(this,parameters)),
    tracker_(new Tracker(this)),
    bundle_adj_(new BundleAdj(this, parameters)),
    initializer_(new Initializer(this, parameters)),
    parameters_(parameters),
    camera_vector_(new std::vector<CameraForMapping*>),
    keyframe_vector_(new std::vector<int>),
    frame_current_(-1),
    update_cameras_thread_finished_(false),
    frontend_thread_finished_(false),
    cand_tracked_flag_(false),
    pts_activated_flag_(false),
    keyframe_added_flag_(false)
    { };

    void test_mapping();
    void test_tracking();
    void eval_initializer();
    void test_dso();
    void test_optimization_pose();
    void test_optimization_points();

    void debugAllCameras(bool show_imgs=false);
    void waitForNewFrame();
    void waitForTrackedCandidates();
    void waitForInitialization();
    void waitForPointActivation();
    void waitForKeyframeAdded();
    void waitForOptimization();

    int getLastKeyframeIdx();
    int getSecondLastKeyframeIdx();
    CameraForMapping* getCurrentCamera();
    CameraForMapping* getLastCamera();
    CameraForMapping* getSecondLastCamera();
    CameraForMapping* getLastKeyframe();

    PoseNormError* getTotalPosesNormError();
    float getTotalPointsNormError();

  private:
    const Environment* environment_;
    KeyframeHandler* keyframe_handler_;
    Mapper* mapper_;
    Tracker* tracker_;
    BundleAdj* bundle_adj_;
    Initializer* initializer_;
    Params* const parameters_;
    std::vector<CameraForMapping*>* camera_vector_;
    std::vector<int>* keyframe_vector_;
    int frame_current_;

    friend class Initializer;
    friend class BundleAdj;
    friend class KeyframeHandler;
    friend class Mapper;
    friend class Tracker;
    friend class Spectator;

    mutable std::mutex mu_frame_;
    mutable std::mutex mu_candidate_tracking_;
    mutable std::mutex mu_initialization_;
    mutable std::mutex mu_point_activation_;
    mutable std::mutex mu_optimization_;
    mutable std::mutex mu_restart_opt_;
    mutable std::mutex mu_keyframe_added_;

    std::condition_variable frame_updated_;
    std::condition_variable initialization_done_;
    std::condition_variable points_activated_;
    std::condition_variable optimization_done_;

    std::condition_variable cand_tracked_;
    std::condition_variable pts_activated_;
    std::condition_variable keyframe_added_;


    std::thread update_cameras_thread_;
    std::thread frontend_thread_;
    std::thread initialization_thread_;
    std::thread optimization_thread;
    bool update_cameras_thread_finished_;
    bool frontend_thread_finished_;

    bool cand_tracked_flag_;
    bool pts_activated_flag_;
    bool keyframe_added_flag_;



    void setOptimizationFlags( bool debug_optimization, int opt_norm, int test_single, int image_id, bool test_marginalization);


    void addCamera(int counter);

    void updateCamerasFromEnvironment();
    void updateCamerasFromVideostream();
    void doMapping();
    void doInitialization(bool initialization_loop=false, bool debug_initialization=true, bool debug_mapping=false, bool track_candidates=false, bool take_gt_points=false);
    void doFrontEndPart(bool all_keyframes=false, bool wait_for_initialization=true,  bool take_gt_poses=false, bool take_gt_points=false, bool track_candidates=false, int guess_type=VELOCITY_CONSTANT, bool debug_mapping=false, bool debug_tracking=false);
    void doOptimization(bool active_all_candidates=false, bool debug_optimization=false, int opt_norm=HUBER, int test_single=TEST_ALL, int image_id=INTENSITY_ID, bool test_marginalization=false);

    void noiseToPoses(float var_angle, float var_position);
    Eigen::VectorXf* noiseToPosesSame(float var_angle, float var_position);
    void noiseToPoints(float var_invdepth);

    bool makeJsonForCands(const std::string& path_name, CameraForMapping* camera);
    bool makeJsonForActivePts(const std::string& path_name, CameraForMapping* camera);
    bool makeJsonForCameras(const std::string& path_name);

};
