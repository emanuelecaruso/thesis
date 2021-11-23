#pragma once
#include "camera.h"
#include "keyframe.h"
#include "mapper.h"
#include "tracker.h"
#include "environment.h"
#include <thread>
#include <mutex>
#include <condition_variable>



class Dtam{
  public:
    Dtam(Environment* environment, float grad_threshold,
          int num_candidates, int num_active_keyframes, int wavelet_levels) :
    environment_(environment),
    keyframe_handler_(new KeyframeHandler(this, num_active_keyframes)),
    mapper_(new Mapper(this,grad_threshold,num_candidates)),
    tracker_(new Tracker(this)),
    wavelet_levels_(wavelet_levels)
    {
      frame_current_=0;
      camera_vector_= new std::vector<CameraForMapping*>;
      keyframe_vector_= new std::vector<int>;
    };


    void test_mapping();
    void test_tracking(Environment* environment);
    void testFeatures();

    void debugAllCameras(bool show_imgs=false);
    int getFrameCurrent();
    void waitForNewFrame();


  private:
    const Environment* environment_;
    KeyframeHandler* const keyframe_handler_;
    Mapper* const mapper_;
    Tracker* const tracker_;
    friend class KeyframeHandler;
    friend class Mapper;
    friend class Tracker;

    const int wavelet_levels_;
    std::vector<CameraForMapping*>* camera_vector_;
    std::vector<int>* keyframe_vector_;
    int frame_current_;

    std::mutex mu_frame_;
    std::condition_variable frame_updated_;

    std::thread update_cameras_thread_;
    std::thread mapping_thread_;
    std::thread tracking_thread_;


    void addCamera();

    void updateCamerasFromVideostream();
    void doMapping();
    void doInitialization(bool all_keyframes=false, bool takeGtPoses=false);
    void doTracking();
    void showFeatures(int idx, float size);

};
