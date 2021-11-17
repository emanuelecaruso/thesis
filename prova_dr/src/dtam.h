#pragma once
#include "camera.h"
#include "mapper.h"
#include "tracker.h"
#include "environment.h"
#include <thread>
#include <mutex>
#include <condition_variable>


class Dtam{
  public:
    Dtam(Environment* environment) :
    environment_(environment),
    mapper_(new Mapper(this)),
    tracker_(new Tracker(this))
    {
      frame_current_=0;
      camera_vector_= new std::vector<CameraForMapping*>;
    };


    void test_mapping();
    void test_tracking(Environment* environment);
    void testFeatures();

    void debugAllCameras(bool show_imgs=false);
    int getFrameCurrent();


  private:
    const Environment* environment_;
    Mapper* const mapper_;
    Tracker* const tracker_;
    friend class Mapper;
    friend class Tracker;
    std::vector<CameraForMapping*>* camera_vector_;
    int frame_current_;

    std::mutex mu_frame_;
    std::condition_variable first_2_frames_available_;

    std::thread update_cameras_thread_;
    std::thread mapping_thread_;
    std::thread tracking_thread_;



    void addCamera(bool takeGtPoses);

    void updateCamerasFromVideostream(bool takeGtPoses);
    void doMapping();
    void doTracking();
    void showFeatures(int idx, float size);

};
