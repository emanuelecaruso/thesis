#pragma once
#include "camera.h"
#include "mapper.h"
#include "tracker.h"
#include "environment.h"
#include <thread>


class Dtam{
  public:
    Dtam(Environment* environment) :
    environment_(environment),
    mapper_(new Mapper(this)),
    tracker_(new Tracker(this))
    {
      frame_current_=0;
      camera_vector_= new std::vector<Camera*>;
    };

    void test_mapping();
    void test_tracking(Environment* environment);
    void debugAllCameras(bool show_imgs=false);


  private:
    std::vector<Camera*>* camera_vector_;
    const Environment* environment_;
    const Mapper* mapper_;
    const Tracker* tracker_;
    int frame_current_;

    std::thread update_cameras_thread_;
    std::thread mapping_thread_;
    std::thread tracking_thread_;

    void addCamera(bool takeGtPoses);

    void updateCamerasFromVideostream(bool takeGtPoses);
    void doMapping();
    void doTracking();

};
