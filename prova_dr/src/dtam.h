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
  //
  //   double showImgs(int scale);
  //   void addCamera(Camera_cpu* camera_cpu);
  //   bool setReferenceCamera(int index_r);


    // void loadCameras(CameraVector camera_vector);

    void addCamera(bool takeGtPoses);
    bool getCurrentFrame();

    void updateCamerasFromVideostream(bool takeGtPoses);
    // bool setReferenceCamera(int index_r);
    // void prepareCameraForDtam(int index_m);
    //
    // void updateDepthMap( int index_m, bool check=false);
    // void updateDepthMap_parallel_cpu( int index_m, bool check=false);
};
