#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

class Dtam;

class KeyframeHandler{
  public:
    const Dtam* dtam_;
    const int num_active_keyframes_;

    KeyframeHandler(Dtam* dtam, int num_active_keyframes):
    dtam_(dtam),
    num_active_keyframes_(num_active_keyframes){};

    bool add_keyframe(bool all_keyframes=false);
    bool marginalize_keyframe(bool all_keyframes=false);

  private:
    void push_keyframe();

    bool add_keyframe_all();
    void marginalize_keyframe_all();

    bool add_keyframe_select();
    void marginalize_keyframe_select();

    void notify_if_2_frames_are_available();
};
