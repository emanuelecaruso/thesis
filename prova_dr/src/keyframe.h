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

    bool addKeyframe(bool all_keyframes=false);
    bool marginalize_keyframe(bool all_keyframes=false);

  private:
    void pushKeyframeFrontend();
    void pushKeyframeBundleadj();
    bool addKeyframe_all();

    void marginalizeKeyframeFrontend(int idx);
    void marginalizeKeyframeBundleadj(int idx);
    void marginalizeKeyframeAll();

    bool addKeyframe_select();
    void marginalizeKeyframeSelect();

};
