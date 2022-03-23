#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

class Dtam;
class CameraForMapping;

class KeyframeHandler{
  public:
    const Dtam* dtam_;
    const int num_active_keyframes_;

    KeyframeHandler(Dtam* dtam, int num_active_keyframes):
    dtam_(dtam),
    num_active_keyframes_(num_active_keyframes){};

    bool addKeyframe(bool all_keyframes=false);
    void addFirstKeyframe();
    bool marginalize_keyframe(bool all_keyframes=false);

  private:
    void pushKeyframeFrontend();
    void pushKeyframeBundleadj();
    bool pushKF();

    void marginalizeKeyframeFrontend(int idx);
    void marginalizeKeyframeBundleadj(int idx);
    void marginalizeKeyframe(int idx);

    bool addKeyframe_select();
    void marginalizeKeyframeSelect();

    float getFlowDist();
    float getPercentuageMarg(CameraForMapping* keyframe);
    float getScore(CameraForMapping* keyframe);

};
