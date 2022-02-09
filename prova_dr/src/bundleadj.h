#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration


class JacobiansBA{
  public:
    Eigen::Matrix<float,1,6>* J_r;
    Eigen::Matrix<float,1,6>* J_m;
    float J_d;
    Eigen::Matrix<float,6,1>* J_r_transp;
    Eigen::Matrix<float,6,1>* J_m_transp;

    const int J_r_block_idx = -1;
    const int J_m_block_idx = -1;
    const int J_d_block_idx = -1;

    JacobiansBA(Eigen::Matrix<float,1,6>* J_r_, Eigen::Matrix<float,1,6>* J_m_, float J_d_,
                int J_r_block_idx_, int J_m_block_idx_, int J_d_block_idx_ ):
    J_r(J_r_),
    J_m(J_m_),
    J_r_transp(new Eigen::Matrix<float,6,1>),
    J_m_transp(new Eigen::Matrix<float,6,1>),
    J_d(J_d_),
    J_r_block_idx(J_r_block_idx_),
    J_m_block_idx(J_m_block_idx_),
    J_d_block_idx(J_d_block_idx_)
    {
      *J_r_transp=J_r_->transpose();
      *J_m_transp=J_m_->transpose();
    }

};


class HessianAndB{
  public:

    HessianAndB(int pose_block_size, int point_block_size ):

    // initialize H blocks
    H_pose_pose(new Eigen::MatrixXf(pose_block_size,point_block_size) ),
    H_pose_point(new Eigen::MatrixXf(point_block_size,point_block_size)),
    H_point_pose(new Eigen::MatrixXf(pose_block_size,point_block_size)),
    H_point_point(new Eigen::MatrixXf(point_block_size,point_block_size)),

    // initialize b blocks
    b_pose(new Eigen::MatrixXf(pose_block_size,1)),
    b_point(new Eigen::MatrixXf(point_block_size,1))
    {
      H_pose_pose->setZero();
      H_pose_point->setZero();
      H_point_pose->setZero();
      H_point_point->setZero();
      b_pose->setZero();
      b_point->setZero();
    }

    void updateHessianAndB(JacobiansBA* jacobians, float error);

  private:
    Eigen::MatrixXf* H_pose_pose;
    Eigen::MatrixXf* H_pose_point;
    Eigen::MatrixXf* H_point_pose;
    Eigen::MatrixXf* H_point_point;

    Eigen::MatrixXf* b_pose;
    Eigen::MatrixXf* b_point;

};

class BundleAdj{

  public:
    BundleAdj(Dtam* dtam, Params* parameters):
    debug_optimization_(false),
    dtam_(dtam),
    parameters_(parameters),
    keyframe_vector_ba_(new std::vector<int>),
    frame_current_ba(-1),
    num_active_points_(0),
    min_num_of_active_pts_per_region_(INT_MAX)
    {};


    void projectActivePoints();
    void updateCurrentGuess();
    void updateProjActivePoints();
    ActivePointProjected* projectActivePoint(ActivePoint* active_pt, CamCouple* cam_couple);
    void marginalizeActivePoints();
    void activateNewPointsAndGetCoarseActivePoints();
    void collectCoarseActivePoints();

    JacobiansBA* getJacobians(ActivePointProjected* active_pt_proj);

    void optimize(int pose_block_size, int point_block_size);
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
    int min_num_of_active_pts_per_region_;


    ActivePoint* activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs);
    void addCoarseActivePointInRegion(ActivePoint* active_pt);

    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);
    int selectNewActivePoints();

    void marginalize();
    void updateStateBlockIdxs(int& pose_block_size, int& point_block_size);

    Eigen::Matrix<float,1,3>* getJfirst(ActivePointProjected* active_pt_proj, Eigen::Vector3f& point_m);
    Eigen::Matrix<float,1,6>* getJr(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first);
    Eigen::Matrix<float,1,6>* getJm(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first, Eigen::Vector3f& point_m);
    float getJd(ActivePointProjected* active_pt_proj, Eigen::Matrix<float,1,3>* J_first);
    float getError(ActivePointProjected* active_pt_proj);

};
