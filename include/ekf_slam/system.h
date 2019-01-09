#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "ekf_slam/common.h"
#include "ekf_slam/laser_frame.h"
#include "ekf_slam/line_detector.h"
#include "ekf_slam/line_segment.h"
#include "ekf_slam/map.h"
#include "ekf_slam/ekf.h"

namespace ekf_slam{
class System{
public:
  using Ptr = shared_ptr<System>;
  System(){};
  System(LineDetector::Ptr line_detector, EKFilter::Ptr ekfilter);
  enum class State{INITIALIZING,RUNNING,LOST};
  State state_;
  
  //init function:
  //1. set initial pose and initial joint value
  //2. load initial frame and extract lines as initial map
  //3. get covariance of initial state
  void initSLAM(const Vector3d& T_init, LaserFrame::Ptr frame_init, const Matrix3d& P_T_init, const Vector2d& joint_value_init);
  //one system step 
  //1. extended kalman filter step 
  //2. update map 
  void oneSLAMStep(LaserFrame::Ptr frame_new, const Vector2d& joint_value_new, const double& wheel_radius);
  void oneStateTransitionStep(const Vector2d& joint_value_new, const double& wheel_radius);
  const Vector3d& pose() const;
private:
  //Line detector 
  LineDetector::Ptr line_detector_;
  //extended kalman filter 
  EKFilter::Ptr ekfilter_;
  //posterior estimation of laser pose
  Vector3d T_prev_;
  //robot wheel joint value 
  Vector2d joint_value_prev_;
  //laser frame, contains all the laser reading and pose 
  LaserFrame::Ptr frame_prev_;
  //posterior estimation of covariance matrix of state (pose and map) 
  MatrixXd P_prev_;
  //map
  Map::Ptr map_;
};
}
#endif