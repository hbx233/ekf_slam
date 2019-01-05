#ifndef EKF_H_
#define EKF_H_
#include "ekf_slam/common.h"
#include "ekf_slam/line_segment.h"
#include "ekf_slam/line_detector.h"
#include "ekf_slam/laser_frame.h"
namespace ekf_slam{
class EKFilter{
public:
  using Ptr = shared_ptr<EKFilter>;

public:
  //set filter parameter 
  EKFilter(double k, double b, double g);

/**
 * State Transition Part
 */
public:
  //do state transition to compute the prior estimation of 
  //control input u[0]: left wheel, u[1]: right wheel
  void stateTransition(const Vector3d& x_prev, const Matrix3d P_prev, const Vector2d& u);
private:
  //prior estimation of state
  Vector3d x_prior;
  Matrix3d P_prior;
  Matrix3d F_x; 
  Matrix32d F_u;
  //control input noise covariance parameter 
  double k_;
  //inter-wheel distance 
  double b_;
/**
 *Measurement Prediction Part
 */
public: 
  //predict the measurements given landmarks and prior state
  //number of predicted measurements equals to number of landmarks provided 
  void predictMeasurementsFromPriorState(const vector<LineSegment>& landmarks);
  //associate the observation with predicted measurements
  //gate the distance between each observation and prediction
  //if one observation's distance with all prediction fall outside the gate,
  //this observed measurement is considered as false measurement and will be abandoned 
  void associateObservationWithPrediction(const vector<LineSegment>& z, const vector<Matrix2d>& R);
private:
  vector<LineSegment> z_predict;
  vector<Matrix23d> H;
  vector<pair<int,int>> matches;
  //association gate value 
  double g_;
/**
 *Posterior Estimation
 */
public:
  void compPosteriorEstimation(const vector<LineSegment>& z, const vector<Matrix2d>& R, Vector3d& x, Matrix3d& P);
/**
 * EKF
 */
public:
  void ekfOneStep(const Vector3d& x_prev, const Matrix3d& P_prev,const Vector2d& u, const vector<LineSegment>& z, const vector<Matrix2d>& R, const vector<LineSegment>& landmarks, Vector3d& x_posterior, Matrix3d& P_posterior);
private: 
  long ekf_step_;//record how many steps have been processed in EkfFilter

};
}
#endif