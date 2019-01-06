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
  //construct the state vector using robot state and map
  //take seperate robot state and map to construct state vector, better for future build map function
  //seperate the internal vectorized filter function with the outter implementation 
  void constructStateVector(const Vector3d& T_prev, const vector<LineSegment>& map_prev, const MatrixXd& P_prev);
public:
  //previous state vector need to be constructed from robot state and map 
  VectorXd x_prev;
  //initialize all vector and matrix
  void initVectorAndMatrix(const int& map_size);
  //do state transition to compute the prior estimation of 
  //control input u[0]: left wheel, u[1]: right wheel
  void stateTransition(const MatrixXd& P_prev, const Vector2d& u);
public:
  //prior estimation of the state
  VectorXd x_prior;
  MatrixXd P_prior;
  MatrixXd F_x; 
  MatrixXd F_u;
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
  void predictMeasurementsFromPriorState(const vector<LineSegment>& map_prev);
  //associate the observation with predicted measurements
  //gate the distance between each observation and prediction
  //if one observation's distance with all prediction fall outside the gate,
  //this observed measurement is considered as false measurement and will be abandoned 
  void associateObservationWithPrediction(const vector<LineSegment>& z, const vector<Matrix2d>& R);
public:
  vector<LineSegment> z_predict;
  vector<MatrixXd> H;
  vector<pair<int,int>> matches;
  //association gate value 
  double g_;
/**
 *Posterior Estimation
 */
public:
  //compute posterior estimation of state and covariance matrix 
  void compPosteriorEstimation(const vector<LineSegment>& z, const vector<Matrix2d>& R);
public:
  VectorXd x_posterior;
  MatrixXd P_posterior;
/**
 * EKF
 */
public:
  //interface function 
  void ekfOneStep
    (const Vector3d& T_prev,const Vector2d& u, const vector<LineSegment>& z, const vector<Matrix2d>& R, const vector<LineSegment>& map_prev, const MatrixXd& P_prev, Vector3d& T_posterior, vector<LineSegment>& map_posterior, MatrixXd& P_posterior);
private: 
  long ekf_step_;//record how many steps have been processed in EkfFilter
  
};
}
#endif