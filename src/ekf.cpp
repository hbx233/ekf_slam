#include "ekf_slam/ekf.h"
namespace ekf_slam{
EKFilter::EKFilter(double k, double b, double g)
 :k_(k),b_(b),g_(g),ekf_step_(0){}

void EKFilter::stateTransition(const Vector3d& x_prev, const Matrix3d P_prev, const Vector2d& u)
{
  x_prior(0) = x_prev(0) + (u(0)+u(1))/2 * cos(x_prev(2) +(u(1)-u(0))/(2*b_));
  x_prior(1) = x_prev(1) + (u(0)+u(1))/2 * sin(x_prev(2) +(u(1)-u(0))/(2*b_));
  x_prior(2) = x_prev(2) + (u(1)-u(0))/b_;
  //jacobian matrix of state transition function
  //with respect to state and control input 
  //F_x
  F_x = Matrix3d::Identity();
  F_x(0,2) = -(u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_x(1,2) =  (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  //F_u
  F_u(0,0) = 0.5 * cos(x_prev(2)+(u(1)-u(0))/(2*b_)) + 1/(2*b_) * (u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(0,1) = 0.5 * cos(x_prev(2)+(u(1)-u(0))/(2*b_)) - 1/(2*b_) * (u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(1,0) = 0.5 * sin(x_prev(2)+(u(1)-u(0))/(2*b_)) - 1/(2*b_) * (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(1,1) = 0.5 * sin(x_prev(2)+(u(1)-u(0))/(2*b_)) + 1/(2*b_) * (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(2,0) = -1/b_;
  F_u(2,1) = 1/b_;
  
  //Control input noise covariance matrix 
  Matrix2d Q = Matrix2d::Zero();
  Q(0,0) = k_ * std::abs(u(0));
  Q(1,1) = k_ * std::abs(u(1));
  //compute covariance matrix for prior estimation
  P_prior = F_x * P_prev * F_x.transpose() + F_u * Q * F_u.transpose();
}
void EKFilter::predictMeasurementsFromPriorState(const vector<LineSegment>& landmarks)
{
  bool isNegative;
  //clear the z_hat and H 
  z_predict = vector<LineSegment>(landmarks.size());
  H = vector<Matrix23d>(landmarks.size());
  for(int i = 0; i<landmarks.size(); i++){
    //predict the measurement based on prior estimation of state: x_hat_
    landmarks[i].convertToFrameT(x_prior(0),x_prior(1),x_prior(2),z_predict[i],isNegative);
    H[i] = Matrix23d::Zero();
    H[i](0,2) = -1;
    if(isNegative){
      //changed the sign of r, so need to change sign of H_ for second row
      //cout<<"Negative r"<<endl;
      H[i](1,0) = cos(landmarks[i].alpha_);
      H[i](1,1) = sin(landmarks[i].alpha_);
    } else{
      H[i](1,0) = -cos(landmarks[i].alpha_);
      H[i](1,1) = -sin(landmarks[i].alpha_);
    }
  }
}
void EKFilter::associateObservationWithPrediction(const vector<LineSegment>& z, const vector<Matrix2d>& R)
{
  //initialize distance matrix 
  double d[z.size()][z_predict.size()]={0};
  //compute the distance between each observation and prediction 
  for(int i = 0; i<z.size(); i++){
    for(int j = 0; j<z_predict.size(); j++){
      //compute innovation z-z_hat_
      Vector2d v(z[i].alpha_-z_predict[j].alpha_, z[i].r_-z_predict[j].r_);
      //compute innovation covariance 
      Matrix2d cov = H[j] * P_prior * H[j].transpose() + R[i];
      //compute mahalanobis distance 
      d[i][j] = v.transpose() * cov.inverse() * v;
    }
  }
  //threshold and determine the association from distance 
  for(int i =0; i<z.size(); i++){
    double smallest_distance = g_;
    int smallest_index = -1;
    for(int j=0; j<z_predict.size(); j++){
      if(d[i][j]<g_){
	//fall in the gate
	if(d[i][j]<smallest_distance){
	  smallest_distance = d[i][j];
	  smallest_index = j;
	}
      }
    }
    if(smallest_index!=-1){
      //successfully matched  
      matches.push_back(pair<int,int>(i,smallest_index));
    }
  }
}
void EKFilter::compPosteriorEstimation(const vector<LineSegment>& z, const vector<Matrix2d>& R,Vector3d& x_posterior, Matrix3d& P_posterior)
{
  //make H and R 
  int n = matches.size(); //number of valid association 
  Eigen::MatrixXd H_stacked(2*n,3);
  Eigen::MatrixXd R_diag(2*n,2*n);
  R_diag=Eigen::MatrixXd::Zero(2*n,2*n);
  Eigen::VectorXd v_stacked(2*n);
  //from matches stack all H and v and block diagonalize all R
  for(int i = 0; i<matches.size(); i++){
    H_stacked.block(2*i,0,2,3) = H[matches[i].second];
    R_diag.block(2*i,2*i,2,2) = R[matches[i].first];
    v_stacked.segment(2*i,2) = z[matches[i].first].vector() - z_predict[matches[i].second].vector();
  }
  //compute Kalman gain 
  Eigen::MatrixXd cov(2*n,2*n);
  cov = H_stacked * P_prior * H_stacked.transpose() + R_diag;
  Eigen::MatrixXd K(3,2*n);
  K = P_prior * H_stacked.transpose() * cov.inverse();
  x_posterior = x_prior + K * v_stacked;
  P_posterior = P_prior - K * cov * K.transpose();
}
void EKFilter::ekfOneStep(const Vector3d& x_prev, const Matrix3d& P_prev, const Vector2d& u, const vector<LineSegment>& z, const vector<Matrix2d>& R, const vector< LineSegment >& landmarks, Vector3d& x_posterior, Matrix3d& P_posterior)
{
  //clear all the internal data 
  z_predict.clear();
  H.clear();
  matches.clear();
  
  //state transition, get prior estimation  
  stateTransition(x_prev,P_prev,u);
  //measurement prediction 
  predictMeasurementsFromPriorState(landmarks);
  //associate measurement prediction with observed measurement 
  associateObservationWithPrediction(z,R);
  //extended kalman filter, get posterior estimation 
  compPosteriorEstimation(z,R,x_posterior,P_posterior);
  
  //increase filter step 
  ekf_step_++;
}



}