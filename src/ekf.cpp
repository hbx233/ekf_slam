#include "ekf_slam/ekf.h"
namespace ekf_slam{
EKFilter::EKFilter(double k, double b, double g)
 :k_(k),b_(b),g_(g),ekf_step_(0){}

void EKFilter::constructStateVector(const Vector3d& T_prev, const vector<LineSegment>& map_prev, const MatrixXd& P_prev)
{
  //first CHECK if the provided covariance matrix's size same with state plus map
  int n = map_prev.size();
  if(P_prev.rows() != P_prev.cols() || P_prev.rows() != (3+2*n)){
    ERROR_MSG("Covariance Matrix size incorrect: ");
    cout<<"Size should be: "<<3+2*n<<", "<<"but have size: "<<P_prev.rows()<<endl;
    exit(0);
  }
  //state vector for ekf 
  //previous state and covariance 
  x_prev = VectorXd(3+2*n);
  //concatinate robot state and map ehtries to make state vector 
  x_prev.head(3) = T_prev;
  for(int i=0; i<n; i++){
    x_prev.segment(3+2*i,2) = map_prev[i].vector();
  }
}

void EKFilter::initVectorAndMatrix(const int& map_size)
{
  int n = 3 + 2*map_size;
  // initiate all the internal vector and matrix
  x_prior = VectorXd(n);
  P_prior = MatrixXd(n,n);
  F_x = MatrixXd::Identity(n,n);
  F_u = MatrixXd::Zero(n,2);
  // initiate measurement prediction
  z_predict.clear();
  H.clear();
  z_predict = vector<LineSegment>(map_size);
  H = vector<MatrixXd>(map_size,MatrixXd::Zero(2,n));
}


void EKFilter::stateTransition(const MatrixXd& P_prev, const Vector2d& u)
{
  //only do state transition for robot state, map's state won't change
  x_prior(0) = x_prev(0) + (u(0)+u(1))/2 * cos(x_prev(2) +(u(1)-u(0))/(2*b_));
  x_prior(1) = x_prev(1) + (u(0)+u(1))/2 * sin(x_prev(2) +(u(1)-u(0))/(2*b_));
  x_prior(2) = x_prev(2) + (u(1)-u(0))/b_;
  //map won't change in state transition 
  x_prior.tail(x_prev.size()-3) = x_prev.tail(x_prev.size()-3);
  //jacobian matrix of state transition function
  //with respect to state and control input 
  //F_x
  //already initialized to identity matrix of size [3+2*n,3+2*n]
  F_x(0,2) = -(u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_x(1,2) =  (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  //map related jacobian matrix relative to itself is identity matrix 
  //F_x(3:end,3:end)=I
  //F_u
  //already 
  //map's landmark won't change with input 
  F_u = MatrixXd::Zero(x_prev.size(),2);
  F_u(0,0) = 0.5 * cos(x_prev(2)+(u(1)-u(0))/(2*b_)) + 1/(2*b_) * (u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(0,1) = 0.5 * cos(x_prev(2)+(u(1)-u(0))/(2*b_)) - 1/(2*b_) * (u(0) + u(1))/2 * sin(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(1,0) = 0.5 * sin(x_prev(2)+(u(1)-u(0))/(2*b_)) - 1/(2*b_) * (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(1,1) = 0.5 * sin(x_prev(2)+(u(1)-u(0))/(2*b_)) + 1/(2*b_) * (u(0) + u(1))/2 * cos(x_prev(2)+(u(1)-u(0))/(2*b_));
  F_u(2,0) = -1/b_;
  F_u(2,1) = 1/b_;
  //map related jacobian matrix relative to control input is zero
  
  //Control input noise covariance matrix 
  Matrix2d Q = Matrix2d::Zero();
  Q(0,0) = 0.0004; //k_ * std::abs(u(0));
  Q(1,1) = 0.0004; //k_ * std::abs(u(1));
  //compute covariance matrix for prior estimation
  P_prior = F_x * P_prev * F_x.transpose() + F_u * Q * F_u.transpose();
}
void EKFilter::predictMeasurementsFromPriorState(const vector<LineSegment>& map_prev)
{
  bool isNegative;
  //compute H for every 
  for(int i = 0; i<map_prev.size(); i++){
    //predict the measurement based on prior estimation of state: x_prior
    map_prev[i].convertToFrameT(x_prior(0),x_prior(1),x_prior(2),z_predict[i],isNegative);
    //compute the jacobian of predicted measurement 
    H[i](0,2) = -1;
    if(isNegative){
      //changed the sign of r, so need to change sign of H_ for second row
      //cout<<"Negative r"<<endl;
      H[i](1,0) = cos(map_prev[i].alpha_);
      H[i](1,1) = sin(map_prev[i].alpha_);
      if(i>1){
        H[i](0,3+2*i) = 1;
        H[i](1,3+2*i) = -x_prior(0)*sin(map_prev[i].alpha_) + x_prior(1)*cos(map_prev[i].alpha_);
        H[i](1,3+2*i+1) = -1;
      }
    } else{
      H[i](1,0) = -cos(map_prev[i].alpha_);
      H[i](1,1) = -sin(map_prev[i].alpha_);
      if(i>1){
        H[i](0,3+2*i) = 1;
        H[i](1,3+2*i) = x_prior(0)*sin(map_prev[i].alpha_) - x_prior(1)*cos(map_prev[i].alpha_);
        H[i](1,3+2*i+1) = 1;
      }
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
      if(i==0 && j==0){
	cout<<"first distance"<<endl;
	cout<<v<<endl;
	cout<<H[0]<<endl;
	cout<<P_prior<<endl;
	cout<<H[j] * P_prior * H[j].transpose()<<endl;
	cout<<R[i]<<endl;
	cout<<cov<<endl;
	cout<<cov.inverse()<<endl;
      }
    }
  }
  //output d for debug 
  cout<<"[DEBUG] d value"<<endl;
  for(int i=0; i<z.size();i++){
    for( int j=0; j<z_predict.size(); j++){
      cout<<d[i][j]<<' ';
    }
    cout<<endl;
  }
  //threshold and determine the association from distance 
  for(int i =0; i<z.size(); i++){
    double smallest_distance = std::pow(g_,2);
    int smallest_index = -1;
    for(int j=0; j<z_predict.size(); j++){
      if(d[i][j]<std::pow(g_,2)){
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
void EKFilter::compPosteriorEstimation(const vector<LineSegment>& z, const vector<Matrix2d>& R)
{
  //make H and R 
  int n = matches.size(); //number of valid association 
  Eigen::MatrixXd H_stacked(2*n,x_prior.size());
  Eigen::MatrixXd R_diag(2*n,2*n);
  R_diag=Eigen::MatrixXd::Zero(2*n,2*n);
  Eigen::VectorXd v_stacked(2*n);
  //from matches stack all H and v and block diagonalize all R
  for(int i = 0; i<matches.size(); i++){
    H_stacked.block(2*i,0,2,x_prior.size()) = H[matches[i].second];
    R_diag.block(2*i,2*i,2,2) = R[matches[i].first];
    v_stacked.segment(2*i,2) = z[matches[i].first].vector() - z_predict[matches[i].second].vector();
  }
  cout<<"H_stacked"<<endl;
  cout<<H_stacked<<endl;
  cout<<"R_diag"<<endl;
  cout<<R_diag<<endl;
  cout<<"v_stacked"<<endl;
  cout<<v_stacked<<endl;
  
  //compute Kalman gain 
  Eigen::MatrixXd cov(2*n,2*n);
  cov = H_stacked * P_prior * H_stacked.transpose() + R_diag;
  Eigen::MatrixXd K;
  K = P_prior * H_stacked.transpose() * cov.inverse();
  x_posterior = x_prior + K * v_stacked;
  P_posterior = P_prior - K * cov * K.transpose();
}
void EKFilter::ekfOneStep(const Vector3d& T_prev,const Vector2d& u, const vector<LineSegment>& z, const vector<Matrix2d>& R, const vector<LineSegment>& map_prev, const MatrixXd& P_prev, Vector3d& T_posterior, vector<LineSegment>& map_posterior, MatrixXd& P_posterior)
{ 
  constructStateVector(T_prev,map_prev,P_prev);
  initVectorAndMatrix(map_prev.size());
  //state transition, get prior estimation  
  stateTransition(P_prev,u);
  //measurement prediction 
  predictMeasurementsFromPriorState(map_prev);
  //associate measurement prediction with observed measurement 
  associateObservationWithPrediction(z,R);
  //extended kalman filter, get posterior estimation 
  compPosteriorEstimation(z,R);
  //convert posterior estimation to T and map
  T_posterior = x_posterior.head<3>();  
  //first copy the previous map to map posterior estimation 
  map_posterior = map_prev;
  for(int i=0; i<map_prev.size();i++){
    //change line estimation of map, keep other things in map unchanged  
    map_posterior[i].alpha_ = x_posterior(3+2*i);
    map_posterior[i].r_ = x_posterior(3+2*i+1);
  }
  P_posterior = this->P_posterior;
  //increase filter step 
  ekf_step_++;
}



}