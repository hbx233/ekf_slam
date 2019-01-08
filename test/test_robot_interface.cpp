#include "ekf_slam/robot_interface.h"
#include "ekf_slam/laser_frame.h"
#include "ekf_slam/common.h"
#include "ekf_slam/ekf.h"
#include "ekf_slam/line_detector.h"
#include "ekf_slam/line_segment.h"
using ekf_slam::RobotInterface;
using ekf_slam::LaserFrame;
using ekf_slam::LineSegment;
using ekf_slam::EKFilter;
int main(int argc, char **argv){
  ros::init(argc, argv, "ekf slam");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(20);
  
  int run_count = 1;
  RobotInterface::Ptr robotPtr = std::make_shared<ekf_slam::RobotInterface>(nh);
  Vector2d curr_joint;
  Vector2d past_joint;
  
  EKFilter::Ptr ekfPtr = std::make_shared<EKFilter>(0.1,0.25,std::sqrt(10));
  Vector3d T_prev(0,-2,1.5714);
  vector<LineSegment> map_prev(1,LineSegment(1,1));
  MatrixXd P_prev = MatrixXd::Zero(5,5);
  P_prev(0,0) = 0.01; P_prev(1,1) = 0.01; P_prev(2,2) =0.01;
  P_prev(3,3) = 4; P_prev(4,4) = 4;
  while(ros::ok()){
    cout<<"Run: "<<run_count<<endl;
    if(robotPtr->state_==RobotInterface::State::Initialized){
      cout<<"Initialized"<<endl;
    } else if(robotPtr->state_ == RobotInterface::State::Running){
      cout<<"Running"<<endl;
      if(run_count==1){
	//set current joint value
	curr_joint = robotPtr->getWheelJointValue();
	robotPtr->setRobotVelocity(0.5,0.2);
	robotPtr->getElapsedTimeFromLastCall();
	past_joint = curr_joint;
	//set initial estimated pose and set estimated pose visibility to true
	robotPtr->setEstimatedPose(Vector3d(0,-2,1.5714));
	robotPtr->setEstimatedPoseVisibility(true);
      } else{
        double elp_time = robotPtr->getElapsedTimeFromLastCall();
        past_joint = curr_joint;
        curr_joint = robotPtr->getWheelJointValue();
        //calculate input 
        Vector2d vel = robotPtr->getWheelVel();
	
        //Vector2d u = vel * elp_time * 0.05/2;
        Vector2d u = (curr_joint-past_joint)*0.05;
	cout<<"elapsed time: "<<elp_time<<endl;
        cout<<u<<endl;
        cout<<vel<<endl;
	//do state transition step 
	ekfPtr->constructStateVector(T_prev,map_prev,P_prev);
	ekfPtr->initVectorAndMatrix();
	ekfPtr->stateTransition(P_prev,u);
	T_prev = ekfPtr->x_prior.head<3>();
	robotPtr->setEstimatedPose(T_prev);
	cout<<"Estimation Error: "<<endl;
	cout<<T_prev-robotPtr->robotPose()<<endl;
      }
      run_count++;
    } else{
      cout<<"Lost"<<endl;
    }
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}