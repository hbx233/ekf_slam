#include "ekf_slam/system.h"
#include "ekf_slam/robot_interface.h"
using ekf_slam::EKFilter;
using ekf_slam::LaserFrame;
using ekf_slam::LineDetector;
using ekf_slam::LineSegment;
using ekf_slam::Map;
using ekf_slam::System;
using ekf_slam::RobotInterface;

#define ST 0
#define EKF 1

int main(int argc, char** argv){
  //ros initiation 
  ros::init(argc, argv, "ekf slam");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(20);
  //create a robot interface 
  RobotInterface::Ptr robot = std::make_shared<RobotInterface>(nh);
  //get robot parameter 
  double wheel_distance = robot->robotWheelDistance();
  double wheel_radius = robot->robotWheelDiameter()/2;
  cout<<wheel_radius<<endl;
  //create ekfilter
  EKFilter::Ptr ekfilter = std::make_shared<EKFilter>(0.01,wheel_distance,std::sqrt(10));
  //create LineDetect 
  LineDetector::Ptr line_detector = LineDetector::create(0.01,15);
  //create a slam system wrapper 
  System::Ptr system = std::make_shared<System>(line_detector,ekfilter);
  
  //spin loop 
  long run_count=0;
  while(ros::ok()){
    //fsm switch for robot state
    switch(robot->state_){
      case RobotInterface::State::Initialized:
      {
	//initialized all interface parameter and connected all publishers and subscribers 
	//but still not receiving any data from robot 
	cout<<"Initialized, no data received"<<endl;
	break;
      }
      case RobotInterface::State::Running:
      {
	cout<<"Robot is Running, receiving data"<<endl;
	//received data streamd from robot
	//1. collect all the data received from robot
	shared_ptr<LaserFrame> frame_new = robot->getLaserFrame();
	Vector2d joint_value_new = robot->getWheelJointValue();
	//2. begin SLAM System fsm switch
	switch(system->state_){
	  case System::State::INITIALIZING:
	  {
	    Vector3d T_init(0,-2,1.5714);
	    Matrix3d P_T_init = Matrix3d::Zero();
	    P_T_init(0,0) = 0.1;P_T_init(1,1) = 0.1; P_T_init(2,2) = 0.1;
#if ST
	    //empty frame
	    LaserFrame::Ptr empty_frame = std::make_shared<LaserFrame>();
	    system->initSLAM(T_init,empty_frame,P_T_init,joint_value_new);
#endif
#if EKF
	    system->initSLAM(T_init,frame_new,P_T_init,joint_value_new);
#endif
	    //set velocity 
	    robot->setRobotVelocity(0.1,0);
	    //set initial pose visualization
	    robot->setEstimatedPose(T_init);
	    robot->setEstimatedPoseVisibility(true);
	    break;
	  }
	  case System::State::RUNNING:
	  {
#if EKF
	    system->oneSLAMStep(frame_new,joint_value_new,wheel_radius);
#endif	    
#if ST    
	    system->oneStateTransitionStep(joint_value_new,wheel_radius);
#endif    
	    //display the estimated pose 
	    cout<<system->pose()<<endl;
	    robot->setEstimatedPose(system->pose());
	    break;
	  }
	  case System::State::LOST:
	  {
	    //TODO:
	    //Need to determine in which case the robot is lost(cannot get a good estimation of state)
	    cout<<"Robot is Lost for state estimation"<<endl;
	    break;
	  }
	}
	break;
      }
      case RobotInterface::State::Lost:
      {
	cout<<"Connection to robot is lost"<<endl;
	break;
      }
    }
    //ros spin 
    ros::spinOnce();
    loopRate.sleep();
  }
}