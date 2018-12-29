#include "ekf_slam/robot_interface.h"
#include "ekf_slam/laser_frame.h"
#include "ekf_slam/common.h"
int main(int argc, char **argv){
  ros::init(argc, argv, "ekf slam");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(10);
  
  ekf_slam::RobotInterface::Ptr robotPtr = std::make_shared<ekf_slam::RobotInterface>(nh);
  
  while(ros::ok()){
    if(robotPtr->state_==ekf_slam::RobotInterface::State::Initialized){
      cout<<"Initialized"<<endl;
    } else if(robotPtr->state_ == ekf_slam::RobotInterface::State::Running){
      cout<<"Running"<<endl;
    } else{
      cout<<"Lost"<<endl;
    }
    robotPtr->setRobotVelocity(0,30);
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}