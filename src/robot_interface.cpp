#include "ekf_slam/common.h"
#include "ekf_slam/robot_interface.h"

namespace ekf_slam {



double RobotInterface::getRobotDiameter() const
{
  if(state_ == State::Initialized){
    return robot_diameter_;
  } else{
    ERROR_MSG("Robot Configuration not Received");
    return 0;
  }
}

double RobotInterface::getWheelDistance() const
{
  if(state_ == State::Initialized){
    return robot_wheel_distance_;
  } else{
    ERROR_MSG("Robot Configuration not Received");
    return 0;
  } 
}
double RobotInterface::getWheelDiameter() const
{
  if(state_ == State::Initialized){
    return robot_wheel_diameter_;
  } else{
    ERROR_MSG("Robot Configuration not Received");
    return 0;
  }
}
SE2 RobotInterface::getLaserFrameInRobotFrame() const
{
  if (state_ == State::Initialized) {
    return T_robot_laser_;
  } else{
    ERROR_MSG("Robot Configuration not Received");
    return 0;
  }
}

// Robot Configuration Callback functions 
void RobotInterface::robot_dia_callback(const std_msgs::Float32& msg)
{
  robot_diameter_ = msg.data;
}
void RobotInterface::robot_wheel_dia_callback(const std_msgs::Float32& msg)
{
  robot_wheel_diameter_ = msg.data;
}
void RobotInterface::robot_wheel_dist_callback(const std_msgs::Float32& msg)
{
  robot_wheel_distance_ = msg.data;
}
void RobotInterface::laser_frame_callback(const geometry_msgs::Pose2D& msg)
{
  T_robot_laser_ = convertPose2DToSE2(msg);
}

// Robot State Callback functions 
void RobotInterface::T_wr_callback(const geometry_msgs::Pose2D& msg)
{
  T_world_robot_ = convertPose2DToSE2(msg);
}
void RobotInterface::T_we_callback(const geometry_msgs::Pose2D& msg)
{
  T_world_estimated_ = convertPose2DToSE2(msg);
}
void RobotInterface::T_wt_callback(const geometry_msgs::Pose2D& msg)
{
  T_world_target_ = convertPose2DToSE2(msg);
}

// Laser Scan Callback function 
void RobotInterface::laser_scan_callback(const sensor_msgs::LaserScan& msg)
{
  
}
















}


