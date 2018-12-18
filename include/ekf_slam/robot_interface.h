/**
 * Robot interface for the differential robot simulated in vrep with laser sensor 
 */
#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include "ekf_slam/common.h"

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
namespace ekf_slam{

class RobotInterface
{
  /**Robot State
   * Lost: RobotInterface find no robot to connect
   * Initialized: RobotInterface received all initialization message
   * Running: Robot is running and RobotInterface is receiving message from robot
   */
  enum class State {Lost, Initialized, Running};
  //Initial state is lost, no connection
  State state_{State::Lost};
public:
  using Ptr = std::shared_ptr<RobotInterface>;

public:
  RobotInterface();
  RobotInterface(ros::NodeHandle nh);
  //Cannot copy
  RobotInterface(const RobotInterface& ) = delete;
  RobotInterface& operator=(const RobotInterface& ) = delete;
public:
  /** 
   * Getters of robot configuration
   */ 
  //Circular robot diameter 
  double getRobotDiameter() const;
  //Two wheels' diameter
  double getWheelDiameter() const;
  //Distance between two wheels
  double getWheelDistance() const;
  //Transformation of Laser sensor frame in Robot reference frame
  SE2 getLaserFrameInRobotFrame() const;
  
  /**
   *  Getters of robot state(2D pose)
   */
  //Robot's absolute pose in world frame
  SE2 getRobotPose() const;
  //Robot's start pose in the world frame
  SE2 getStartPose() const;
  void setStartPose(const SE2& start_pose);
  //Robot's estimated pose in world frame
  SE2 getEstimatedPose() const;
  void setEstimatedPose(const SE2& estimate_pose);
  //Robot's target pose in world frame
  SE2 getTargetPose() const;
  void setTargetPose(const SE2& target_pose);
  
  /**
   * Getters of laser reading
   */
  PointCloud<PointXY> getLaserScanCoordInRobot() const;
  PointCloud<PointXY> getLaserScanCoordInWorld() const;
  
  /**
   * Robot command
   */
  void setWheelVelocity(double l_vel, double r_vel);
  
private:
  //robot configuration 
  double robot_diameter_{0};
  string topic_robot_diameter_;
  ros::Subscriber sub_robot_diameter_;
  
  double robot_wheel_diameter_{0};
  string topic_robot_wheel_diameter_;
  ros::Subscriber sub_robot_wheel_diameter_;
  
  double robot_wheel_distance_{0};
  string topic_robot_wheel_distance_;
  ros::Subscriber sub_robot_wheel_distance_;
  
  //laser frame in robot reference frame 
  SE2 T_robot_laser_;
  string topic_T_robot_laser_;
  ros::Subscriber sub_T_robot_laser_;
  
  PointCloud<PointXY> laser_scan_coord_robot_;
  RointCloud<PointXY> laser_scan_coord_world_;
  string topic_laser_scan;
  ros::Subscriber sub_laser_scan;
  
  //robot frame in world reference frame, ground truth  
  SE2 T_world_robot_;
  string T_world_robot_topic_;
  ros::Subscriber sub_T_world_robot_;
  
  //estimated robot frame in world reference frame 
  SE2 T_world_estimated_;
  string T_world_estimated_topic_;
  ros::Subscriber sub_T_world_estimated_;
  ros::Publisher pub_T_world_estimated_;
  
  //target frame in world reference frame 
  SE2 T_world_target_;
  string T_world_target_topic_;
  ros::Subscriber sub_T_world_target_;
  ros::Publisher pub_T_world_target_;
  
  double left_wheel_vel_{0};
  string left_wheel_vel_topic_;
  ros::Publisher pub_left_wheel_vel_;
  
  double right_wheel_vel_{0};
  string right_wheel_vel_topic_;
  ros::Publisher pub_right_wheel_vel_;

private:
  //Callback functions for robot configuration 
  void robot_dia_callback(const std_msgs::Float32& msg);
  void robot_wheel_dia_callback(const std_msgs::Float32& msg);
  void robot_wheel_dist_callback(const std_msgs::Float32& msg);
  void laser_frame_callback(const geometry_msgs::Pose2D& msg);
  //Callback functions for laser scan, convert LaserScan message to 2D coordinate 
  void laser_scan_callback(const sensor_msgs::LaserScan& msg);
  
  
  //Callback functions for robot state
  void T_wr_callback(const geometry_msgs::Pose2D& msg);
  void T_we_callback(const geometry_msgs::Pose2D& msg);
  void T_wt_callback(const geometry_msgs::Pose2D& msg);
  //convertion between Pose2D and SE2 
  inline SE2 convertPose2DToSE2(const geometry_msgs::Pose2D& msg);
  inline geometry_msgs::Pose2D convertSE2ToPose2D(const SE2& se2);
 
};
SE2 RobotInterface::convertPose2DToSE2(const geometry_msgs::Pose2D& msg)
{
  return SE2(msg.theta,Vector2d(msg.x,msg.y));
}

geometry_msgs::Pose2D RobotInterface::convertSE2ToPose2D(const SE2& se2)
{
  geometry_msgs::Pose2D pose2D;
  pose2D.x = se2.translation()[0];
  pose2D.y = se2.translation()[1];
  pose2D.theta = se2.so2().log();
}

}

#endif //ROBOT_INTERFACE_H_
