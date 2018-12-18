/**
 * Global include header file 
 * author: Yuxuan Huang 
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <memory>
#include <iostream>
#include <string>
using std::vector;
using std::string;
using std::pair;
using std::shared_ptr;
using std::cout;
using std::endl;
//Third Party dependices 
//Eigen 
#include <Eigen/Core>
using Eigen::Vector2d;
//Sophus
#include <sophus/so2.h>
#include <sophus/se2.h>
using Sophus::SO2;
using Sophus::SE2;
//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using pcl::PointXY;
using pcl::PointCloud;

#define ERROR_MSG(msg) cout<<msg<<endl
