#Build Settings
cmake_minimum_required(VERSION 3.0)
PROJECT( ekf_slam ) 
set( CMAKE_CXX_STANDARD 11 ) 
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#Third Party Dependencies 
##Eigen 
include_directories("/usr/local/include/eigen3")

##Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

##OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
##ROS
###Find catkin and catkin backages 
find_package(catkin REQUIRED roscpp std_msgs sensor_msgs geometry_msgs)

set( THIRD_PARTY_LIBS 
     ${catkin_LIBRARIES}
     )
###declare ROS messages and services 
#NONE 
###Generate added messages and services 
#generate_messages(DEPENDENCIES std_msgs)

##Declare a catkin package 
catkin_package() #is it necessary?

include_directories(${catkin_INCLUDE_DIRS})

#Project directory 
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
