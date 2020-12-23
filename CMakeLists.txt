cmake_minimum_required(VERSION 3.0.2)
project(realsense_ar_track)

##=============================================================================
## CMAKE Flags
##=============================================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

##=============================================================================
## DEPENDANCY
##=============================================================================

#system
include_directories("/usr/local/include")
link_directories("/usr/local/lib")


#gazebo & catkin
find_package(catkin REQUIRED COMPONENTS  roscpp rospy std_msgs geometry_msgs message_generation)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} )

generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(DEPENDS roscpp rospy  message_runtime std_msgs geometry_msgs )

#Eigen
find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})

##=============================================================================
## BUILD SETUP
##=============================================================================

add_executable(ar_tag_node src/ar_tag_subscribe_node.cpp)
target_link_libraries(ar_tag_node ${catkin_LIBRARIES})

