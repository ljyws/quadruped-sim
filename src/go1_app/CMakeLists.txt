cmake_minimum_required(VERSION 3.0.2)
project(go1_app)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
)

find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES go1_app
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs unitree_legged_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/local/include/osqp)
find_package(OsqpEigen REQUIRED)


add_library(go1_lib
        src/Go1Params.h
        src/Go1CtrlState.h
        src/RosInterface.h
        src/RosInterface.cpp
        
        )

add_executable(go1_ctrl src/main.cpp)
target_link_libraries(go1_ctrl
        go1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )




