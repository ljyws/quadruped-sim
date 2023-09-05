cmake_minimum_required(VERSION 3.0.2)
project(go1_ctrl)


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


add_library(go1_ctrl_lib
            # src/common/Controllers/FootSwingTrajectory.h
            # src/common/Controllers/FootSwingTrajectory.cpp
            # src/common/Controllers/GaitScheduler.h
            # src/common/Controllers/GaitScheduler.cpp
            # src/common/Math/Interpolation.h
            src/common/cppTypes.h
            src/common/Types.h
            src/common/cTypes.h
            src/common/Utilities/utilities.h
            src/common/Utilities/Utilities_print.h
            src/common/Utilities/Utilities_print.cpp
            src/common/Utilities/Timer.h
            src/common/Utilities/PeriodicTask.h
            src/common/Utilities/PeriodicTask.cpp
            src/robot/SimulationBridge.h
            src/robot/SimulationBridge.cpp
            src/common/ControlParameters/RobotParameters.h
            src/common/ControlParameters/ControlParameters.h
            src/common/ControlParameters/ControlParameters.cpp
            src/user/unitree_Controller.h
            src/user/unitree_Controller.cpp
            src/robot/main_app.h
            src/robot/main_app.cpp
        
        )

add_executable(go1_ctrl src/user/main.cpp)

target_link_libraries(go1_ctrl
        go1_ctrl_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )




