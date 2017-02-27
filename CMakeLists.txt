cmake_minimum_required(VERSION 2.8.3)
project(husky_trainer)

find_package(catkin REQUIRED COMPONENTS 
    sensor_msgs 
    geometry_msgs 
    roscpp
    pointmatcher_ros
    libpointmatcher
    pcl_ros
    std_msgs
    message_generation
    dynamic_reconfigure
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED COMPONENTS io)

add_message_files(
    FILES
    AnchorPointSwitch.msg
    NamedPointCloud.msg
    TrajectoryError.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

generate_dynamic_reconfigure_options(
    cfg/Repeat.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES husky_trainer
 CATKIN_DEPENDS 
    roscpp 
    pcl_ros
    sensor_msgs
    geometry_msgs
    message_runtime
    dynamic_reconfigure
 DEPENDS system_lib eigen pointmatcher_ros
)

###########
## Build ##
###########

add_definitions(-DHAVE_YAML_CPP)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

link_directories(${Eigen_LIBRARY_DIRS} ${libpointmatcher_LIB})
add_definitions(${Eigen_DEFINITIONS})

add_executable(
teach
include/husky_trainer/AnchorPoint.h
include/husky_trainer/PointMatching.h
include/husky_trainer/GeoUtil.h
src/GeoUtil.cpp
src/PointMatching.cpp
src/AnchorPoint.cpp
src/teach.cpp
)
add_dependencies(teach ${${PROJECT_NAME}_EXPORTED_TARGETS}) 

add_executable(
repeat
include/husky_trainer/Repeat.h
src/CommandRepeater.cpp
src/GeoUtil.cpp
src/PointMatching.cpp
src/AnchorPoint.cpp
src/Controller.cpp
src/Repeat.cpp
src/repeat_main.cpp
)
add_dependencies(repeat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
command_repeater
include/husky_trainer/CommandRepeater.h
include/husky_trainer/ControllerMappings.h
src/CommandRepeater.cpp
src/command_repeater_main.cpp
)


add_executable(teach_cloud_recorder 
    src/cloud_recorder.cpp 
    src/CloudRecorder.cpp
    include/husky_trainer/CloudRecorder.h)
target_link_libraries(teach_cloud_recorder ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(teach ${catkin_LIBRARIES} pointmatcher)
target_link_libraries(repeat ${catkin_LIBRARIES})
target_link_libraries(command_repeater ${catkin_LIBRARIES})



catkin_add_gtest(
husky_trainer_test
src/GeoUtil.cpp
src/AnchorPoint.cpp
src/PointMatching.cpp
test/husky_trainer_test.cpp
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(husky_trainer_test pointmatcher ${catkin_LIBRARIES})
