cmake_minimum_required(VERSION 2.8.3)
project(husky_trainer)

# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    sensor_msgs 
    geometry_msgs 
    roscpp
    pointmatcher_ros
)

find_package(Eigen REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES husky_trainer
 CATKIN_DEPENDS roscpp pcl_ros
 DEPENDS system_lib eigen pointmatcher_ros
)

###########
## Build ##
###########

add_definitions(-DHAVE_YAML_CPP)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

link_directories(${Eigen_LIBRARY_DIRS} ${libpointmatcher_LIB})
add_definitions(${Eigen_DEFINITIONS})

add_executable(
husky_teach_node
include/husky_trainer/AnchorPoint.h
include/husky_trainer/PointMatching.h
include/husky_trainer/GeoUtil.h
src/GeoUtil.cpp
src/PointMatching.cpp
src/AnchorPoint.cpp
src/teach.cpp
)

add_executable(
husky_repeat_node
include/husky_trainer/AnchorPoint.h
include/husky_trainer/PointMatching.h
include/husky_trainer/GeoUtil.h
src/GeoUtil.cpp
src/PointMatching.cpp
src/AnchorPoint.cpp
src/repeat.cpp)

target_link_libraries(husky_teach_node ${catkin_LIBRARIES} pointmatcher)
target_link_libraries(husky_repeat_node ${catkin_LIBRARIES} pointmatcher)


catkin_add_gtest(
husky_trainer_test
src/GeoUtil.cpp
src/AnchorPoint.cpp
src/PointMatching.cpp
test/husky_trainer_test.cpp
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(husky_trainer_test pointmatcher ${catkin_LIBRARIES})
