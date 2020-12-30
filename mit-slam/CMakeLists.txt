cmake_minimum_required(VERSION 3.5)
project(mit-slam)

set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# NOTE(tonioteran) Hack to make OpenMP work for our old version of CMake
# (https://cliutils.gitlab.io/modern-cmake/chapters/packages/OpenMP.html).
find_package(OpenMP REQUIRED)
# For CMake < 3.9, we need to make the target ourselves
if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX
    PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX
    PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

# Find PCL
find_package(PCL REQUIRED)

# Try to get TEASER++ to be recognized.
find_package(teaserpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  opencv3_catkin
  tf
  tf2
  eigen_conversions
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  gtsam_catkin
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    opencv3_catkin
    tf
    tf2
    image_transport
    cv_bridge
    gtsam_catkin
)

# Create SLAM library.
set(SLAM_SRCS
  src/BlobTracker.cpp
  src/GeometryUtils.cpp
  src/ParamUtils.cpp
  src/MsgsUtils.cpp
  src/SlamNode.cpp
  src/CloudOdometer.cpp
  src/GraphManager.cpp
  src/RotationKinematicFactor.cpp
  )
add_library(mit-slam SHARED ${SLAM_SRCS})
target_include_directories(mit-slam PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(mit-slam PUBLIC -ltbbmalloc ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${PCL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_features)
# Quick test without teaser.
# target_link_libraries(mit-slam PUBLIC ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# Whenever using Eigen3 > 3.3.0
# target_include_directories(mit-slam PUBLIC include)
# target_link_libraries(mit-slam PUBLIC ${catkin_LIBRARIES} Eigen::Eigen3 teaserpp::teaser_registration)


# Quick checkout node test.
include_directories(${catkin_INCLUDE_DIRS})
add_executable(quick_checkout src/quick_checkout.cpp)
#target_include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(quick_checkout ${catkin_LIBRARIES})

# Disabled for now due to change in registration function
# Quick sample test of TEASER++
#add_executable(teaser_checkout src/teaser_checkout.cpp)
#target_link_libraries(teaser_checkout mit-slam)

# Main executable for the SLAM system.
include_directories(${catkin_INCLUDE_DIRS})
add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node mit-slam ${catkin_LIBRARIES})

# Node to calculate ground truth relative pose estimate.
add_executable(relative_pose_node src/relative_pose_node.cpp)
target_link_libraries(relative_pose_node mit-slam)

# Include unit tests directory to the project.
#enable_testing()
#add_subdirectory(test)

# create_test_targets(DIR test
#   LIBS mit-slam
#   INC include
#   )

#install_launch_files()
file (GLOB LAUNCH_FILES launch/*.launch launch/*.yaml launch/*.xml)
get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Astrobee-esque way of adding messages. Assumes folder named 'msg' is in the package.
add_message_files(
  FILES
  AngState.msg
  InertialParams.msg
)
