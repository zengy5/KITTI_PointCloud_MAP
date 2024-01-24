cmake_minimum_required(VERSION 3.0.2)
project(kitti)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(kitti kitti_merge.cpp)
target_link_libraries(kitti
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}

  )
