cmake_minimum_required(VERSION 2.8)

project(BA_eigen)

find_package(Eigen3 REQUIRED)
find_package(glog)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(BA_eigen_only_pose BA_eigen_only_pose.cpp)
target_link_libraries(BA_eigen_only_pose glog::glog Eigen3::Eigen)

add_executable(BA_eigen_pose_and_points BA_eigen_pose_and_points.cpp)
target_link_libraries(BA_eigen_pose_and_points glog::glog Eigen3::Eigen)

