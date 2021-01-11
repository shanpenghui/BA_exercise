cmake_minimum_required(VERSION 2.8)

project(BA_ceres)

set(CERES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)
message(${CERES_PATH})

find_package(Ceres REQUIRED PATHS "${CERES_PATH}/Thirdparty/ceres-solver")
include_directories(${CERES_INCLUDE_DIRS})


add_executable(Ceres_exam1_hello_world Ceres_exam1_hello_world.cpp)
target_link_libraries(Ceres_exam1_hello_world ${CERES_LIBRARIES})

add_executable(Ceres_exam2_BAL Ceres_exam2_BAL.cpp)
target_link_libraries(Ceres_exam2_BAL ${CERES_LIBRARIES})