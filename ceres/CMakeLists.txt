cmake_minimum_required(VERSION 2.8)

project(BA_ceres)

set(CERES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)
message(${CERES_PATH})

find_package(Ceres REQUIRED PATHS "${CERES_PATH}/Thirdparty/ceres-solver")
include_directories(${CERES_INCLUDE_DIRS})


add_executable(Ceres_hello_world Ceres_hello_world.cpp)
target_link_libraries(Ceres_hello_world ${CERES_LIBRARIES})
