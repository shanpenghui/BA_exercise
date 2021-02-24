cmake_minimum_required(VERSION 2.8)

project(BA_g2o)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(glog)

set(G2O_PATH ${CMAKE_CURRENT_BINARY_DIR}/../../ThirdParties/g2o)
message(${G2O_PATH}/cmake_modules)
LIST(APPEND CMAKE_MODULE_PATH "${G2O_PATH}/cmake_modules")
#set(G2O_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)
#message(${G2O_PATH})

#set(CMAKE_MODULE_PATH "${G2O_PATH};${CMAKE_MODULE_PATH}")
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(SuiteSparse)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${G2O_PATH}/install/include
        ${CSPARSE_INCLUDE_DIR} # 要加上，否则找不到cs.h
        ${SUITESPARSE_INCLUDE_DIRS}
)

link_directories(${G2O_PATH}/lib)
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse
        g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension
        g2o_opengl_helper g2o_stuff g2o_types_sclam2d g2o_parser
        g2o_solver_dense g2o_solver_pcg g2o_solver_cholmod g2o_types_data g2o_types_sim3 ${SuiteSparse_LIBRARIES}
        cxsparse # 要加上，否则会出错
        )

add_executable(BA_g2o BA_g2o.cpp)
target_link_libraries(BA_g2o ${OpenCV_LIBS} ${G2O_LIBS} glog::glog Eigen3::Eigen)

add_executable(BA_g2o_structure_only BA_g2o_structure_only.cpp)
target_link_libraries(BA_g2o_structure_only ${OpenCV_LIBS} ${G2O_LIBS} glog::glog Eigen3::Eigen)

add_executable(g2o_examples_ba examples/ba/ba_demo.cpp)
target_link_libraries(g2o_examples_ba ${G2O_LIBS} Eigen3::Eigen)