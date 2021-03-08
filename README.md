# BA_exercise
A exercise of BA, using g2o, ceres and eigen.

感谢六哥、黄志明、付堉家的支持。

# Clone
git clone --recurse-submodules -j8 https://github.com/shanpenghui/BA_exercise

# Third Parties
The version listed is currently used. Check current version by the command in brackets.

- Cmake 3.16.3 ++ (cmake --version)
- Eigen 3.3.7 ++ (pkg-config --modversion eigen3)
- OpenCV 4.2.0 ++ (pkg-config --modversion opencv4)
- g2o (Sat Feb 27 18:55:01 2021 ++)
- ceres (Wed Feb 17 18:38:29 2021 ++)
- glog (Mon Mar 1 12:59:21 2021 ++)

# Install Third Parties

You should install these libraries in order.

## 1.Eigen
```shell script
sudo apt update && sudo apt-get install libeigen3-dev
or
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && mkdir build && cd build && git checkout 3.3.7 && cmake .. && make -j32 && sudo make install
```

## 2.OpenCV
Ref: [https://docs.opencv.org/master/d0/d3d/tutorial_general_install.html](https://docs.opencv.org/master/d0/d3d/tutorial_general_install.html)

```shell script
sudo apt update && sudo apt-get install libeigen3-dev
or
git clone https://github.com/opencv/opencv
git -C opencv checkout 4.2.0
git clone https://github.com/opencv/opencv_contrib
git -C opencv_contrib checkout 4.2.0
git clone https://github.com/opencv/opencv_extra
git -C opencv_extra checkout 4.2.0
cmake ..
make -j32
sudo make install
```

## 3.g2o
Ref: [https://github.com/RainerKuemmerle/g2o](https://github.com/RainerKuemmerle/g2o)

**Remember to change the DCMAKE_INSTALL_PREFIX path!!!**

```shell script
sudo apt update && sudo apt install libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && mkdir install && mkdir build && cd build && git checkout master
cmake .. && make -j32 && sudo make install
or
cmake .. -DCMAKE_INSTALL_PREFIX="/home/shenz/Documents/vslam/vslam_exercise/ThirdParties/g2o/install" && make -j32 && make install
```

## 4.ceres
Ref: [http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)

**Remember to change the DCMAKE_INSTALL_PREFIX path!!!**

```shell script
# CMake google-glog gflags BLAS LAPACK Eigen3 SuiteSparse(optional) CXSparse(optional)
sudo apt update && sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver && mkdir build && cd build && git checkout master
cmake .. && make -j32 && sudo make install
or
cmake .. -DCMAKE_INSTALL_PREFIX="/home/shenz/Documents/vslam/vslam_exercise/ThirdParties/ceres-solver" && make -j4 && make install 
```

## 5.googlelog
Ref: [https://github.com/google/glog.git](https://github.com/google/glog.git)

```shell script
git clone https://github.com/google/glog.git
cd glog && mkdir build && cd build && git checkout master && cmake .. && make -j32 && sudo make install
```

# Usage

```
mkdir build && cd build && cmake .. && make -j32
cd ceres && ./BA_ceres
or others such as:
cd g2o && ./BA_g2o
```

# 输出
## g2o
![image](https://github.com/shanpenghui/BA_exercise/blob/main/imgs/g2o.png)
## ceres
![image](https://github.com/shanpenghui/BA_exercise/blob/main/imgs/ceres.png)
## eigen
![image](https://github.com/shanpenghui/BA_exercise/blob/main/imgs/eigen.png)

# 备注

当前版本是没有优化point pose的，即只优化位姿
