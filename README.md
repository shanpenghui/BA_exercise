# BA_exercise
A exercise of BA, using g2o, ceres and eigen
Before build the source, make sure that the third party library is installed rightly.
其中g2o版本是六哥的版本，感谢六哥。同时也感谢黄志明的大力支持和解惑，非常感谢。

# Clone
git clone --recurse-submodules -j8 https://github.com/shanpenghui/BA_exercise

# ThirdParties

Just run a shell file to install third parties.
```shell script
cd shells && ./insall_third_parties.sh
```

Or you can choose to install them manually.
## 1.googlelog
[https://github.com/google/glog.git](https://github.com/google/glog.git)

```shell script
git clone https://github.com/google/glog.git
cd glog && rm -rf build && mkdir build && cd build && git checkout master && git pull
cmake .. && make -j4 && sudo make install
```
## 2.g2o
[https://github.com/RainerKuemmerle/g2o](https://github.com/RainerKuemmerle/g2o)

### Requirements

-   C++14 compiler (CI pipeline runs with gcc, clang and MSVC)
-   cmake             <http://www.cmake.org>
-   Eigen3            <http://eigen.tuxfamily.org>

On Ubuntu / Debian these dependencies are resolved by installing the
following packages.

-   cmake
-   libeigen3-dev

### Optional requirements

-   suitesparse       <http://faculty.cse.tamu.edu/davis/suitesparse.html>
-   Qt5               <http://qt-project.org>
-   libQGLViewer      <http://www.libqglviewer.com>

On Ubuntu / Debian these dependencies are resolved by installing the
following packages.

-   libsuitesparse-dev
-   qtdeclarative5-dev
-   qt5-qmake
-   libqglviewer-dev-qt5

**Remember to change the DCMAKE_INSTALL_PREFIX path!!!**

```shell script
sudo apt install libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
git clone https://github.com/RainerKuemmerle/g2o
cd g2o && rm -rf build install && mkdir install && mkdir build && cd build && git checkout master && git pull
cmake .. -DCMAKE_INSTALL_PREFIX="/home/shenz/Documents/ba/BA_exercise/ThirdParties/g2o/install" && make -j4 && make install
```

## 3.ceres
[http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)

```shell script
# CMake google-glog gflags BLAS LAPACK Eigen3 SuiteSparse(optional) CXSparse(optional)
sudo apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
```

**Remember to change the DCMAKE_INSTALL_PREFIX path!!!**

```shell script
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && rm -rf build install && mkdir build && mkdir install && cd build && git checkout master && git pull
cmake .. -DCMAKE_INSTALL_PREFIX="/home/shenz/Documents/ba/BA_exercise/ThirdParties/ceres-solver/install" && make -j32 && make install 
```

# Usage

## 1.Start BA_g2o
```
cd g2o
mkdir build
cd build
cmake ..
make -j4
./BA_g2o
```

## 2.Start BA_ceres
>Before running BA_ceres, you can get used to using ceres by the examples offered in source folder named ceres
```
cd ceres
mkdir build
cd build
cmake ..
make -j4
./BA_ceres
```

## 3.Start BA_eigen
```
cd eigen
mkdir build
cd build
cmake ..
make -j4
./BA_eigen
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
