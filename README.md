# BA_exercise
A exercise of BA, using g2o, ceres and eigen
Note that all CMakeList files is for CLion IDE, may get sth wrong in terminal, especially the third party library.

# Clone
git clone --recurse-submodules -j8 https://github.com/shanpenghui/BA_exercise

# Usage

## 1.Build g2o
```
cd Thirdparty/g2o
mkdir build
cd build
cmake ..
make -j4
```

## 2.Build ceres
```
cd Thirdparty/ceres-solver
mkdir build
cd build
cmake ..
make -j4
```

## 3.Start BA_g2o

## 4.Start BA_ceres

>Before running BA_ceres, you can get used to using ceres by the examples offered in source folder named ceres
