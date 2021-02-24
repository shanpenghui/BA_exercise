echo "Installing the third parties ..."

cd ..

cd ThirdParties

currentDir=$(pwd)
#echo ${currentDir}

# Install ceres
#rm -rf ceres-solver
sudo apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
if  [ -d "ceres-solver" ];then
  echo  ""
else
  git clone https://github.com/ceres-solver/ceres-solver.git
fi
cd ceres-solver
if  [ -d "build" ];then
  echo  ""
else
  mkdir build
fi
cd build && git checkout master
cmake .. -DCMAKE_INSTALL_PREFIX="${currentDir}/ceres-solver" && make -j32 && make install && cd .. && cd ..
echo $(pwd)

# Install g2o
#rm -rf g2o
if  [ -d "g2o" ];then
  echo  ""
else
  git clone https://github.com/RainerKuemmerle/g2o.git
fi
cd g2o
if  [ -d "build" ];then
  echo  ""
else
  mkdir build
fi
if  [ -d "install" ];then
  echo  ""
else
  mkdir install
fi
cd build && git checkout master && git pull
cmake .. -DCMAKE_INSTALL_PREFIX="${currentDir}/g2o/install" && make -j32 && make install && cd .. && cd ..
echo $(pwd)

# Install glog
#rm -rf glog
if  [ -d "glog" ];then
  echo  ""
else
  git clone https://github.com/google/glog.git
fi
cd glog
if  [ -d "build" ];then
  echo  ""
else
  mkdir build
fi
if  [ -d "install" ];then
  echo  ""
else
  mkdir install
fi
cd build && git checkout master && git pull
cmake .. && make -j32 && sudo make install
