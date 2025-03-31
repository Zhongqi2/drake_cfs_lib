# CMake Project with an Installed Drake

This uses the CMake `find_package(drake)` mechanism to find an installed instance of Drake.

# Instructions

These instructions are only supported for Ubuntu 22.04 (Jammy).

```shell
###############################################################
# Install Prerequisites
###############################################################
# Various system dependencies
sudo ../scripts/setup/linux/ubuntu/jammy/install_prereqs

# (Optionally) Install GTest
# You could also explicitly pull gtest into the CMake build directly:
#     https://github.com/google/googletest/tree/master/googletest
sudo apt-get install libgtest-dev
ls
mkdir ~/gtest && cd ~/gtest && cmake /usr/src/gtest && make
sudo cp *.a /usr/local/lib

###############################################################
# Install Drake to $HOME/drake
###############################################################

# 1) A specific version (date-stamped)
# curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-20240214-jammy.tar.gz

# 2) The latest (usually last night's build)
curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz
tar -xvzf drake-latest-jammy.tar.gz -C $HOME

# 3) Manual Installation
# git clone https://github.com/RobotLocomotion/drake.git
# (mkdir drake-build && cd drake-build && cmake -DCMAKE_INSTALL_PREFIX=$HOME/drake ../drake && make)

# 4) Manual Installation w/ Licensed Gurobi
# Install & setup gurobi (http://drake.mit.edu/bazel.html?highlight=gurobi#install-on-ubuntu)
# git clone https://github.com/RobotLocomotion/drake.git
# (mkdir drake-build && cd drake-build && cmake -DCMAKE_INSTALL_PREFIX=$HOME/drake -DWITH_GUROBI=ON ../drake && make)

###############################################################
# Build Everything
###############################################################
git clone https://github.com/RobotLocomotion/drake-external-examples.git
cd drake-external-examples
mkdir drake_cmake_installed-build && cd drake_cmake_installed-build
cmake -DCMAKE_PREFIX_PATH=$HOME/drake ../drake_cmake_installed
make# trajectory_optimization_lib
