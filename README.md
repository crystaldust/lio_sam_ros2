## Installation

### Install system packages

#### Arch Linux

```bash
# The AUR packages will compile from the source code, which takes some time.
$ yaourt --noconfirm pcl # select the aur/pcl
$ yaourt --noconfirm gtsam # select the aur/gtsam, or aur/gtsam-mkl if you install on an Intel CPU
```

#### Ubuntu(not tested)

```bash
# Install PCL
$ apt install libpcl-dev

# Install GTSAM
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
# More details:
# https://gtsam.org/get_started/
```

### Install ROS2 dependencies

```bash
$ mkdir ~/lio-sam-ros2-deps/src -p # or anywhere you like
$ cp ./deps.repos ~/lio-sam-ros2-deps
$ cd ~/lio-sam-ros2-deps/
$ vcs import src/ < deps.repos
$ colcon install --merge-install --cmake-args "-DCMAKE_CXX_STANDARD_LIBRARIES=-lpython3.9" # You might need the cmake-args if you're using a new Python version
$ source install/setup.bash # Don't forget to source the environment!
```

#### Build LIO-SAM ROS2

```bash
# Make sure you are now at the root of the lio-sam-ros2 project
$ colcon build
$ souce install/setup.bash
```

