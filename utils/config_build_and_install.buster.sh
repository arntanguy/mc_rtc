export ROS_DISTRO=noetic
export APT_DEPENDENCIES="wget cmake build-essential gfortran doxygen cython cython3 python-pip python3-pip python-nose python3-nose python-numpy python3-numpy python-coverage python3-coverage python-setuptools python3-setuptools libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libgeos++-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev python-git python-pyqt5 qt5-default libqwt-qt5-dev python-matplotlib"
export ROS_APT_DEPENDENCIES="ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-rosdoc-lite ros-${ROS_DISTRO}-common-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz"
if $BUILD_BENCHMARKS
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES libbenchmark-dev"
fi

mc_rtc_extra_steps()
{
  # FIXME: There is currently no released version of catkin_tools for buster so we're building from source here
  # Remove this hack once catkin_tools is officially released
  sudo pip install --user git+https://github.com/catkin/catkin_tools.git
}
