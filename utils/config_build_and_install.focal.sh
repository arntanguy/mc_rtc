export ROS_DISTRO=noetic
export SYSTEM_HAS_SPDLOG=ON
export APT_DEPENCENCIES="curl wget cmake build-essential gfortran doxygen libeigen3-dev doxygen doxygen-latex libboost-all-dev libtinyxml2-dev libgeos++-dev libnanomsg-dev libyaml-cpp-dev libltdl-dev qt5-default libqwt-qt5-dev libspdlog-dev ninja-build"
export APT_PYTHON2_DEPENDENCIES="cython python-nose python-numpy python-coverage python-setuptools"
export APT_PYTHON3_DEPENDENCIES="cython3 python3-pip python3-nose python3-numpy python3-coverage python3-setuptools python3-matplotlib python3-pyqt5"

if [ "x$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ]
then
  export APT_DEPENCENCIES="$APT_DEPENDENCIES $APT_PYTHON2_DEPENDENCIES $APT_PYTHON3_DEPENDENCIES"
elif [ "x$PYTHON_FORCE_PYTHON3" == xON ]
then
  export APT_DEPENCENCIES="$APT_DEPENDENCIES $APT_PYTHON3_DEPENDENCIES"
elif [ "x$PYTHON_FORCE_PYTHON2" == xON ]
then
  export APT_DEPENCENCIES="$APT_DEPENDENCIES $APT_PYTHON2_DEPENDENCIES"
fi

if $BUILD_BENCHMARKS
then
  export APT_DEPENDENCIES="$APT_DEPENDENCIES libbenchmark-dev"
fi

mc_rtc_extra_steps()
{
  if [ x"$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ] || [ x"$PYTHON_FORCE_PYTHON2" == "xON" ] 
  then
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && sudo python2 get-pip.py && rm -f get-pip.py
    sudo apt install --reinstall python3-pip # The boostrap script above seems to remove the system's pip3 executable
    sudo pip install matplotlib
  fi
  if [ x"$PYTHON_BUILD_PYTHON2_AND_PYTHON3" == xON ] || [ x"$PYTHON_FORCE_PYTHON3" == "xON" ] 
  then
    export MC_LOG_UI_PYTHON_EXECUTABLE=python3
  fi
}
