HUMBLE_INSTALL_DIR=~/ros2_humble/
DEB_VERSION=$(env -i bash -c '. /etc/os-release; echo $VERSION_CODENAME')


InstallRos2()
{
  locale  # check for UTF-8

  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  locale  # verify settings

  apt-cache policy | grep universe

  sudo apt install software-properties-common
  sudo add-apt-repository universe

  sudo apt update && sudo apt install curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $DEB_VERSION) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-flake8 \
    python3-flake8-* \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget

  sudo apt install libtinyxml2-dev
  sudo apt install libasio-dev 
  sudo apt install libacl1-dev
  sudo apt install gcc g++ cmake libacl1-dev libncurses5-dev pkg-config
  sudo apt install libeigen3-dev
  sudo apt install python3-lark
  sudo apt install python3-numpy
  sudo apt install libldap2-dev
  sudo apt install rtirq-init
  sudo apt install libbullet-dev
  sudo apt install python3-opencv
  sudo apt install libopencv-dev
  sudo apt-get install mesa-common-dev
  sudo apt install libqt5core5a
  sudo apt-get install qtbase5-dev
  sudo apt-get install qtdeclarative5-dev
  sudo apt install python-is-python3
  sudo pip3 install netifaces


  # Remove  root access for serial port
  sudo apt remove modemmanager
#  sudo usermod -a -G dialout username

#  THE_DIR=$(pwd)
#
#  cd /tmp/
#  git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
#  cd cyclonedds
#  mkdir build
#  cmake ../
#  make
#  sudo make install
#
#  cd $THE_DIR



  mkdir -p $HUMBLE_INSTALL_DIR/src
  cp ros2.repos.32bit $HUMBLE_INSTALL_DIR/

  cd $HUMBLE_INSTALL_DIR
  #wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
  vcs import src < ros2.repos.32bit

  sudo apt upgrade

  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

  colcon build --symlink-install --packages-skip-build-finished
  echo ". ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
}

InstallCm730()
{
  sudo systemctl mask brltty.path
  sudo systemctl mask brltty
  cd $HUMBLE_INSTALL_DIR/src
  rm -rf ros2_cm730
  git clone https://github.com/thedancomplex/ros2_cm730

  cd $HUMBLE_INSTALL_DIR
  colcon build --symlink-install --packages-skip-build-finished
}

ShowUsage()
{
	echo 
	echo '==================================='
	echo '==================================='
	echo '====== ROS2 and CM730 Install======'
	echo '======  for the Darwin OP    ======'
	echo '==================================='
	echo '======== Daniel M. Lofaro ========='
	echo '======== dan@danlofaro.com ========'
	echo '==================================='
	echo '==================================='
	echo ''
	echo 'ros2   : installs ros 2 from source'
        echo '         (~24hr on Darwin OPs CPU) '
	echo 'cm730  : installs cm730 (ros2)     '
	echo '         drivers                   '
	echo
}


case "$1" in
	'ros2' )
		InstallRos2 $@
	;;
	
	'cm730' )
		InstallCm730 $@
	;;
	
	* )
		ShowUsage
		exit 1
esac

exit 0



