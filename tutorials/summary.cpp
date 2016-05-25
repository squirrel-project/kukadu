/*! \mainpage kukadu Tutorials
  \tableofcontents
  \section introduction Introduction
  \ref introductionpage
  \section installation Installation
  \ref installationpage
  \section gettingstarted Getting Started
  \ref gettingstartedpage
  \section modules kukadu Modules
  \ref modulespage
  \subsection robot The robot module
  \ref robotpage
  \subsection kinematics The kinematics and Planning module
  \ref kinematicspage
  \subsection control The control module
  \ref controlpage
  \subsection ml The machine learning module
  \ref mlpage
*/
/*! \page introductionpage Introduction
* kukadu is a software framework for robotics applications. It was used
* exensively in the lab of the
* <a href="http://iis.uibk.ac.at">Intelligent and Interactive</a>
* systems group at the University of Innsbruck. It supports several
* different modules that are essential for robotics applications such
* as
* 	- Robot control (\ref robotpage)
* 	- Kinematics and path planning (\ref kinematicspage)
* 	- Robust control policies (\ref controlpage)
* 	- Machine learning (\ref mlpage)
* 		- Regression
* 		- Classification
* 		- Kernel Methods
* 		- Reinforcement Learning
* 
* kukadu is written in C++ and supports usage in combination with
* <a href="http://www.ros.org/">ROS</a>. It provides several general
* interfaces and implements it with state of the art methods from
* control, machine learning and robotic manipulation.
* 
* This document provides a setup guide (\ref installationpage)
* including a recommendation on how
* to develop kukadu software using <a href="https://www.qt.io/ide/">
* QtCreator</a>. Further, several tutorials and an API documentation
* is given.
* 
* Next (\ref installationpage)
*/

/*! \page installationpage Installation
 * kukadu requires a current <a href="http://www.ros.org/">ROS</a>
 * system installed on your PC. We recommend ROS Indigo, however,
 * using kukadu with any later version of ROS should not yield any
 * problem. An installation guide for different operating systems can
 * be found <a href="http://wiki.ros.org/indigo/Installation">here</a>.
 * Currently, kukadu is only supported with Ubuntu (>= 14.10) and
 * a gcc compiler that supports at least C++11.
 * 
 * As kukadu provides a clean interface to a wide range of robotics
 * applications and state-of-the art methods, several dependencies
 * have to be installed.
 * 
 * \code
 * sudo apt-get install libgsl0-dev gnuplot gnuplot-x11 libarmadillo-dev libboost-all-dev libncurses5-dev libarmadillo-dev liballegro5-dev ros-indigo-pcl-ros ros-indigo-moveit-ros-planning-interface python3.4-dev liblapacke-dev gtk+2.0 bison build-essential cmake doxygen fabric flex freeglut3-dev g++ gcc gfortran git-core gnuplot graphviz-dev libann-dev libcv-dev libcvaux-dev libdc1394-22-dev libf2c2-dev libgtest-dev libgtkglext1-dev libhighgui-dev liblapack-dev libplib-dev libqhull-dev libsdl1.2-dev libx11-dev libx11-dev libxi-dev libxmu-dev make meld python-nose python-unittest2 realpath regexxer swig2.0 tcl8.5-dev tk-dev tk8.5-dev libfreenect-dev qt5-default ros-indigo-desktop-full ros-indigo-cob-common ros-indigo-ros-comm ros-indigo-geometry ros-indigo-common-msgs ros-indigo-control-msgs ros-indigo-geometry-experimental libgsl0ldbl libgsl0-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev ros-indigo-moveit-core ros-indigo-moveit-ros-planning ros-indigo-moveit-ros-planning-interface libghc-zlib-dev zlibc zlib1g-dbg zlib-bin ros-indigo-qt-build libqwt6 libqwt-dev libsdl1.2-dev ros-indigo-moveit-full ros-indigo-cmake-modules ros-indigo-map-msgs ros-indigo-controller-manager
 * \endcode
 * 
 * After installing the dependencies, you can clone kukadu to a
 * <a href="http://wiki.ros.org/catkin">catkin</a> workspace from
 * our <a href="https://git-scm.com/">Git</a> repository. If you are
 * new to programming under ROS, you might be interested in the
 * <a href="http://wiki.ros.org/catkin/Tutorials">catkin tutorial</a>.
 * You can find the kukadu Git repository
 * <a href="https://github.com/shangl/kukadu">here</a>. If you are
 * not familiar with Git, simply go to your CATKIN_DIR/src directory
 * and insert the following line to your terminal
 * \code
 * git clone --recursive https://github.com/shangl/kukadu.git
 * \endcode
 * After cloning the repository, you can start compiling it by
 * change to the root of your catkin workspace and typing
 * \code
 * catkin_make
 * \endcode
 * After the successful compilation kukadu is ready and you may
 * continue with the next section (\ref gettingstartedpage).
*/

/*! \page gettingstartedpage Getting Started
 * Next (\ref modulespage)
*/

/*! \page modulespage Modules
*/

/*! \page robotpage The robot module
*/

/*! \page kinematicspage The kinematics module
*/

/*! \page controlpage The control module
*/

/*! \page mlpage The machine learning module
*/
