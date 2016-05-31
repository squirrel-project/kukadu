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
 * 
 * Prev (\ref introductionpage), Next (\ref gettingstartedpage)
*/

/*! \page gettingstartedpage Getting Started
 * TODO: here should be an example on how to start kukadu with a
 * simulator example and a test script that is actually moving
 * 
 * Prev (\ref installationpage), Next (\ref modulespage)
*/

/*! \page modulespage Modules
 * Currently, kukadu consists for 4 major modules:
 *   - The robot module (\ref robotpage): The robot module is the
 * interface to a concrete robot. It provides an
 * interface that defines what kukadu expects a robot to be able to do.
 * If this interface is implemented for a specific robot, the complete
 * stack of the framework is available.
 *   - The kinematics module (\ref kinematicspage): As the name already
 * says, the kinematics module provides kinematics
 * for a given robot. It includes interfaces and implementations for
 * forward and inverse kinematics as well as path planning functionality.
 * The only prerequisite is an appropriate robot model in URDF format.
 * Currently, interfaces to MoveIt and KOMO are available.
 *   - The control module (\ref controlpage): The control module provides
 * interfaces for control algorithms as well as certain implementations
 * of these interfaces such as DMPs and splines.
 *   - The machine learning module (\ref mlpage): The machine learning
 * module provides interfaces for supervised learning and reinforcement
 * learning. Further, these interfaces are implemented for several methods
 * such as
 *     - Linear regression
 *     - Locally weighted regression
 *     - Gaussian process regression
 *     - SVM
 *     - Control policy reinforcement learning (PoWER)
 *     - Reinforcement learning (Projective simulation, Monte carlo
 * RL, Q-Learning, SARSA)
 * 
 * Prev (\ref gettingstartedpage), Next (\ref robotpage)
 * 
*/

/*! \page robotpage The robot module
 * The \ref Robot module is the
 * interface to a concrete robot. It provides an
 * interface that defines what kukadu expects a robot to be able to do.
 * If this interface is implemented for a specific robot, the complete
 * stack of the framework is available.
 * 
 * The major interface is the abstract kukadu::ControlQueue class. It
 * defines all the functions that are available for a robot (e.g. a
 * robotic arm) without the robot base. If you want to bind kukadu
 * to the robot in your lab, you have to implement the pure virtual
 * functions of the kukadu::ControlQueue. Currently, there exist 2
 * implementations of kukadu::ControlQueue:
 *   - kukadu::KukieControlQueue: Binds the kukadu framework to robots
 * for which the <a href="https://iis.uibk.ac.at/intranet/projects/robot/armtopics">kukie</a>
 * framework is available. Kukie is a robot control framework based
 * on ROS that defines certain topics and services for low-level
 * robot control. Check out the kukie page to find out, if kukie
 * is available for your system.
 *   - kukadu::PlottingControlQueue: Provides the kukadu functionility
 * independent of the actual robot. With the kukadu::PlottingControlQueue no
 * robot can be controlled, but it can be used as dummy robot in order
 * to test other functionality. The kukadu::PlottingControlQueue simply
 * ignores all submitted control commands.
 * 
 * <h1>kukadu::KukieControlQueue</h1>
 * The following code snippet shows how to use the kukadu::KukieControlQueue.
 * In order to make the program run, make sure that the kukie system is
 * started (e.g. by starting the simulator or the real robot).
 * \includelineno control_queue.cpp
 * 
 * <h2>Code description</h2>
 * Lets have a closer look to the tutorial code. First of all, the
 * kukadu library needs to be included (line 1). All kukadu functionality
 * is contained in the kukadu namespace (line 4). kukadu is a framework
 * embedded in ROS, so the first action that should be done is setting
 * up the ros connection (lines 8 - 10).
 * 
 * The direct connection to the robot is given by the kukadu::ControlQueue
 * interface. This means that an implementation of that interface should
 * be instantiated per robot. This is done line 13 where an instance
 * of the kukadu::KukieControlQueue is created for the left arm in
 * simulation. In line 16, the queue is started - so the queue is ready
 * to go. However, you can't move the robot until it is switched to
 * an appropriate control mode (lines 19 - 22). Afterwards, the robot
 * is ready to go, so you should pay special attention to it.
 * 
 * In the first step, we can make the robot arm move by a simple
 * point to point movement (line 26). This means that you don't need to take
 * care how the robot reaches the desired position. All the path planning
 * is done automatically by the system and you only need to execute
 * the kukadu::ControlQueue::jointPtp() function (there is also a method
 * for kukadu::ControlQueue::cartesianPtp()). This function blocks
 * until the last packet is sent to the robot - however, it is not guaranteed
 * that the robot reaches that position with precision. You can check
 * if it was precise enough by looking at the returned joint positions.
 * 
 * After the PtP execution the program retrieves the current joint state
 * of the robot with the kukadu::Controlqueue::getCurrentJoints() function.
 * There is also a function for the Cartesian state named kukadu::ControlQueue::getCurrentCartesianPose().
 * 
 * The final step of the demo program is the execution of a specific
 * trajectory. In this case you can not only define the target but also
 * the path the robot should use to get there. In general, the kukadu::ControlQueue
 * is a clocked queue that submits one joint packet per clock cycle.
 * This packet is then submitted to the robot. If no new packet is
 * set, then the robot is commanded to stay at the current position.
 * In lines 34 - 40, a specific trajectory is submitted to the queue
 * by added one packet per clock cycle using the kukadu::ControlQueue::move()
 * function. This procedure is synchronized to the queue by the
 * kukadu::ControlQueue::synchronizeToQueue() function. This function blocks
 * until at most N packets (in this case N = 1) is left in the queue.
 * This makes sure that your program stays in synch with the queue.
 * However, if you dont use this function, all packets are added to the queue
 * at once and the queue sent on packet per clock cycle. However, you need
 * to make sure to synchronize with the queue otherwise then.
 * 
 * In lines 44 - 53, the queue is disconnected from the robot. The most
 * important part is to leave the execution mode (line 47). After leaving
 * the execution mode, the queue is stopped (line 50) and the program
 * waits until the queue is stopped completely (line 53).
 * 
 * Prev (\ref modulespage), Next (\ref kinematicspage)
*/

/*! \page kinematicspage The kinematics module
 * Prev (\ref robotpage), Next (\ref controlpage)
*/

/*! \page controlpage The control module
 * Prev (\ref kinematicspage), Next (\ref mlpage)
*/

/*! \page mlpage The machine learning module
 * Prev (\ref controlpage)
*/
