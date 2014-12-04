#ifndef CONTROLQUEUE
#define CONTROLQUEUE

#include "../utils/DestroyableObject.h"
#include "../utils/types.h"
#include <armadillo>
#include <memory>
#include <thread>

#define COMMAND_NOT_SET -100

/**
 * \defgroup RobotFramework
 * This framework provides hardware access to robots and hands
 */

/** \brief Provides an interface for robot controllers
 * 
 * This class provides a set of methods that are necessary to execute basic manipulations with robots. As this typically is a real time task, this class
 * contains a queue and sends data to the robot at specified times.
 * \ingroup RobotFramework
 */
class ControlQueue : public DestroyableObject {

private:
	
	int degOfFreedom;
    std::shared_ptr<std::thread> thr;

public:
	/** \brief Constructor taking the robot dependent degrees of freedom
	 * \param degOfFreedom number of robots degrees of freedom
	 */
	ControlQueue(int degOfFreedom);
	
	/**
	 * \brief Returns number of robots degrees of freedom
	 */
	int getMovementDegreesOfFreedom();
	
	/**
	 * \brief Starts new thread to control the robot with real time capability
	 */
    std::shared_ptr<std::thread> startQueueThread();
	
	/**
	 * \brief This method is started in a new thread by startQueue
	 */
	virtual void run() = 0;
	
	/**
	 * \brief Sets a flag to stop the control thread after current iteration is executed
	 */
	virtual void setFinish() = 0;
	
	/**
	 * \brief Adds next joint position to queue
	 * \param joints joints to add
	 */
    virtual void addJointsPosToQueue(arma::vec joints) = 0;
	
	/**
	 * \brief Switches robot modes. A state might be a real time command mode or an monitoring mode
	 * \param mode mode id
	 */
	virtual void switchMode(int mode) = 0;
	
	/**
	 * \brief Stops current mode and switches back to default mode (e.g. monitoring mode)
	 */
	virtual void stopCurrentMode() = 0;
	
	/**
	 * \brief Blocks, if more than the defined maximum element count is in the queue
	 * \param maxNumJointsInQueue maximum number of joints in queue
	 */
	virtual void synchronizeToControlQueue(int maxNumJointsInQueue) = 0;
	
	/**
	 * \brief Sets joints in which the should be in before robot enters command mode
	 * \param joints array of joint positions
	 */
    virtual void setStartingJoints(arma::vec joints) = 0;
	
	/**
	 * \brief Implements simple point to point movement in joint space
	 * \param joints array of joint positions
	 */
    virtual void moveJoints(arma::vec joints) = 0;
	
	/**
	 * \brief Changes the load data of the robot (e.g. needs to be used whenever robot picks up an object)
	 * \param loadMass mass of the picked up object
	 * \param loadPos position of the objects center of gravity relative to the manipulator
	 */
	virtual void setAdditionalLoad(float loadMass, float loadPos) = 0;
	
	/**
	 * \brief Sets certain stiffness parameters in cartesian space (if the robot supports this) according to a mass spring damper system model
	 * \param cpstiffnessxyz stiffness of the robot in the cartesian space
	 * \param cpstiffnessabc stiffness of the rotational axis of the tool mounting point in cartesian space
	 * \param cpdamping damping of the robots axis in cartesian space
	 * \param cpmaxdelta maximum allows deviation in cartesian space
	 * \param maxforce maximum allowed applied force
	 * \param axismaxdeltatrq maximum allowed applied torque
	 */
	virtual void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) = 0;
	
	/**
	 * \brief Returns current robot position in cartesian space
	 */
    virtual mes_result getCartesianPos() = 0;
	
	/**
	 * \brief Returns the robot joints the robot has been directly before starting command mode
	 */
    virtual arma::vec getStartingJoints() = 0;
	
	/**
	 * \brief Returns joints if the robot is in monitor mode
	 */
    virtual arma::vec retrieveJointsFromRobot() = 0;
	
	/**
	 * \brief Returns joints if the robot is in command mode
	 */
	virtual mes_result getCurrentJoints() = 0;

    virtual mes_result getCurrentJntFrcTrq() = 0;

    virtual mes_result getCurrentCartesianFrcTrq() = 0;
	
	/**
	 * \brief Returns true if the command mode initialization is done
	 */
	virtual bool isInitialized() = 0;

    virtual std::string getRobotName() = 0;
    virtual std::vector<std::string> getJointNames() = 0;
    
};

#endif
