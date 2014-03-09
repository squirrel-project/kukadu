#ifndef SCHUNKHAND
#define SCHUNKHAND

#include <iostream>

#include "GenericHand.h"

#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

/** \brief Provides control capabilities for the Schunk SDH robotic hand
 * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
 * \ingroup RobotFramework
 */
class SchunkHand : public GenericHand {

private:

    const char* usbDevice;
	
	SDH::cSDHBase::eGraspId currentGrasp;
	
	SDH::cSDH* hand;
	cSDHOptions* options;

public:
	
	/** \brief SchunkHand constructor taking the connection port name
	 * \param usbDevice usb port name (e.g. "/dev/ttyUSB0")
	 */
    SchunkHand(const char* usbDevice);

	void connectHand();
	void closeHand(double percentage, double velocity);
	void disconnectHand();
	void setGrasp(SDH::cSDHBase::eGraspId grasp);
	void safelyDestroy();

};

#endif
