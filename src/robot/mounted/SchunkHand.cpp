#include "SchunkHand.h"

using namespace SDH;
using namespace std;

SchunkHand::SchunkHand(char* usbDevice) {
	
	this->usbDevice = usbDevice;
	
	// changed shunk hand demo code
	int fakeArgc = 2;
	char** fakeArgv = new char*[2];
	fakeArgv[0] = "demo-simple";
	fakeArgv[1] = new char[string("--sdh_rs_device=").length() + string(usbDevice).length() + 1];
	strcpy(fakeArgv[1], "--sdh_rs_device=");
	strcat(fakeArgv[1], usbDevice);
	
	SDH_ASSERT_TYPESIZES();

	// handle command line options: set defaults first then overwrite by parsing actual command line
	options = new cSDHOptions;

	options->Parse(fakeArgc, fakeArgv, "", "SchunkHand", "v1", cSDH::GetLibraryName(), cSDH::GetLibraryRelease());
	
	currentGrasp = cSDH::eGID_CENTRICAL;

}

void SchunkHand::setGrasp(cSDHBase::eGraspId grasp) {
	currentGrasp = grasp;
}

void SchunkHand::connectHand() {
	
	// Create an instance "hand" of the class cSDH:
	hand = new cSDH(options->use_radians, options->use_fahrenheit, options->debug_level);

	// Open configured communication to the SDH device
	options->OpenCommunication(*hand);

	// Switch to "pose" controller mode and set default velocities first:
	hand->SetController(hand->eCT_POSE);

}

void SchunkHand::closeHand(double percentage, double velocity) {
	
	std::cout << "(SdhHand) close_ratio = 0.0 (fully open)..." << endl;
	std::cout.flush();
	hand->GripHand(currentGrasp, percentage, velocity, true);
	std::cout << "(SdhHand) finished" << endl;
	std::cout.flush();
	
}

void SchunkHand::disconnectHand() {
	hand->Close();
}

void SchunkHand::safelyDestroy() {
	hand->EmergencyStop();
}