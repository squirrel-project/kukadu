#include <iostream>
#include <vector>

// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH
using namespace std;

int main( int argc, char** argv ) {
	
	int fakeArgc = 2;
	char** fakeArgv = new char*[2];
	fakeArgv[0] = "demo-simple";
	fakeArgv[1] = "--sdh_rs_device=/dev/ttyUSB1";
	
	argc = fakeArgc;
	argv = fakeArgv;
	
	SDH_ASSERT_TYPESIZES();

	//---------------------
	// handle command line options: set defaults first then overwrite by parsing actual command line
	cSDHOptions options;

	options.Parse( argc, argv, "", "demo-griphand", "v1", cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
	//
	//---------------------

	//---------------------
	// initialize debug message printing:
	cDBG cdbg( options.debug_level > 0, "red", options.debuglog );
	g_sdh_debug_log = options.debuglog;

	cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

	// reduce debug level for subsystems
	options.debug_level-=1;
	//---------------------

	try
	{
		// Create an instance "hand" of the class cSDH:
		cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
		cdbg << "Successfully created cSDH instance\n";

		// Open configured communication to the SDH device
		options.OpenCommunication( hand );
		cdbg << "Successfully opened communication to SDH\n";

		// Switch to "pose" controller mode and set default velocities first:
		hand.SetController( hand.eCT_POSE );

		cSDH::eGraspId grasp_id = cSDH::eGID_CENTRICAL; // start with first valid grasp
		double close_ratio;
		double velocity = 25.0;

//		while ( grasp_id < cSDH::eGID_DIMENSION )
//		{
			// remark: the std::cout.flush() is needed to make the output appear in time
			// when compiled with VCC
			std::cout << "Performing GripHand() for grasp " << int(grasp_id) << "=\"" << hand.GetStringFromGraspId( grasp_id ) << "\"\n"; std::cout.flush();

			close_ratio = 0.0;
			std::cout << "   close_ratio = 0.0 (fully open)..."; std::cout.flush();
			hand.GripHand( grasp_id, close_ratio, velocity, true );
			std::cout << " finished\n"; std::cout.flush();
			SleepSec( 1.0 );
			
			close_ratio = 1.0;
			std::cout << "   close_ratio = 1.0 (closed open)..."; std::cout.flush();
			hand.GripHand( grasp_id, close_ratio, velocity, true );
			std::cout << " finished\n"; std::cout.flush();
			SleepSec( 1.0 );
			
			double d;
			cin >> d;
/*
			close_ratio = 0.5;
			std::cout << "   close_ratio = 0.5 (half closed)..."; std::cout.flush();
			hand.GripHand( grasp_id, close_ratio, velocity, true );
			std::cout << " finished\n"; std::cout.flush();
			SleepSec( 1.0 );
*/
/*
			close_ratio = 0.9;
			std::cout << "   close_ratio = 1.0 (fully closed)..."; std::cout.flush();
			hand.GripHand( grasp_id, close_ratio, velocity, true );
			std::cout << " finished\n"; std::cout.flush();
*/
/*
			close_ratio = 0.0;
			std::cout << "   reopening..."; std::cout.flush();
			hand.GripHand( grasp_id, close_ratio, velocity, true );
			std::cout << " finished\n"; std::cout.flush();

			grasp_id = (cSDH::eGraspId)( grasp_id + 1 );
//		}
*/
		// Finally close connection to SDH again, this switches the axis controllers off
	//	hand.Close();
	}
	catch (cSDHLibraryException* e)
	{
		std::cerr << "demo-griphand main(): An exception was caught: " << e->what() << "\n";
		delete e;
	}
	
}
