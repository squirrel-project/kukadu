#include "TerminalCostComputer.h"

using namespace std;

double TerminalCostComputer::computeCost(std::shared_ptr<ControllerResult> results) {
	
	double delta = 0.0;
	cout << "(TerminalCostComputer) Enter the deviation in query space (also be aware of the sign; e.g. qmeasured +/- cost = qdesired)...";
	cin >> delta;
	
	return delta;
	
}
