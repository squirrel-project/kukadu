GSLLIBS = -L/usr/local/lib -L./lib -lgsl -lgslcblas -lSDHLibrary-CPP -lstdc++
GSLINCS = -I/usr/local/include -I/usr/include
./lib/libkukadu.so:	./src/robot/robotDriver/src/kuka/friUdp.cpp ./src/robot/robotDriver/src/kuka/friRemote.cpp ./src/robot/robotDriver/src/friMain.cpp ./src/robot/robotDriver/src/friMisc.cpp ./src/robot/KukaControlQueue.cpp ./src/utils/utils.cpp ./src/utils/Tokenizer.cpp ./src/learning/GeneralFitter.cpp ./src/trajectory/DMPExecutor.cpp ./src/trajectory/TrajectoryGenerator.cpp ./src/trajectory/DMPTrajectoryGenerator.cpp ./src/trajectory/PolyTrajectoryGenerator.cpp ./src/trajectory/TrajectoryDMPLearner.cpp ./src/trajectory/DMPGeneralizer.cpp ./src/learning/GenericKernel.cpp ./src/learning/TricubeKernel.cpp ./src/learning/KernelRegressor.cpp ./src/learning/GaussianProcessRegressor.cpp ./src/learning/LWRRegressor.cpp ./src/robot/mounted/sdhoptions.cpp ./src/utils/DestroyableObject.cpp ./src/robot/mounted/GenericHand.cpp ./src/robot/mounted/SchunkHand.cpp ./src/learning/QuadraticKernel.cpp ./src/learning/GaussianKernel.cpp ./src/robot/robotDriver/src/kuka/friUdp.h ./src/robot/robotDriver/src/kuka/friRemote.h ./src/robot/robotDriver/src/friMain.h ./src/robot/robotDriver/src/friMisc.h ./src/robot/ControlQueue.h ./src/robot/ControlQueue.cpp ./src/robot/KukaControlQueue.h ./src/utils/utils.h ./src/utils/Tokenizer.h ./src/learning/GeneralFitter.h ./src/trajectory/DMPExecutor.h ./src/trajectory/TrajectoryGenerator.h ./src/trajectory/DMPTrajectoryGenerator.h ./src/trajectory/PolyTrajectoryGenerator.h ./src/trajectory/TrajectoryDMPLearner.h ./src/trajectory/DMPGeneralizer.h ./src/learning/GenericKernel.h ./src/learning/TricubeKernel.h ./src/learning/KernelRegressor.h ./src/learning/GaussianProcessRegressor.h ./src/learning/LWRRegressor.h ./src/robot/mounted/sdhoptions.h ./src/utils/DestroyableObject.h ./src/robot/mounted/GenericHand.h ./src/robot/mounted/SchunkHand.h ./src/learning/QuadraticKernel.h ./src/learning/GaussianKernel.h ./src/trajectory/DMPReinforcer.cpp ./src/trajectory/DMPReinforcer.h ./src/trajectory/TerminalCostComputer.cpp ./src/trajectory/CostComputer.h ./src/trajectory/GenDMPReinforcer.cpp ./src/utils/gnuplot-cpp/gnuplot_i.hpp ./src/utils/gnuplot-cpp/gnuplot_i.cpp ./include/kukadu.h
	g++ -w -std=c++0x -fPIC -shared -pthread -DOSNAME_LINUX=1 ./src/robot/robotDriver/src/kuka/friUdp.cpp ./src/robot/robotDriver/src/kuka/friRemote.cpp ./src/robot/robotDriver/src/friMain.cpp ./src/robot/robotDriver/src/friMisc.cpp ./src/robot/KukaControlQueue.cpp ./src/utils/utils.cpp ./src/utils/Tokenizer.cpp ./src/learning/GeneralFitter.cpp ./src/trajectory/DMPExecutor.cpp ./src/trajectory/TrajectoryGenerator.cpp ./src/trajectory/DMPTrajectoryGenerator.cpp ./src/trajectory/PolyTrajectoryGenerator.cpp ./src/trajectory/TrajectoryDMPLearner.cpp ./src/trajectory/DMPGeneralizer.cpp ./src/learning/GenericKernel.cpp ./src/learning/TricubeKernel.cpp ./src/learning/KernelRegressor.cpp ./src/learning/GaussianProcessRegressor.cpp ./src/learning/LWRRegressor.cpp ./src/robot/mounted/sdhoptions.cpp ./src/utils/DestroyableObject.cpp ./src/robot/mounted/GenericHand.cpp ./src/robot/mounted/SchunkHand.cpp ./src/learning/QuadraticKernel.cpp ./src/learning/GaussianKernel.cpp ./src/trajectory/DMPReinforcer.cpp ./src/trajectory/TerminalCostComputer.cpp ./src/trajectory/GenDMPReinforcer.cpp ./src/robot/ControlQueue.cpp ./src/utils/gnuplot-cpp/gnuplot_i.cpp -o ./lib/libkukadu.so $(GSLINCS) $(GSLLIBS) -larmadillo
./bin/dmp:	./lib/libkukadu.so ./src/main.cpp ./include/kukadu.h
	g++ -w -std=c++0x -pthread -DOSNAME_LINUX=1 ./src/main.cpp -o ./bin/dmp $(GSLINCS) $(GSLLIBS) -larmadillo -lkukadu
./bin/drill:	./lib/libkukadu.so ./src/mainDrill.cpp ./include/kukadu.h
	g++ -w -std=c++0x -pthread -DOSNAME_LINUX=1 ./src/mainDrill.cpp -o ./bin/drill $(GSLINCS) $(GSLLIBS) -larmadillo -lkukadu
kukadu: ./lib/libkukadu.so
main:	./bin/dmp
exe:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -e -i$(in)
sim:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -s -i$(in)
mes:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -m -o$(out)
mesgen:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -n -i$(in) -o$(out)
rl:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -r -b$(base) -o$(out) -q$(qout) -p
gen:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -g
testing:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -t
testhand:
	g++ -DOSNAME_LINUX=1 ./src/robot/mounted/testHand.cpp ./src/robot/mounted/sdhoptions.cpp -o ./bin/testhand $(GSLINCS) $(GSLLIBS)
	./bin/testhand
drillMain:	./bin/drill
drillSim:	drillMain
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/drill -sim
drillExe:	drillMain
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/drill -exe
./bin/screw:	./lib/libkukadu.so ./src/mainScrew.cpp ./include/kukadu.h
	g++ -w -std=c++0x -pthread -DOSNAME_LINUX=1 ./src/mainScrew.cpp -o ./bin/screw $(GSLINCS) $(GSLLIBS) -larmadillo -lkukadu
screw:	./bin/screw
screwExe:	screw
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/screw
hand:	main
	LD_LIBRARY_PATH=/usr/local/lib:./lib ./bin/dmp -h
