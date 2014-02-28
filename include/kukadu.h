#ifndef KUKADU
#define KUKADU

#include "../src/trajectory/DMPGeneralizer.h"
#include "../src/learning/GaussianKernel.h"
#include "../src/learning/GaussianProcessRegressor.h"
#include "../src/learning/GeneralFitter.h"
#include "../src/learning/GenericKernel.h"
#include "../src/learning/KernelRegressor.h"
#include "../src/learning/LWRRegressor.h"
#include "../src/learning/QuadraticKernel.h"
#include "../src/learning/TricubeKernel.h"
#include "../src/learning/metric_learning/Mahalanobis.h"
#include "../src/learning/metric_learning/MahalanobisLearner.h"
#include "../src/learning/metric_learning/InfTheoMetricLearner.h"
#include "../src/learning/metric_learning/InfTheoConstraints.h"
#include "../src/learning/metric_learning/TogersonMetricLearner.h"
#include "../src/utils/CustomSet.h"
#include "../src/robot/mounted/GenericHand.h"
#include "../src/robot/mounted/SchunkHand.h"
#include "../src/robot/ControlQueue.h"
#include "../src/robot/KukaControlQueue.h"
#include "../src/trajectory/DMPExecutor.h"
#include "../src/trajectory/DMPTrajectoryGenerator.h"
#include "../src/trajectory/PolyTrajectoryGenerator.h"
#include "../src/trajectory/TrajectoryDMPLearner.h"
#include "../src/trajectory/TrajectoryGenerator.h"
#include "../src/trajectory/TrajectoryExecutor.h"
#include "../src/learning/reinforcement_learning/CostComputer.h"
#include "../src/learning/reinforcement_learning/PoWER.h"
#include "../src/learning/reinforcement_learning/DMPReinforcer.h"
#include "../src/learning/reinforcement_learning/GenDMPReinforcer.h"
#include "../src/learning/reinforcement_learning/SampleRewardComputer.h"
#include "../src/learning/reinforcement_learning/GaussianObstacleRewardComputer.h"
#include "../src/learning/reinforcement_learning/TrajectoryBasedReward.h"
#include "../src/trajectory/DictionaryGeneralizer.h"
#include "../src/trajectory/TrajectoryComparator.h"
#include "../src/trajectory/DMPTrajectoryComparator.h"
#include "../src/types/Trajectory.h"
#include "../src/types/SingleSampleTrajectory.h"
#include "../src/types/DMP.h"
#include "../src/types/LinCombDmp.h"
#include "../src/types/DictionaryTrajectory.h"
#include "../src/types/QueryPoint.h"
#include "../src/types/DMPBase.h"
#include "../src/utils/DestroyableObject.h"
#include "../src/utils/Tokenizer.h"
#include "../src/utils/types.h"
#include "../src/utils/utils.h"
#include "../src/utils/conversion_utils.h"

#ifdef ROS_SUPPORT
#include "../src/robot/OrocosControlQueue.h"
#endif

#endif