#include "DSA_loop_functions.h"

DSA_loop_functions::DSA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
    //MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick()),
    ResourceDensityDelay(0),
    //RandomSeed(GetSimulator().GetRandomSeed()),
    SimCounter(0),
    MaxSimCounter(1),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(4),
    DrawIDs(1),
//    DrawIDs(0),
    DrawTrails(0),
    DrawTargetRays(0),
    FoodDistribution(1),
//    FoodDistribution(9),
    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),
//    NestRadius(0.25),
    NestRadius(0.5),
//    NestRadiusSquared(0.0625),
    NestRadiusSquared(0.25),
    NestElevation(0.01),
    SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
    ticks_per_second(0),
    sim_time(0),
    score(0),
    PrintFinalScore(0),
    SimulatorTicksperSec(0),
    m_pcRNG(NULL)
{}

void DSA_loop_functions::Init(TConfigurationNode& node) {
CSimulator     *simulator     = &GetSimulator();
  CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("default");
  ticks_per_second = physicsEngine->GetInverseSimulationClockTick();
 argos::TConfigurationNode DDSA_node = argos::GetNode(node, "DDSA");
 argos::GetNodeAttribute(DDSA_node, "PrintFinalScore",                   PrintFinalScore);
 argos::GetNodeAttribute(DDSA_node, "FoodDistribution",                  FoodDistribution);
 argos::GetNodeAttribute(DDSA_node, "FoodItemCount",                  FoodItemCount);
 argos::GetNodeAttribute(DDSA_node, "NestRadius",                 NestRadius);
 argos::GetNodeAttribute(DDSA_node, "FoodBoundsWidth",                 FoodBoundsWidth);
 argos::GetNodeAttribute(DDSA_node, "FoodBoundsHeight",                 FoodBoundsHeight);

 NestRadiusSquared = NestRadius*NestRadius;

    // calculate the forage range and compensate for the robot's radius of 0.085m
    argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
    
    argos::Real rangeX1 = (ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY1 = (ArenaSize.GetY() / 2.0) - 0.085;
    ForageRangeX_1.Set(-rangeX1, rangeX1);
    ForageRangeY_1.Set(-rangeY1, rangeY1);
  
    argos::Real rangeX = FoodBoundsWidth/2.0;//(ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = FoodBoundsHeight/2.0;//(ArenaSize.GetY() / 2.0) - 0.085;  
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    SimulatorTicksperSec = GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();
    
    CSpace::TMapPerType::iterator it;

	for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        DSA_controller& c2 = dynamic_cast<DSA_controller&>(c);

        c2.SetLoopFunctions(this);
	}
//    CRandom::CreateCategory("testing", 12345);
//    /* Create a new RNG */
//    m_pcRNG = argos::CRandom::CreateRNG("testing");

	SetFoodDistribution();
//    SimulatorTicksperSec = 32;
    stoptime1 = 0;
    stoptime2 = 0;
    RobotReachedWayPoint = 0;
    FirstCheck = 0;
    Result_Checked = 1;
}


double DSA_loop_functions::Score()
{  
  return score;
}


void DSA_loop_functions::setScore(double s)
{
  score = s;
  if (score >= FoodItemCount) 
    {
      PostExperiment();
      exit(0);
    }
}

void DSA_loop_functions::PostExperiment() 
{
  if (PrintFinalScore == 1) printf("%f, %f\n", getSimTimeInSeconds(), score);
}


void DSA_loop_functions::PreStep() 
{
    argos::UInt16 counter = 0;
    argos::Real RobotCourseAngle = 0;
    
    /* reset the RobotReachedWayPoint flag */
    RobotReachedWayPoint = 0;
    
    /* reset the RobotReachedWayPoint flag */
    NewWayPointAdded = 0;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    //Reset the Result_Checked variable
    Result_Checked = 1;
    
    
    /* Get the hadndle to each robot and check if any one waypoint reached*/
    for(CSpace::TMapPerType::iterator it2 = m_cFootbots.begin();
        it2 != m_cFootbots.end();
        ++it2)
    {
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(it2->second);
        BaseController& cController2 = dynamic_cast<BaseController&>(cFootBot2.GetControllableEntity().GetController());
        
        BaseController::RobotData& stRobotDataCopy = cController2.GetRobotData();
//        if(stRobotDataCopy.Waypoint_Added == true)
//        {
//            stRobotDataStruct.push_back(stRobotDataCopy);
//        }
        
        NewWayPointAdded |= stRobotDataCopy.Waypoint_Added;
        
        if(FirstCheck == 0)
        {
            Result_Checked &= stRobotDataCopy.Checked;
        }
        
    }
    FirstCheck = Result_Checked;
    
    /* check collinearity and intersection if target reached or new waypoint added or its the start of code */
    if(FirstCheck == 1 and  NewWayPointAdded == 1)
    {
        for(CSpace::TMapPerType::iterator it4 = m_cFootbots.begin();
            it4 != m_cFootbots.end();
            ++it4)
        {
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBot4 = *any_cast<CFootBotEntity*>(it4->second);
            BaseController& cController4 = dynamic_cast<BaseController&>(cFootBot4.GetControllableEntity().GetController());
            
            cController4.SetHardStopMovement();
            cController4.ResetIntersectionData();
        }
        
        /* Get the hadndle to each robot */
        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
            it != m_cFootbots.end();
            ++it)
        {
            
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
            BaseController& cController = dynamic_cast<BaseController&>(cFootBot.GetControllableEntity().GetController());
            
            BaseController::RobotData& sRobotDataprevious = cController.GetRobotData();
            BaseController::IntersectionData& sIntersectionDataprevious = cController.GetIntersectionData();
            
            
            /* Get the hadndle to each next robot */
            for(CSpace::TMapPerType::iterator it1 = std::next(it, 1);
                it1 != m_cFootbots.end();
                ++it1)
                
            {
                CFootBotEntity& cFootBot1 = *any_cast<CFootBotEntity*>(it1->second);
                BaseController& cController1 = dynamic_cast<BaseController&>(cFootBot1.GetControllableEntity().GetController());
                
                BaseController::RobotData& sRobotDatanext = cController1.GetRobotData();
                BaseController::IntersectionData& sIntersectionDatanext = cController1.GetIntersectionData();
                
                cController.SetHardStopMovement();
                cController1.SetHardStopMovement();
                
                sRobotDataprevious.StartWaypoint = cController.GetPosition();
                sRobotDatanext.StartWaypoint = cController1.GetPosition();
                
                // Reset the collinearity flag
                sRobotDataprevious.CollinearFlag = false;
                sRobotDatanext.CollinearFlag = false;
                
                /* check if robot's end waypoint is collinear in other robot's start and end waypoint */
                CheckCollinearity(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
                
                if(sRobotDataprevious.CollinearFlag == 1 and !sRobotDatanext.WaypointStack.empty())
                {
                    sRobotDatanext.AddedPoint = sRobotDatanext.WaypointStack.top();
                    
                    cController1.SetTarget(sRobotDatanext.AddedPoint);
                    sRobotDatanext.WaypointStackpopped = true;
                    sRobotDatanext.TargetWaypoint = sRobotDatanext.AddedPoint;
                    cController1.SetStopMovement();
                    sRobotDatanext.WaypointStack.pop();
                    
//                    if(sRobotDatanext.StopTurningTime > 0)
//                    {
//                        sRobotDatanext.StopTurningTime = 0;
//                    }
                    
                }
                
                if(sRobotDataprevious.CollinearFlag !=1)
                {
                    if(sRobotDataprevious.WaypointStackpopped == true or sRobotDataprevious.GoingToOrFromNest == true)
                    {
                        
                        GetPointAtSafeDistance(sRobotDatanext);
                        Find_Intersection(sRobotDataprevious.StartWaypoint, sRobotDataprevious.TargetWaypoint,
                                          sRobotDatanext.StartWaypoint, PointChangeDirection,sIntersectionDatanext);
                        if(sIntersectionDatanext.Intersection_flag == 1)
                        {
    
                            IntersectionCollisionCheck(sRobotDataprevious.StartWaypoint, sRobotDatanext.StartWaypoint,sRobotDataprevious,
                                                       sRobotDatanext,sIntersectionDatanext,1);
                            cController.SetStopMovement();
                            cController1.SetStopMovement();
                        }
                        else
                        {
                            Find_Intersection(sRobotDataprevious.StartWaypoint, sRobotDataprevious.TargetWaypoint,
                                              PointChangeDirection, PointAtSafeDistance,sIntersectionDatanext);
    
                            if(sIntersectionDatanext.Intersection_flag == 1)
                            {
    
                                IntersectionCollisionCheck(sRobotDataprevious.StartWaypoint, PointChangeDirection,sRobotDataprevious,
                                                           sRobotDatanext,sIntersectionDatanext,2);
                                cController.SetStopMovement();
                                cController1.SetStopMovement();
                            }
                        }
                        
                        
                    }
                    else if(sRobotDatanext.WaypointStackpopped == true or sRobotDatanext.GoingToOrFromNest == true)
                    {
                        
                        GetPointAtSafeDistance(sRobotDataprevious);
                        Find_Intersection(sRobotDataprevious.StartWaypoint, PointChangeDirection,
                                          sRobotDatanext.StartWaypoint, sRobotDatanext.TargetWaypoint,sIntersectionDatanext);
                        if(sIntersectionDatanext.Intersection_flag == 1)
                        {
                            
                            IntersectionCollisionCheck(sRobotDatanext.StartWaypoint, sRobotDataprevious.StartWaypoint,
                                                       sRobotDatanext, sRobotDataprevious,sIntersectionDatanext,1);
                            cController.SetStopMovement();
                            cController1.SetStopMovement();
                        }
                        else
                        {
                            Find_Intersection(PointChangeDirection, PointAtSafeDistance, sRobotDatanext.StartWaypoint,                      sRobotDatanext.TargetWaypoint, sIntersectionDatanext);
                            
                            if(sIntersectionDatanext.Intersection_flag == 1)
                            {
                                
                                IntersectionCollisionCheck(sRobotDatanext.StartWaypoint, PointChangeDirection,
                                                           sRobotDatanext,sRobotDataprevious, sIntersectionDatanext,2);
                                cController.SetStopMovement();
                                cController1.SetStopMovement();
                            }
                        }
                        
                        
                    }
                    
                }

            
            } /* end of inner for loop */
        } /* end of outer for loop */
        for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
            it5 != m_cFootbots.end();
            ++it5)
        {
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
            BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
            BaseController::RobotData& stRobotData_1 = cController5.GetRobotData();
    
            stRobotData_1.pathcheck = true;
            stRobotData_1.WaypointStackpopped =  false;
            cController5.SetMovement();
        }

        
    }
    
    sim_time++;
}


/*****
 *
 *****/
argos::Real DSA_loop_functions::getSimTimeInSeconds()
{
  return sim_time/ticks_per_second;
}


/*****
 *
 *****/
void DSA_loop_functions::SetFoodDistribution() {
    switch(FoodDistribution) {
        case 0:
            RandomFoodDistribution();
            break;
        case 1:
            ClusterFoodDistribution();
            break;
        case 2:
            PowerLawFoodDistribution();
            break;
        default:
            argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
    }
}

/*****
 *
 *****/
void DSA_loop_functions::RandomFoodDistribution() {
    FoodList.clear();

    argos::CVector2 placementPosition;

    for(size_t i = 0; i < FoodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
            
        }

        FoodList.push_back(placementPosition);
        FoodColoringList.push_back(argos::CColor::BLACK);
    }
}

void DSA_loop_functions::ClusterFoodDistribution() {
    FoodList.clear();
    
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = FoodItemCount;//Wayne: Changed since no longer necessary
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

    FindClusterLengthWidth();//Wayne: sets cluster sides (X,Y)

    //-----Wayne: Creates vector of number of food in each cluster
    size_t index = 0;
    size_t ClusterFoodCount = 0;
    size_t foodCount = 0;
    vector <size_t> FoodClusterCount;
    
    //initialize vector
    for (int i = 0; i < NumberOfClusters; i++){
        FoodClusterCount.push_back(0);
    }
    
    //add food
    while (foodCount < FoodItemCount){
        FoodClusterCount[index] = FoodClusterCount[index]+ 1;
        foodCount++;
        index++;
        if (index == NumberOfClusters) index = 0;
        
    }
    
    //make vector cumulative in food
    for (int i = 1; i < NumberOfClusters; i++){
        FoodClusterCount[i] += FoodClusterCount[i - 1];
    }
    //------Wayne: end of vector creation
    
	for(size_t i = 0; i < NumberOfClusters; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        

		while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
            
		}
        
        /*Wayne: Modified to break from loops if food count reached.
         Provides support for unequal clusters and odd food numbers.
         Necessary for DustUp and Jumble Distribution changes. */
        
		for(size_t j = 0; j < ClusterLengthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
                if (foodPlaced == FoodClusterCount[i]) break;
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
            if (foodPlaced == FoodClusterCount[i]) break;
		}
        if (foodPlaced == FoodItemCount) break;
	}
}

void DSA_loop_functions::PowerLawFoodDistribution() {
	FoodList.clear();
    
    argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;
    
    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}

    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
                placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
		}
	}
    
}


//Wayne: makes as square a cluster as it can
void DSA_loop_functions::FindClusterLengthWidth(){
    
    size_t tempClusterAreaCount = 0;
    size_t tempFoodItemCount =  FoodItemCount;
    
    while (tempFoodItemCount % NumberOfClusters != 0){
        tempFoodItemCount++;
    }
    
    //Find number of seeds in cluster
    size_t ClusterAreaCount = tempFoodItemCount / NumberOfClusters;
    
    //Find square root (max for both sides)
    size_t x =  sqrt(ClusterAreaCount);
    
    if (ClusterAreaCount % x != 0 || (x == 1 && FoodItemCount > NumberOfClusters)){
        ClusterLengthY = x + 1;
        ClusterWidthX = x + 1;
    }
    else {
        ClusterWidthX = x;
        ClusterLengthY = x;
    }
}


/*****
 *
 *****/
bool DSA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
    argos::CVector2 placementPosition = p;

    argos::Real foodOffset   = 3.0 * FoodRadius;
    argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
    argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

    argos::Real x_min = p.GetX() - FoodRadius;
    argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

    argos::Real y_min = p.GetY() - FoodRadius;
    argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

    if((x_min < (ForageRangeX.GetMin() + FoodRadius)) ||
       (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
       (y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
       (y_max > (ForageRangeY.GetMax() - FoodRadius))) {
        return true;
    }

    for(size_t j = 0; j < length; j++) {
        for(size_t k = 0; k < width; k++) {
            if(IsCollidingWithFood(placementPosition)) return true;
            if(IsCollidingWithNest(placementPosition)) return true;
            placementPosition.SetX(placementPosition.GetX() + foodOffset);
        }

        placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
        placementPosition.SetY(placementPosition.GetY() + foodOffset);
    }

    return false;
}

/*****
 *
 *****/
bool DSA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
    argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
    argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - NestPosition).SquareLength() < NRPB_squared);
}

/*****
 *
 *****/
bool DSA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
    argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
    argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < FoodList.size(); i++) {
        if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}


/****************************************************************************************************************/
/* Function to calculate intersection points of robot paths */
/****************************************************************************************************************/
void DSA_loop_functions::Check_GoingToNest(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
{
    if(ptr1.TargetWaypoint == ptr2.TargetWaypoint)
    {
        ptr2.StopTurningTime = (ptr2.id_robot);
        update_movement_state = true;
    }
}

/****************************************************************************************************************/
/* Function to calculate intersection points of robot paths */
/****************************************************************************************************************/
//void DSA_loop_functions::Find_Intersection(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                             BaseController::IntersectionData &ptr3){
//
//    argos::Real x_inter;
//    argos::Real y_inter;
//    argos::Real A1, A2, B1, B2, C1, C2, det;
//
//    /* get the start and target position of robot 1 */
//    argos::CVector2 StartPosition_Robot1 = ptr1.StartWaypoint;
//    argos::CVector2 TargetPosition_Robot1 = ptr1.TargetWaypoint;
//
//    /* get the start and target position of robot 2 */
//    argos::CVector2 StartPosition_Robot2 = ptr2.StartWaypoint;
//    argos::CVector2 TargetPosition_Robot2 = ptr2.TargetWaypoint;
//
//    /*A1 = Robot1_goal_y - Robot1_start_y*/
//    A1 = TargetPosition_Robot1.GetY() - StartPosition_Robot1.GetY();
//
//
//
//    /*B1 = Robot1_start_x - Robot1_goal_x*/
//    B1 = StartPosition_Robot1.GetX() - TargetPosition_Robot1.GetX();
//
//
//
//    /* C1 = A1 * Robot1_start_x + B1 * Robot1_start_y */
//    C1 = A1 * StartPosition_Robot1.GetX() + B1 * StartPosition_Robot1.GetY();
//
//
//    /*A2 = Robot2_goal_y - Robot2_start_y*/
//    A2 = TargetPosition_Robot2.GetY() - StartPosition_Robot2.GetY();
//
//    /*B2 = Robot2_start_x - Robot2_goal_x*/
//    B2 = StartPosition_Robot2.GetX() - TargetPosition_Robot2.GetX();
//
//    /* C2 = A2 * Robot2_start_x + B2 * Robot2_start_y */
//    C2 = A2 * StartPosition_Robot2.GetX() + B2 * StartPosition_Robot2.GetY();
//
//    det = A1*B2 - A2*B1;
//
//    if(det == 0)
//    {
//        /* Lines are parallel */
//        ptr3.Intersection_flag= 0;
//    }
//
//    /* Lines intersect and find the intersection point */
//    else{
//        x_inter = (B2*C1 - B1*C2)/det;
//
//        y_inter = (A1*C2 - A2*C1)/det;
//
//        /* Check if intersection point is out of bound for the line segment */
//        if((x_inter < std::max(std::min(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
//                               std::min(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))) or
//           (x_inter > std::min(std::max(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
//                               std::max(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))))
//        {
//            ptr3.Intersection_flag = 0;
//        }
//        else if((y_inter < std::max(std::min(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
//                                    std::min(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))) or
//                (y_inter > std::min(std::max(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
//                                    std::max(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))))
//        {
//            ptr3.Intersection_flag = 0;
//        }
//        else
//        {
//            ptr3.Intersection_flag = 1;
//            ptr3.IntersectionPoint.Set(x_inter, y_inter);
//            ptr3.Robot_ID_Intersectingwith = ptr1.id_robot;
//        }
//    }
//}

/********************************************************************************************/
/* Function to calculate distance to Target*/
/********************************************************************************************/
argos::Real DSA_loop_functions::CalculateDistance(argos::CVector2 cPosition1, argos::CVector2 cPosition2){
    argos::Real dist_y, dist_x, distance;
    
    dist_x = cPosition2.GetX() - cPosition1.GetX();
    dist_y = cPosition2.GetY() - cPosition1.GetY();
    
    distance = sqrt((dist_y * dist_y)+(dist_x * dist_x));
    
    return distance;
}

/********************************************************************************************/
/* Function to calculate distance to Target*/
/********************************************************************************************/
argos::UInt16 DSA_loop_functions::GetTimeToTurn(argos:: Real newAngleToTurnInDegrees, argos::Real speed)
{
    // s = arc_length = robot_radius * turning_angle
    // NOTE: the footbot robot has a radius of 0.085 m... or 8.5 cm...
    // adjusting with + 0.02 m, or + 2 cm, increases accuracy...
    argos::Real TicksToWait;
    argos::Real s = 0.105 * newAngleToTurnInDegrees;
    TicksToWait = std::ceil((SimulatorTicksperSec * s) / speed);
    return TicksToWait;
}



/**************************************************************************************************************************/
/* Function to calculate time in terms of ticks */
/**************************************************************************************************************************/
argos::UInt16 DSA_loop_functions::GetTicksToWait(argos::Real dist, argos::Real speed)
{
    argos::UInt16 wait_ticks = std::ceil((abs(dist) * SimulatorTicksperSec) / speed);
    
    return wait_ticks;
}


/**************************************************************************************************************************/
/* Function to calculate time required by robot to reach intersection point */
/**************************************************************************************************************************/
//void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                                      BaseController::IntersectionData &ptr3){
//
//    argos::UInt16 TicksToWait_Robot1, TicksToWait_Robot2, TicksToWaitforSafedistance, TimeToIntersection, TimeDiff;
//    argos::Real DistanceToIntersection_Robot1, DistanceToIntersection_Robot2, IntersectionDistance;
//    argos::Real AdjustedVelocity;
//    bool Intersection_flag;
//
//    ptr1.Intersection_Adjustment = 0;
//    ptr2.Intersection_Adjustment = 0;
//
//    /* Get the distance between start point and intersection point */
//    DistanceToIntersection_Robot1 = CalculateDistance(ptr1.StartWaypoint, ptr3.IntersectionPoint);
//    DistanceToIntersection_Robot2 = CalculateDistance(ptr2.StartWaypoint, ptr3.IntersectionPoint);
//
//    /* calculate the time required to reach the intersection point */
////    TicksToWait_Robot1 = GetTicksToWait(DistanceToIntersection_Robot1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime;
//    TicksToWait_Robot1 = CalculateTotalTime(ptr1, ptr3.IntersectionPoint, ptr1.StartWaypoint);
//    TicksToWait_Robot2 = CalculateTotalTime(ptr2, ptr3.IntersectionPoint, ptr2.StartWaypoint);
////    TicksToWait_Robot2 = GetTicksToWait(DistanceToIntersection_Robot2, ptr2.fLinearWheelSpeed)+ ptr2.StopTurningTime;
//
//
//    TimeDiff = abs(TicksToWait_Robot1 - TicksToWait_Robot2);
//
//    TicksToWaitforSafedistance = GetTicksToWait(Safedistance , MaxLinearSpeed);
//
//
//    //if the difference between the times is equal to safe distance time between two robots
//    if(TimeDiff <= TicksToWaitforSafedistance)
//    {
//
//        /* there is a chance of collision */
//        /* slow down the velocity of robot 2 as its priority is lower */
//
//        if(DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2)
//        {
//            IntersectionDistance = DistanceToIntersection_Robot2;
//            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = MinLinearSpeed;
//                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
//                Real stop_time = abs(Time - TimeToIntersection);
//                ptr2.StopTurningTime += stop_time;
//            }
//            ptr2.fLinearWheelSpeed = AdjustedVelocity;
//            ptr2.Intersection_Adjustment = 1;
//
//        }
//        /* slow down the velocity of robot 1 as its priority is lower */
//        else if(DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1)
//        {
//            IntersectionDistance = DistanceToIntersection_Robot1;
//            TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = MinLinearSpeed;
//                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
//                Real stop_time = abs(Time - TimeToIntersection);
//                ptr1.StopTurningTime += stop_time;
//            }
//            ptr1.fLinearWheelSpeed = AdjustedVelocity;
//            ptr1.Intersection_Adjustment = 1;
//        }
//        else{
//            if(ptr1.GoingToNest)
//            {
//                IntersectionDistance = DistanceToIntersection_Robot2;
//                TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/ TimeToIntersection) * SimulatorTicksperSec;
//
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = MinLinearSpeed;
//                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
//                    Real stop_time = abs(Time - TimeToIntersection);
//                    ptr2.StopTurningTime += stop_time;
//                }
//                ptr2.fLinearWheelSpeed = AdjustedVelocity;
//                ptr2.Intersection_Adjustment = 1;
//            }
//            else{
//                IntersectionDistance = DistanceToIntersection_Robot1;
//                TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = MinLinearSpeed;
//                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
//                    Real stop_time = abs(Time - TimeToIntersection);
//                    ptr1.StopTurningTime += stop_time;
//                }
//                ptr1.fLinearWheelSpeed = AdjustedVelocity;
//                ptr1.Intersection_Adjustment = 1;
//            }
//        }
//
//    }
//
//    /* Reset the flag */
//    ptr3.Intersection_flag = 0;
//
//}

/*****************************************************************************************************************/
/* Function to calculate time to reach target */
/*****************************************************************************************************************/
argos::UInt16 DSA_loop_functions::CalculateTotalTime(BaseController::RobotData& ptr, argos::CVector2 TargetPoint, argos::CVector2 StartPoint)
{
    // get the linear distance
    argos::Real dist_linear = CalculateDistance(StartPoint, TargetPoint);
    
    // get the time for the linear distance
    argos::UInt16 time_linear = GetTicksToWait(dist_linear , ptr.fLinearWheelSpeed);
    
    // get the heading angle
    argos::CRadians headingToTarget = (TargetPoint - StartPoint).Angle();
    argos::Real headingangle = ToDegrees(headingToTarget).GetValue();
    
    // get the time for the turning towards target
    argos::UInt16 time_turn = GetTimeToTurn(headingangle, ptr.fBaseAngularWheelSpeed);
    
    // total time  to reach target
    argos::UInt16 total_time = time_linear + time_turn + ptr.StopTurningTime;
    
    return total_time;
}


/*****************************************************************************************************************/
/* Function to adjust velocity for  multiple robots going to nest at the same time */
/*****************************************************************************************************************/
void DSA_loop_functions::AdjustTimeToReachNest(BaseController::RobotData& ptr1, BaseController::RobotData& ptr2)
{

    argos::UInt16 time_Robot1, time_Robot2, time_safe_dist, time_turn_safe_dist;
    
    argos::Real dist1, dist2;
    
    
    dist1 = CalculateDistance(ptr1.StartWaypoint, NestPosition);
    dist2 = CalculateDistance(ptr2.StartWaypoint, NestPosition);
    
    // calculate time for robot 1 to reach nest
    time_Robot1 = CalculateTotalTime(ptr1,  ptr1.TargetWaypoint, ptr1.StartWaypoint);
    
    // calculate time for robot 2 to reach nest
    time_Robot2 = CalculateTotalTime(ptr2,  ptr2.TargetWaypoint, ptr2.StartWaypoint);

    
    // check if robot 2 is already in nest
//    if(dist2 < 0.40 )
    
    if(ptr2.GoingToNest == true && dist2 < 0.35 && dist1 > NestRadiusSquared)
    {
        //calculate time for robot 1 to go out of nest for a safe distance
        time_safe_dist = GetTicksToWait(0.5 , ptr2.fLinearWheelSpeed);
        
        //calculate time for robot 1 to turn to get out of nest
        time_turn_safe_dist = GetTimeToTurn(180, ptr2.fBaseAngularWheelSpeed);
        
        // add stop time for the lower priority robot if the time difference is not sufficient
        if(abs(time_Robot1 - time_Robot2) <= abs(time_safe_dist + time_turn_safe_dist))
        {
            ptr1.CollinearFlag = 1;
            if((time_safe_dist + time_turn_safe_dist) <= 50)
            {
                ptr1.StopTurningTime += 100;
            }
            else
            {
                ptr1.StopTurningTime += ((time_safe_dist + time_turn_safe_dist));
            }
        }
    }
    else
    {

        //calculate time for robot 1 to go out of nest for a safe distance
        time_safe_dist = GetTicksToWait(0.5 , ptr1.fLinearWheelSpeed);

        //calculate time for robot 1 to turn to get out of nest
        time_turn_safe_dist = GetTimeToTurn(180, ptr1.fBaseAngularWheelSpeed);

        // add stop time for the lower priority robot if the time difference is not sufficient
        if(abs(time_Robot1 - time_Robot2) <= abs(time_safe_dist + time_turn_safe_dist))
        {
            ptr1.CollinearFlag = 1;
            if((time_safe_dist + time_turn_safe_dist) <= 50)
            {
                ptr2.StopTurningTime += 100;
            }
            else
            {
                ptr2.StopTurningTime += ((time_safe_dist + time_turn_safe_dist));
                
            }
        }
    }

            
}

/*****************************************************************************************************************/
/* Function to add a new way point */
/*****************************************************************************************************************/
//void DSA_loop_functions::AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                        argos::UInt8 ptrIndex)

//void DSA_loop_functions::AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
//
//{
//    argos::CVector2 AddedWaypoint;
//    argos::Real diff, x, y;
//
//    argos::CVector2 du;
//
//    argos::Real Factor = 0.3;
//    x = 0;
//    y= 0;
//    diff = 0;

    
    
    
//    x = ptr2.StartWaypoint.GetX()
    
//    if(ptr1.StartWaypoint.GetX() > ptr2.StartWaypoint.GetX())
//    {
//        du = (-Factor * ptr2.NormailzedVector) + ptr2.StartWaypoint;
//    }
//    else
//    {
//       du = (Factor * ptr2.NormailzedVector) + ptr2.StartWaypoint;
//    }
//
//    if(du.GetX() < (ForageRangeX_1.GetMin() + Factor))
//    {
//        diff = abs((ForageRangeX_1.GetMin() + Factor) - ptr2.StartWaypoint.GetX());
//        x = ptr2.StartWaypoint.GetX() - diff;
//        du.SetX(x);
//    }
//    else if(du.GetX() > (ForageRangeX_1.GetMax() - Factor))
//    {
//
//        diff = abs(ptr2.StartWaypoint.GetX() - (ForageRangeX_1.GetMax() - Factor));
//        x = ptr2.StartWaypoint.GetX() + diff;
//        du.SetX(x);
//    }
//    if(du.GetY() < (ForageRangeY_1.GetMin() + Factor))
//    {
//        diff = abs((ForageRangeX_1.GetMin() + Factor) - ptr2.StartWaypoint.GetY());
//        y = ptr2.StartWaypoint.GetY() - diff;
//        du.SetY(y);
//    }
//    else if(du.GetY() > (ForageRangeY_1.GetMax() - Factor))
//    {
//
//        diff = abs(ptr2.StartWaypoint.GetY() - (ForageRangeY_1.GetMax() - Factor));
//        y = ptr2.StartWaypoint.GetY() + diff;
//        du.SetY(y);
//    }
//
//    AddedWaypoint = du;
    
//    if(ptr1.StartWaypoint.GetX() >= ptr2.StartWaypoint.GetX())
//    {
//        x = ptr2.StartWaypoint.GetX() - Factor;
//    }
//    else
//    {
//         x = ptr2.StartWaypoint.GetX() + Factor;
//    }
//    if(ptr1.StartWaypoint.GetY() >= ptr2.StartWaypoint.GetY())
//    {
//        y = ptr2.StartWaypoint.GetY() - Factor;
//    }
//    else
//    {
//        y = ptr2.StartWaypoint.GetY() + Factor;
//    }
//
//    if(x < (ForageRangeX_1.GetMin() + Factor))
//    {
//        diff = abs((ForageRangeX_1.GetMin() + Factor) - ptr2.StartWaypoint.GetX());
//        x = ptr2.StartWaypoint.GetX() - diff;
//    }
//    else if(x > (ForageRangeX_1.GetMax() - Factor))
//    {
//
//        diff = abs(ptr2.StartWaypoint.GetX() - (ForageRangeX_1.GetMax() - Factor));
//        x = ptr2.StartWaypoint.GetX() + diff;
//    }
//
//    if(y < (ForageRangeY_1.GetMin() + Factor))
//    {
//        diff = abs((ForageRangeY_1.GetMin() + Factor) - ptr2.StartWaypoint.GetY());
//        y = ptr2.StartWaypoint.GetY() - diff;
//    }
//    else if(y > (ForageRangeY_1.GetMax() - Factor))
//    {
//
//        diff = abs(ptr2.StartWaypoint.GetY() - (ForageRangeY_1.GetMax() - Factor));
//        y = ptr2.StartWaypoint.GetY() + diff;
//    }
//
//
//    AddedWaypoint.Set(x, y);
//    diff= 0;
//    x=0;
//    y=0;
//
//
//
//
//    if(ptr2.WaypointCounter <=10)
//    {
//        ptr1.CollinearFlag = 1;
//        if(ptr2.WaypointStack.empty())
//        {
//            ptr2.WaypointStack.push(ptr2.TargetWaypoint);
//        }
//
//        /* add a way point before the final goal */
//        ptr2.WaypointStack.push(AddedWaypoint);
////        ptr1.StopTurningTime += 20;
//        ptr2.WaypointCounter++;
//    }
//
    
    


//}

/*****************************************************************************************************************/
/* Function to check if robot added waypoints don't intersect */
/*****************************************************************************************************************/
void DSA_loop_functions::CalculateWaitTime(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
{

    argos:: Real dist1, dist2;
//    argos::UInt16 stoptime1, stoptime2;
    dist1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
    dist2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);

    argos::UInt16 ticks1 = GetTicksToWait(dist1 , ptr1.fLinearWheelSpeed);
    argos::UInt16 ticks2 = GetTicksToWait(dist2 , ptr2.fLinearWheelSpeed);
    argos::UInt16 ticks_safedist = GetTicksToWait(0.3 , ptr1.fLinearWheelSpeed);
    
    
    stoptime1 = 0;
    stoptime2 = 0;

    if(ptr1.GoingToNest == true and ptr2.GoingToNest == false)
    {
//        if(dist1 <= dist2)
//        {
//            if(abs((ticks1) - ticks2) < ticks_safedist)
//            {
//                ptr2.StopTurningTime = ticks2 + (ticks_safedist) + abs((2*ticks1) - ticks2);
//                ptr1.StopTurningTime = 0;
//            }
//        }
//        else{
            if(abs((ticks1) - ticks2) <= ticks_safedist)
            {
                stoptime1 = ticks1 + (ticks_safedist) + abs((2*ticks2)) + 10;
                stoptime2 = 0;
//                ptr1.StopTurningTime = 4;
////                ptr1.StopTurningTime = ticks1 + (ticks_safedist) + abs((2*ticks2) - ticks1);
//                ptr2.StopTurningTime = 0;
            }
//        }
    }
    
    else if(ptr2.GoingToNest == true and ptr1.GoingToNest == false)
    {
//        if(dist2 <= dist1 or ptr1.GoingToNes)
//        {
            if(abs((ticks2) - ticks1) <= ticks_safedist)
            {
//                ptr1.StopTurningTime = ticks1 + (ticks_safedist) + abs((2*ticks2) - ticks1);
//                ptr1.StopTurningTime = 4;
//                ptr2.StopTurningTime = 0;
                stoptime1 = 0;
                stoptime2 = ticks2 + (ticks_safedist) + abs((2*ticks1)) + 10;
          
            }
//        }
    }
    
    else{
        if(dist1 <= dist2)
        {
            if(abs((ticks1) - ticks2) <= ticks_safedist)
            {
                stoptime1 = 0;
                stoptime2 = ticks2 + (ticks_safedist) + abs((2*ticks1)) + 10;
               
//                ptr2.StopTurningTime = 4;
////                ptr2.StopTurningTime = ticks2 + (ticks_safedist) + abs((2*ticks1) - ticks2);
//                ptr1.StopTurningTime = 0;
            }
        }
        else{
            if(abs((ticks2) - ticks1) <= ticks_safedist)
            {
                stoptime1 = ticks1 + (ticks_safedist) + abs((2*ticks2)) + 10;
                stoptime2 = 0;
//                ptr1.StopTurningTime = 4;
//                ptr1.StopTurningTime = ticks1 + (ticks_safedist) + abs((2*ticks2) - ticks1);
//                ptr2.StopTurningTime = 0;
            }
        }
    }
   
//    ptr1.StopTurningTime = stoptime1;
//    ptr2.StopTurningTime = stoptime2;
}
/*****************************************************************************************************************/
/* Function to calculate angle between two vectors */
/*****************************************************************************************************************/
argos::Real DSA_loop_functions::CalculateAngleBetweenVectors(CVector3 v1, CVector3 v2)
{
    argos::Real DegreeAngle, v1length, v2length, dot_product;
    
    v1length = v1.Length();
    v2length = v2.Length();
    
    dot_product = v1.DotProduct(v2);
    // calculate the angle between vectors
    // cos(angle) = (a.b/(|a|.|b|))
    DegreeAngle = ToDegrees(ACos(Real(dot_product/(v1length * v2length)))).GetValue();
    
    return DegreeAngle;
}

/*****************************************************************************************************************/
/* Function to represent vector when given two points */
/*****************************************************************************************************************/
argos::CVector3 DSA_loop_functions::GetVectorFromTwoPoints(argos::CVector2 StartPoint, argos::CVector2 EndPoint)
{
    argos::CVector3 v;
    v.SetX((EndPoint.GetX() - StartPoint.GetX()));
    v.SetY((EndPoint.GetY() - StartPoint.GetY()));
    v.SetZ(0);
    
    return v;
}

/*************************************************************************************************************************/
/* Function to find the shortest distance between two segments */
/************************************************************************************************************************/
argos:: Real DSA_loop_functions::ShortestDistTwoVectors(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
{
    argos::UInt8 i;
    argos:: Real shortest_dist, min;
    shortest_dist = 100;
    min = 100;
    i = 0;
    
    argos:: Real distance[4] = {0, 0, 0, 0};
    /* In 2d the shortest of the distance between point A and line segment CD, B and CD, C and AB or D and AB.
     So it's a fairly simple "distance between point and line" calculation (if the distances are all the same,
     then the lines are parallel. */
    
    // dist between robot 1 start, goal and robot2 start
    distance[0] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.StartWaypoint);
    //    distance[0] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, ptr2.StartWaypoint);
    
    // dist between robot 1 start, goal and robot2 goal
    distance[1] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.TargetWaypoint);
    //    distance[1] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, TargetPoint2);
    
    // dist between robot 2 start, goal and robot1 start
    distance[2] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.StartWaypoint);
    //    distance[2] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, ptr1.StartWaypoint);
    
    // dist between robot 2 start, goal and robot2 goal
    distance[3] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.TargetWaypoint);
    //    distance[3] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, TargetPoint1);
    
    for(i = 0; i< 4; i++)
    {
        if(distance[i] < min)
        {
            min = distance[i];
        }
        shortest_dist = min;
    }
    
    return shortest_dist;
}



/*****************************************************************************************************************/
/* Function to check if robot start and target points are collinear */
/*****************************************************************************************************************/
void DSA_loop_functions::CheckCollinearity(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                           BaseController::IntersectionData &ptr3)
{
//    argos::CVector3 v1, v2, cross_product, AddedWaypoint, ZeroVector;
//    argos::Real v1length, v2length, dot_product, DegreeAngle, shortest_dist, CosTheta;
    argos::Real vec1length, vec2length,shortest_dist, CosTheta, dot_product;
    argos::CVector2 vec1, vec2;
    
    argos::UInt16 timetowait;
    
    
    
    vec1.Set((ptr1.TargetWaypoint.GetX() - ptr1.StartWaypoint.GetX()), (ptr1.TargetWaypoint.GetY() - ptr1.StartWaypoint.GetY()));
    vec2.Set((ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX()), (ptr2.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
   
    vec1length = vec1.Length();
    vec2length = vec2.Length();
    
    ptr1.vect1 = vec1;
    ptr2.vect2 = vec2;
    
    dot_product = vec1.DotProduct(vec2);
    
    CosTheta = (dot_product/(vec1length * vec2length));
    
    // get the shortest distance between two vectors
    shortest_dist = ShortestDistTwoVectors(ptr1, ptr2);
    
    ptr1.Priority = CosTheta;
    ptr2.Priority = CosTheta;
    // angle = 0 to 21 deg celsius or 180 to 158
    if(abs(CosTheta) <= 1 and abs(CosTheta) >= 0.93)
    {
        if(ptr1.GoingToOrFromNest == true and ptr2.GoingToOrFromNest == true)
        {
            if(shortest_dist <= Safedistance)
            {
                AddNewWayPoint(ptr1, ptr2);
            }
        }
    }
//    ZeroVector.Set(0,0,0);
//    cross_product.Set(0,0,0);
//    dot_product = 0;
    
    // express the robot 1 and robot 2 paths as vector
//    v1 = GetVectorFromTwoPoints(ptr1.StartWaypoint, ptr1.TargetWaypoint);
//    v2 = GetVectorFromTwoPoints(ptr2.StartWaypoint, ptr2.TargetWaypoint);
    
    
//    vec1.SetX((ptr1.TargetWaypoint.GetX() - ptr1.StartWaypoint.GetX()));
//    vec1.SetY((ptr1.TargetWaypoint.GetY() - ptr1.StartWaypoint.GetY()));
//
//
//    vec2.SetX((ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
//    vec2.SetY((ptr2.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
//
//
//
//    v1length = vec1.Length();
//    v2length = vec2.Length();
//
//    dot_product = vec1.DotProduct(vec2);
//
//    // calculate the angle between vectors
//    // cos(angle) = (a.b/(|a|.|b|))
//    DegreeAngle = ToDegrees(ACos(Real(dot_product/(v1length * v2length)))).GetValue();
//
//    // check if two vectors are collinear. 2 vectors are collinear if their cross product is zero vector (applies to 3d vectors)
////    cross_product = v1.CrossProduct(v2);
////
////    ptr1.CrossProduct = cross_product;
////    ptr2.CrossProduct = cross_product;
////
////    dot_product = vec1.DotProduct(vec2);
////
////    DegreeAngle = CalculateAngleBetweenVectors(v1, v2);
//    timetowait = GetTicksToWait(Safedistance , MaxLinearSpeed);
//
//    // get the shortest distance between two robot courses
//
//    shortest_dist = ShortestDistTwoVectors(ptr1, ptr2);
//
//    ptr1.vect1 = vec1;
//    ptr2.vect2 = vec2;
//    ptr1.Priority = DegreeAngle;
//    ptr2.Priority = DegreeAngle;
    
    // add way point if two vectors are collinear
//    if(cross_product == ZeroVector)
//    {
//         if(ptr2.GoingToOrFromNest == true and ptr1.GoingToOrFromNest == true)
//         {
//             AddNewWayPoint(ptr1, ptr2);
//         }
//         else
//         {
//             if(ptr1.GoingToOrFromNest == true)
//             {
//                 ptr1.StopTurningTime+= timetowait;
//             }
//             else
//             {
//                 ptr2.StopTurningTime+= timetowait;
//             }
//         }
//    }
    // if the lines are almost parallel
//    to know the angle between two vectors, I can use the dot product. This gives a value between -1 and 1, where
//    1 means the vectors are parallel and facing the same direction (the angle is 0 degrees).
//    -1 means they are parallel and facing opposite directions (still 180 degrees).
//    0 means the angle between them is 90 degrees.
    
//    else if(0 <= DegreeAngle and OverlappingCourseAngle <= DegreeAngle)
////    else if(abs(dot_product) <= 1 and abs(dot_product) >= 0.9)
//    {
//
//        if(ptr1.GoingToOrFromNest == true and ptr2.GoingToOrFromNest == true)
//        {
//            if(shortest_dist <= Safedistance)
//            {
////                AddNewWayPoint(ptr1, ptr2);
//            }
//        }
//    }


            
            
}

//void DSA_loop_functions::CheckCollinearity(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                           BaseController::IntersectionData &ptr3)
//{
//
//    argos::CVector2 AddedWaypoint, TargetPoint1, TargetPoint2;
//    argos::Real Angle, Distance, distBetweenTargets, distBetweenStartPoints, dist1, dist2;
//
//    ptr1.CollinearFlag = 0;
//    ptr2.CollinearFlag = 0;
//    ptr1.WaypointStackpopped = false;
//    ptr2.WaypointStackpopped = false;
//
//
//    if(ptr1.TargetWaypoint == NestPosition)
//    {
//        TargetPoint1 = CalculateTargePoint(ptr1);
//    }
//    else{
//        TargetPoint1 = ptr1.TargetWaypoint;
//    }
//
//    if(ptr2.TargetWaypoint == NestPosition)
//    {
//        TargetPoint2 = CalculateTargePoint(ptr2);
//    }
//    else{
//        TargetPoint2 = ptr2.TargetWaypoint;
//    }
//
//
//    CheckRobotHeadingCourse(ptr1, ptr2, ptr3);
//
//    if(ptr1.CollinearFlag!= 1)
//    {
//        if(ptr1.TargetWaypoint == NestPosition and ptr2.TargetWaypoint != NestPosition)
//        {
//            /* check if robot 2 target point is between robot 1 start and target point */
//            argos::Real distance_1 = CalculateDistance(ptr1.StartWaypoint, ptr2.TargetWaypoint);
//            argos::Real distance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//            argos::Real distance_Total = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
//
////
////            if(((distance_Total - 0.01) <= distance_1 + distance_2) and ((distance_Total + 0.01) >= distance_1 + distance_2))
////            {
//
//            // ptr 1 is  not in inner spiral. ptr2 is in inner spiral
//
//            if(((ptr1.StartWaypoint.GetX() > ptr2.StartWaypoint.GetX()) and
//               (ptr1.StartWaypoint.GetY() > ptr2.StartWaypoint.GetY())) or
//               (((distance_Total - 0.01) <= distance_1 + distance_2) and
//                ((distance_Total + 0.01) >= distance_1 + distance_2)))
//            {
//                /* check if robot 1 start, target and robot 2 target points are collinear */
//                Real Area_R1R2Goal =  ptr1.StartWaypoint.GetX() * (TargetPoint2.GetY() - TargetPoint1.GetY()) +
//                TargetPoint2.GetX() * (TargetPoint1.GetY() - ptr1.StartWaypoint.GetY()) +
//                TargetPoint1.GetX() * (ptr1.StartWaypoint.GetY() - TargetPoint2.GetY());
//
//                // add waypoint if target position of both robots and start point of robot 1 are collinear
//                if(0 <= abs(Area_R1R2Goal) and abs(Area_R1R2Goal) <= 0.1)
//                {
//                    AddNewWayPoint(ptr1, ptr2);
//                }
//            }
//        }
//        else if(ptr2.TargetWaypoint == NestPosition and ptr1.TargetWaypoint != NestPosition)
//        {
//            /* check if robot 2 start, target and robot 1 target points are collinear */
//            Real Area_R2R1Goal =  TargetPoint1.GetX() * (ptr2.StartWaypoint.GetY() - TargetPoint2.GetY()) +
//            ptr2.StartWaypoint.GetX() * (TargetPoint2.GetY() - TargetPoint1.GetY()) +
//            TargetPoint2.GetX() * (TargetPoint1.GetY() - ptr2.StartWaypoint.GetY());
//
//            /* check if robot 1 target point is between robot 2 start and target point */
//
//            argos::Real ddistance_1 = CalculateDistance(ptr2.StartWaypoint, ptr1.TargetWaypoint);
//            argos::Real ddistance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//            argos::Real ddistance_Total = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
//
//
//            if(((ddistance_Total- 0.01) <= ddistance_1 + ddistance_2) and ((ddistance_Total + 0.01) >= ddistance_1 + ddistance_2))
//            {
//                // add waypoint if target position of both robots and start point of robot 1 are collinear
//                if(0 <= abs(Area_R2R1Goal) and abs(Area_R2R1Goal) <= 0.1)
//                {
//                    AddNewWayPoint(ptr1, ptr2);
//                }
//            }
//        }
////        else
////        {
////
////            CheckRobotHeadingCourse(ptr1, ptr2, ptr3);
////        }
//
//
//
//}


/*************************************************************************************************************************************************************/
/* Function to check if robot course is very close */
/*************************************************************************************************************************************************************/
argos::Real DSA_loop_functions::DistancePointSegment(argos::CVector2 SegPoint1, CVector2 SegPoint2, CVector2 Point)
{
    argos::Real P1 = SegPoint2.GetX() - SegPoint1.GetX();
    argos::Real P2 = SegPoint2.GetY() - SegPoint1.GetY();
    
    argos::Real Product = (P1 * P1) + (P2 * P2);
    
    argos::Real u = (((Point.GetX() - SegPoint1.GetX()) * P1) + ((Point.GetY() - SegPoint1.GetY()) * P2))/Product;
    
    if(u > 1)
    {
        u = 1;
    }
    else if (u < 0)
    {
        u = 0;
    }
    
    argos::Real x = SegPoint1.GetX() + (u * P1);
    
    argos::Real y = SegPoint1.GetY() + (u * P2);
    
    argos::Real dx = x - Point.GetX();
    argos::Real dy = y - Point.GetY();
    
    argos::Real dist = sqrt((dx * dx) + (dy * dy));
    
    return dist;
}


/*************************************************************************************************************************************************************/
/* Function to check if robot course is very close */
/*************************************************************************************************************************************************************/
void DSA_loop_functions::CheckRobotHeadingCourse(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                                 BaseController::IntersectionData &ptr3)
{
    
    argos::Real RobotCourseAngle;
    argos::CVector2 WaypointAdd, PredictedTargetPt1, PredictedTargetPt2;
    argos:: Real shortest_dist;
    argos:: Real distance[4] = {0, 0, 0, 0};
    argos:: Real min;
    argos::UInt8 i;
    min = 100;
    i = 0;

    /* find the angle between lines */
    RobotCourseAngle = CalculateAngleBetweenRobotCourse(ptr1, ptr2);
    ptr1.InitialOrientation = RobotCourseAngle;
    ptr2.InitialOrientation = RobotCourseAngle;
    
    /* In 2d the shortest of the distance between point A and line segment CD, B and CD, C and AB or D and AB.
     So it's a fairly simple "distance between point and line" calculation (if the distances are all the same,
     then the lines are parallel. */
    
    // dist between robot 1 start, goal and robot2 start
//    distance[0] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.StartWaypoint);
    distance[0] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, ptr2.StartWaypoint);
    // dist between robot 1 start, goal and robot2 goal
//    distance[1] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.TargetWaypoint);
    distance[1] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, TargetPoint2);
    
    // dist between robot 2 start, goal and robot1 start
//    distance[2] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.StartWaypoint);
    distance[2] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, ptr1.StartWaypoint);
    
    // dist between robot 2 start, goal and robot2 goal
//    distance[3] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.TargetWaypoint);
    distance[3] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, TargetPoint1);
    
    
    for(i = 0; i< 4; i++)
    {
        if(distance[i] < min)
        {
            min = distance[i];
        }
        shortest_dist = min;
    }
    
    if(ptr1.id_robot == 0 and ptr2.id_robot == 2)
    {
        ptr1.Priority = shortest_dist;
        ptr2.Priority = shortest_dist;
    }
//    argos:: Real dist1 = (ptr1.StartWaypoint - NestPosition).SquareLength();
    argos:: Real dist1 = (ptr1.StartWaypoint - TargetPoint1).SquareLength();
    //        argos:: Real dist2 = (ptr1.TargetWaypoint - NestPosition).SquareLength();
//    argos:: Real dist3 = (ptr2.StartWaypoint - NestPosition).SquareLength();
    argos:: Real dist3 = (ptr2.StartWaypoint - TargetPoint2).SquareLength();
    //        argos:: Real dist4 = (ptr2.TargetWaypoint - NestPosition).SquareLength();

    argos:: Real distance_fromnest1 = CalculateDistance(ptr1.StartWaypoint, NestPosition);
    argos:: Real distance_fromnest2 = CalculateDistance(ptr2.StartWaypoint, NestPosition);
    // if the lines are almost parallel
    if(0 <= RobotCourseAngle and RobotCourseAngle <= OverlappingCourseAngle)
    {
        // if lines are very close to each other
        if(shortest_dist <= Safedistance)
        {

            if((ptr1.TargetWaypoint == NestPosition and ptr2.TargetWaypoint == NestPosition) or
               (ptr1.TargetWaypoint == NestPosition and distance_fromnest2 <= (NestRadiusSquared+0.02) and ptr2.GoingToOrFromNest == true) or
               (ptr2.TargetWaypoint == NestPosition and distance_fromnest1 <= (NestRadiusSquared+0.02) and ptr1.GoingToOrFromNest == true))
//            if((ptr1.TargetWaypoint == NestPosition and ptr2.TargetWaypoint == NestPosition) or
//               (ptr1.TargetWaypoint == NestPosition and ptr2.GoingToOrFromNest == true) or
//               (ptr2.TargetWaypoint == NestPosition and ptr1.GoingToOrFromNest == true))
            
            {
//                    ptr1.CollinearFlag = 1;
                    AddNewWayPoint(ptr1, ptr2);

            }
            // if 2nd robot is in spiral inside 1st robot
            else if(ptr2.TargetWaypoint != NestPosition and ptr2.StartWaypoint.GetX() > ptr1.StartWaypoint.GetX())
            {
                ptr1.StopTurningTime += 10;
            }
            
           
        }
    }

    
}

/*****************************************************************************************************************/
/* Function to find the angle between two lines */
/*****************************************************************************************************************/
Real DSA_loop_functions::CalculateAngleBetweenRobotCourse(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
{
    
//    argos::CDegrees headingToTargetR1 = ToDegrees((ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle());
//
//    ptr1.HeadingAngle = headingToTargetR1;
//    argos::CDegrees headingToTargetR2 = ToDegrees((ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle());
//
//    ptr2.HeadingAngle = headingToTargetR2;
    /* get the current heading angle of the robot */
//    argos::Real AngleRobotCourse = abs(180 - (abs(headingToTargetR1.GetValue()) + abs(headingToTargetR2.GetValue())));
//    argos::Real AngleRobotCourse = abs((abs(headingToTargetR1.GetValue()) - abs(headingToTargetR2.GetValue())));
    
    argos::Real angle;
    CVector2 v1, v2;
    Real v1_length, v2_length;
    
//    acos((v1 DOT v2)/(|v1|*|v2|))
    
    v1 = (ptr1.TargetWaypoint - ptr1.StartWaypoint);
    v2 = (ptr2.TargetWaypoint - ptr2.StartWaypoint);
    v1_length = v1.Length();
    v2_length = v2.Length();
    angle = acos((v1.DotProduct(v2))/(v1_length * v2_length));
    
    
//    argos::CRadians headingToTargetR1 = (ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle();
//    argos::CRadians headingToTargetR2 = (ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle();
//
//
//
//    CDegrees angle1 = ToDegrees(headingToTargetR1);
//    CDegrees angle2 = ToDegrees(headingToTargetR2);
//
//    ptr1.HeadingAngle = ToDegrees(headingToTargetR1);
//
//    ptr2.HeadingAngle = ToDegrees(headingToTargetR2);
//
//    CRadians res_angle_radians = (headingToTargetR1 - headingToTargetR2).SignedNormalize();
//
//    argos::Real AngleRobotCourse = ToDegrees(res_angle_radians).GetValue();
    
    argos::CRadians angle_readians = CRadians(angle);
    argos::Real AngleRobotCourse = ToDegrees(angle_readians).GetValue();
    
    AngleRobotCourse  = abs(AngleRobotCourse);

    return AngleRobotCourse;
    
}


/*****************************************************************************************************************/
/* Function to find the target point on the circle */
/*****************************************************************************************************************/
argos::CVector2 DSA_loop_functions::CalculateTargePoint(BaseController::RobotData& ptr)
{
    // Center of nest circle is (0,0)
    argos::Real x, y;
    argos::CVector2 PointOnCircle;
    argos::Real theta = atan2((ptr.StartWaypoint.GetY() - 0),(ptr.StartWaypoint.GetX()-0));
    x = 0 + NestRadius * cos(theta);
    y = 0 + NestRadius * sin(theta);
    PointOnCircle.Set(x, y);
    
    return PointOnCircle;
}

/*****************************************************************************************************************/
/* Function to find the target point on the circle */
/*****************************************************************************************************************/
void DSA_loop_functions::GetPointAtSafeDistance(BaseController::RobotData& ptr)
{
  
    argos::Real x, y;
    argos::UInt8 i;
    char previous_direction;
    
    x = ptr.StartWaypoint.GetX();
    y = ptr.StartWaypoint.GetY();
    
    PointChangeDirection.Set(x, y);
    
    if(ptr.pattern.size() > 0)
    {
        for(i=1;i<7;i++)
        {
            if((ptr.pattern.size() - i) > 0)
            {
                direction_last = ptr.pattern[ptr.pattern.size() - i];
                if(i == 1)
                {
                    previous_direction = direction_last;
                }
                if(i==3 && (PointChangeDirection == ptr.StartWaypoint))
                {
                   PointChangeDirection.Set(x, y);
                }
                
                switch(direction_last)
                {
                    case 'N':
//                        x = ptr.StartWaypoint.GetX()+ SearcherGap;
//                        y = ptr.StartWaypoint.GetY();
                        if(previous_direction != direction_last)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x + Searcher_Gap;
                        y = y;
                        previous_direction = direction_last;
                        break;
                        
                    case 'S':
                        if(previous_direction != direction_last)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x - Searcher_Gap;
                        y = y;
                        previous_direction = direction_last;
//                        x = ptr.StartWaypoint.GetX() - SearcherGap;
//                        y = ptr.StartWaypoint.GetY();
                      
                        break;
                        
                    case 'E':
                        if(previous_direction != direction_last)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x;
                        y = y - Searcher_Gap;
//                        x = ptr.StartWaypoint.GetX();
//                        y = ptr.StartWaypoint.GetY() - SearcherGap;
                        previous_direction = direction_last;
                        break;
                        
                    case 'W':
                        if(previous_direction != direction_last)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x;
                        y = y + Searcher_Gap;
//                        x = ptr.StartWaypoint.GetX();
//                        y = ptr.StartWaypoint.GetY()+ SearcherGap;
                        previous_direction = direction_last;
                        break;
                }
            }
        }

//        PointAtSafeDistance.Set(x, y);
    }
    
    else if(ptr.pattern.size() == 0)
    {
        /* do nothing */

    }
    PointAtSafeDistance.Set(x, y);
    ptr.IntersectionPt1 = PointChangeDirection;
    ptr.IntersectionPt2 = PointAtSafeDistance;
}

/*****************************************************************************************************************/
/* Function to add new way point */
/*****************************************************************************************************************/
void DSA_loop_functions::AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
{
    argos::CVector2 AddedWaypoint;
    argos::UInt16 TicksToWait_Robot1;
    argos::Real dist, x, y, radius, t1, t2;
    argos::Real  DistanceToTarget_Robot1, DistRobot2NeedToTravel;
    argos::CVector3 a, b, c;
    argos::CRadians headingAngleTowardsTarget;
    argos::Real dist1, dist2;
    argos::CRadians theta;
    argos::CVector2 robot2_vector;
     // add some stop time to allow robot 2 turn and move to waypoint
//    ptr1.StopTurningTime += 10;
    
//    if(ptr1.StartWaypoint.GetX() < ptr2.StartWaypoint.GetX())
//    if((ptr1.StartWaypoint.GetX() > ptr2.StartWaypoint.GetX()) and
//       (ptr1.StartWaypoint.GetY() > ptr2.StartWaypoint.GetY()))
//    {
//        theta = ToRadians(CDegrees(45.0f)).GetValue();
//
//    }
//    else
//    {
//        theta = ToRadians(CDegrees(135.0f)).GetValue();
//    }
    
//    dist = CalculateDistance(ptr1.StartWaypoint, ptr2.TargetWaypoint);
    
    if(ptr2.TargetWaypoint == NestPosition)
    {
        headingAngleTowardsTarget = (ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle();
    }
    else
    {
        headingAngleTowardsTarget = ToRadians(CDegrees(0.0f));
    }
    
//  // vector a = ptr2.TargetPoint - ptr1.Startwaypoint
//    a.SetX((ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
//    a.SetY((ptr2.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
//    a.SetZ(0);
//    // vector b = ptr1.StartWaypoint - ptr2.Startwaypoint
//    b.SetX((ptr1.StartWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
//    b.SetY((ptr1.StartWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
//    b.SetZ(0);
    
      // vector a = ptr2.TargetPoint - ptr2.Startwaypoint
        a.SetX((ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
        a.SetY((ptr2.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
        a.SetZ(0);
        // vector b = ptr1.TargetWaypoint - ptr1.Startwaypoint
        b.SetX((ptr1.TargetWaypoint.GetX() - ptr1.StartWaypoint.GetX()));
        b.SetY((ptr1.TargetWaypoint.GetY() - ptr1.StartWaypoint.GetY()));
        b.SetZ(0);
    
    // vector c = (b x a)
    c = b.CrossProduct(a);
    
    //if z coordinate is +ve, then ptr1 is to the left of ptr2
    if(c.GetZ() > 0)
    {
//        theta = ToRadians(CDegrees(45.0f)).GetValue() - headingAngleTowardsTarget.GetValue();
//        theta = ToRadians(CDegrees(45.0f)).GetValue();
        theta = ToRadians(CDegrees(45.0f));
        dist = CalculateDistance(ptr1.StartWaypoint, TargetPoint1);

    }
    else
    {
//       theta = ToRadians(CDegrees(135.0f)).GetValue() + headingAngleTowardsTarget.GetValue();
//        theta = ToRadians(CDegrees(135.0f)).GetValue();
        theta = ToRadians(CDegrees(135.0f));
        dist = CalculateDistance(ptr2.StartWaypoint, TargetPoint2);
    }
    
//    radius = (dist * sqrt(2))/2;
//
//    x = ptr2.StartWaypoint.GetX() + cos(theta) * radius;
//    y = ptr2.StartWaypoint.GetY() + sin(theta) * radius;
    
//    AddedWaypoint.Set(x,y);
    argos::CVector3 robot2vector3d = GetVectorFromTwoPoints(ptr2.StartWaypoint, TargetPoint2);
    robot2_vector.Set(robot2vector3d.GetX(), robot2vector3d.GetY());
    
    AddedWaypoint = robot2_vector.Rotate(theta);
    ptr2.AddedPoint = AddedWaypoint;
    
    if(ptr2.WaypointCounter <=10)
    {
        ptr1.CollinearFlag = 1;
        if(ptr2.WaypointStack.empty())
        {
            ptr2.WaypointStack.push(ptr2.TargetWaypoint);
        }

        /* add a way point before the final goal */
        ptr2.WaypointStack.push(AddedWaypoint);

//        ptr2.Waypoint_Added = true;
        ptr2.WaypointCounter++;
        
        dist1 = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
        dist2 = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
        
        // check if robot2 is in inner spiral than that of robot1
        if(dist1 >= dist2)
        {
            // check if robot 2 is stopped
            if(ptr2.StopTurningTime > 0)
            {
                ptr1.StopTurningTime += ptr2.StopTurningTime + 10;
            }
            else
            {
                ptr1.StopTurningTime += 10;
            }
        }
        // robot 2 is in the outer spiral than that of robot 1
        else
        {
            ptr1.StopTurningTime += 10;
        }

        
    }

}

/****************************************************************************************************************/
/* Function to check if robot paths intersect */
/****************************************************************************************************************/
void DSA_loop_functions::Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
                                           BaseController::IntersectionData &ptr3){
    
    argos::Real x_inter;
    argos::Real y_inter;
    argos::Real A1, A2, B1, B2, C1, C2, det;
    
    /* get the start and target position of robot 1 */
//    argos::CVector2 StartPosition_Robot1 = ptr1.StartWaypoint;
//    argos::CVector2 TargetPosition_Robot1 = ptr1.TargetWaypoint;
    
    argos::CVector2 Point1 = pt1;
    argos::CVector2 Point2 = pt2;
    
//    /* get the start and target position of robot 2 */
//    argos::CVector2 StartPosition_Robot2 = ptr2.StartWaypoint;
//    argos::CVector2 TargetPosition_Robot2 = ptr2.TargetWaypoint;
    
    argos::CVector2 Point3 = pt3;
    argos::CVector2 Point4 = pt4;
    
    /*A1 = Robot1_goal_y - Robot1_start_y*/
    A1 = Point2.GetY() - Point1.GetY();
    
    
    
    /*B1 = Robot1_start_x - Robot1_goal_x*/
    B1 = Point1.GetX() - Point2.GetX();
    
    
    
    /* C1 = A1 * Robot1_start_x + B1 * Robot1_start_y */
    C1 = A1 * Point1.GetX() + B1 * Point1.GetY();
    
    
    /*A2 = Robot2_goal_y - Robot2_start_y*/
    A2 = Point4.GetY() - Point3.GetY();
    
    /*B2 = Robot2_start_x - Robot2_goal_x*/
    B2 = Point3.GetX() - Point4.GetX();
    
    /* C2 = A2 * Robot2_start_x + B2 * Robot2_start_y */
    C2 = A2 * Point3.GetX() + B2 * Point3.GetY();
    
    det = A1*B2 - A2*B1;
    
    if(det == 0)
    {
        /* Lines are parallel */
        ptr3.Intersection_flag= 0;
    }
    
    /* Lines intersect and find the intersection point */
    else{
        x_inter = (B2*C1 - B1*C2)/det;
        
        y_inter = (A1*C2 - A2*C1)/det;
        
        /* Check if intersection point is out of bound for the line segment */
        if((x_inter < std::max(std::min(Point1.GetX(), Point2.GetX()),
                               std::min(Point3.GetX(), Point4.GetX()))) or
           (x_inter > std::min(std::max(Point1.GetX(), Point2.GetX()),
                               std::max(Point3.GetX(), Point4.GetX()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else if((y_inter < std::max(std::min(Point1.GetY(), Point2.GetY()),
                                    std::min(Point3.GetY(), Point4.GetY()))) or
                (y_inter > std::min(std::max(Point1.GetY(), Point2.GetY()),
                                    std::max(Point3.GetY(), Point4.GetY()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else
        {
            ptr3.Intersection_flag = 1;
            ptr3.IntersectionPoint.Set(x_inter, y_inter);
//            ptr3.Robot_ID_Intersectingwith = ptr1.id_robot;
        }
    }
}


/**************************************************************************************************************************/
/* Function to calculate time required by robot to reach intersection point */
/**************************************************************************************************************************/
//void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                                    BaseController::IntersectionData &ptr3){
void DSA_loop_functions::IntersectionCollisionCheck(CVector2 pt1, CVector2 pt2,
                                                    BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                                    BaseController::IntersectionData &ptr3, argos::UInt8 index)
{
    argos::UInt16 TicksToWait_Robot1, TicksToWait_Robot2, TicksToWaitforSafedistance, TimeToIntersection, TimeDiff, time_turn_safe_dist,            TimeToTurn_2;
    argos::Real DistanceToIntersection_Robot1, DistanceToIntersection_Robot2, IntersectionDistance, DistanceFromStartPoint;
    argos::Real AdjustedVelocity;
    bool Intersection_flag;

    ptr1.Intial_TurningWaitTime = 0;
    ptr2.Intial_TurningWaitTime = 0;
    /* Get the distance between start point and intersection point */
    if(index == 2)
    {
        DistanceFromStartPoint = CalculateDistance(ptr2.StartWaypoint, pt2);
        if((ptr2.StartWaypoint - ptr2.TargetWaypoint).Angle() != (pt2 - ptr3.IntersectionPoint).Angle())
        {
//            argos::CVector2 v1 = (ptr2.TargetWaypoint - ptr2.StartWaypoint);
//            argos::CVector2 v2 = (pt2 - ptr3.IntersectionPoint);
//            argos::Real length_1 = v1.Length();
//            argos::Real length_2 = v2.Length();
//
//            argos::Real angle = acos((v1.DotProduct(v2))/(length_1 * length_2));
//
//            argos::CRadians angle_readians = CRadians(angle);
//            argos::Real AngleRobotCourse = ToDegrees(angle_readians).GetValue();
//
//            AngleRobotCourse  = abs(AngleRobotCourse);

//            TimeToTurn_2 = GetTimeToTurn(AngleRobotCourse, ptr2.fBaseAngularWheelSpeed);
            TimeToTurn_2 = 2*GetTimeToTurn(90, ptr2.fBaseAngularWheelSpeed);
        }
        else
        {

            TimeToTurn_2 = 0;
        }
    }
    else
    {
        DistanceFromStartPoint = 0;
        TimeToTurn_2 = 0;
    }
    DistanceToIntersection_Robot1 = CalculateDistance(pt1, ptr3.IntersectionPoint);
    DistanceToIntersection_Robot2 = CalculateDistance(pt2, ptr3.IntersectionPoint);
//    DistanceToIntersection_Robot2 = CalculateDistance(pt2, ptr3.IntersectionPoint) + DistanceFromStartPoint;
//    DistanceToIntersection_Robot2 = CalculateDistance(pt2, ptr3.IntersectionPoint);

    /* calculate the time required to reach the intersection point */
    argos::Real AngleToTurn = ToDegrees((ptr1.StartWaypoint - TargetPoint1).Angle()).GetValue();
//    argos::UInt16 TimeToTurn_1 = GetTimeToTurn(AngleToTurn, ptr1.fBaseAngularWheelSpeed);
    argos::UInt16 TimeToTurn_1 = GetTimeToTurn(180, ptr1.fBaseAngularWheelSpeed);

    TicksToWait_Robot1 = GetTicksToWait(DistanceToIntersection_Robot1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime + TimeToTurn_1;


    TicksToWait_Robot2 = GetTicksToWait(DistanceToIntersection_Robot2, ptr2.fLinearWheelSpeed) + ptr2.StopTurningTime + TimeToTurn_2;

    TimeDiff = abs(TicksToWait_Robot1 - TicksToWait_Robot2);

    TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + GetTimeToTurn(180, ptr1.fBaseAngularWheelSpeed)) + 10;

    ptr1.Intial_TurningWaitTime = TicksToWait_Robot1;
    ptr2.Intial_TurningWaitTime = TicksToWait_Robot2;
//    ptr1.Priority = TicksToWaitforSafedistance;
//    ptr2.Priority = TicksToWaitforSafedistance;

    //if the difference between the times is equal to safe distance time between two robots
    if(TimeDiff <= TicksToWaitforSafedistance)
    {

        /* there is a chance of collision */
        /* slow down the velocity of robot 2 as its priority is lower */

//        if((ptr1.GoingToOrFromNest == true &&
//           (DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2)) or
//            ptr1.WaypointStackpopped == true)
        if((ptr1.GoingToOrFromNest == true or ptr1.WaypointStackpopped == true) and
           (DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2))
        {
            IntersectionDistance = DistanceToIntersection_Robot1;
            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MaxLinearSpeed;
//                Real Time = GetTicksToWait(IntersectionDistance , MaxLinearSpeed);
//                Real stop_time = abs(Time - TimeToIntersection);
//                Real stop_time = 2 * (TimeDiff + TicksToWaitforSafedistance);
                //                ptr2.StopTurningTime += 200;
//                ptr2.StopTurningTime += ((TimeToIntersection)+GetTimeToTurn(180, ptr2.fBaseAngularWheelSpeed));
                ptr2.StopTurningTime += ((TimeToIntersection));
//                ptr2.StopTurningTime += (stop_time);
            }

            ptr2.fLinearWheelSpeed = AdjustedVelocity;

        }
//        else if((ptr2.GoingToOrFromNest == true &&
//                (DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1))
//                 or ptr2.WaypointStackpopped == true)
        else if((ptr2.GoingToOrFromNest == true or ptr2.WaypointStackpopped == true) and
                (DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1))
        {
            IntersectionDistance = DistanceToIntersection_Robot1;
            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MaxLinearSpeed;
//                Real Time = GetTicksToWait(IntersectionDistance , MaxLinearSpeed);
//                Real stop_time = 2 * (TimeDiff + TicksToWaitforSafedistance);
//                Real stop_time = abs(Time - TimeToIntersection);
//                ptr1.StopTurningTime += ((TimeToIntersection)+GetTimeToTurn(180, ptr1.fBaseAngularWheelSpeed));
                ptr1.StopTurningTime += ((TimeToIntersection));
//                ptr1.StopTurningTime += (stop_time);
            }
            ptr1.fLinearWheelSpeed = AdjustedVelocity;
        }
        else
        {
            if((DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2) and ptr1.GoingToOrFromNest == false)
            {
                IntersectionDistance = DistanceToIntersection_Robot1;
                TimeToIntersection = TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MaxLinearSpeed;
                    ptr2.StopTurningTime += ((TimeToIntersection));
                }
                ptr2.fLinearWheelSpeed = AdjustedVelocity;


            }
            else if((DistanceToIntersection_Robot2 < DistanceToIntersection_Robot2) and ptr2.GoingToOrFromNest == false)
            {
                IntersectionDistance = DistanceToIntersection_Robot2;
                TimeToIntersection = TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MaxLinearSpeed;
                    ptr1.StopTurningTime += ((TimeToIntersection));
                }
                ptr1.fLinearWheelSpeed = AdjustedVelocity;

            }
            else
            {

                IntersectionDistance = DistanceToIntersection_Robot1;
                TimeToIntersection = TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MaxLinearSpeed;
                    ptr2.StopTurningTime += ((TimeToIntersection));
                }
                ptr2.fLinearWheelSpeed = AdjustedVelocity;

            }
        }


    }

    /* Reset the flag */
    ptr3.Intersection_flag = 0;

}

//void DSA_loop_functions::IntersectionCollisionCheck(CVector2 pt1, CVector2 pt2,
//                                                    BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                                    BaseController::IntersectionData &ptr3, argos::UInt8 index)
//{
//    argos::Real intersectiondist1, intersectiondist2, TurningAngle, IntersectionDistance, vec1length, vec2length;
//    argos::UInt16 IntersectionTime1, IntersectionTime2, TicksToWaitforSafedistance, TimeToIntersection, timetoturn1, timetoturn2;
//    argos::CVector2 v1, v2;
//    argos::Real AdjustedVelocity, Velocity, angle_robo1, angle_robo2;
//
//    IntersectionTime1 = 0;
//    IntersectionTime2 = 0;
//    intersectiondist1 = 0;
//    intersectiondist2 = 0;
//    // get the lowest velocity, to get accurate time to get a safe distance between robots
//    if(ptr1.fLinearWheelSpeed < ptr2.fLinearWheelSpeed)
//    {
//        Velocity = ptr1.fLinearWheelSpeed;
//    }
//    else
//    {
//        Velocity = ptr2.fLinearWheelSpeed;
//    }
//    // get the turning time for each robot:
//    angle_robo1 = ToDegrees((ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle()).GetValue();
//    angle_robo2 = ToDegrees((ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle()).GetValue();
//
//    // Get tutning time for each robot to go to the target location
//    timetoturn1 = GetTimeToTurn(angle_robo1, ptr1.fLinearWheelSpeed);
//    timetoturn2 = GetTimeToTurn(angle_robo2, ptr2.fLinearWheelSpeed);
//
//    // ticks to wait while robots are at safe distance from each other (add turning time, if robot needs to change direction for buffer)
//    TicksToWaitforSafedistance = GetTicksToWait(Safedistance, Velocity) + GetTimeToTurn(90, Velocity);
//
//
//    // if the intersection point is found before the robot changes direction
//    // intersection distance is equal to distance to intersection point - diameter of robot to accumulate error for collision avoidance
//    if(index == 1)
//    {
//
//        intersectiondist1 = CalculateDistance(pt1, ptr3.IntersectionPoint);
//        intersectiondist2 = CalculateDistance(pt2, ptr3.IntersectionPoint);
//
//        IntersectionTime1 = GetTicksToWait(intersectiondist1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime + timetoturn1;
//        IntersectionTime2 = GetTicksToWait(intersectiondist2, ptr2.fLinearWheelSpeed) + ptr2.StopTurningTime + timetoturn2;
//    }
//    // get the time to intersection where robot changes the direction
//    else
//    {
//        intersectiondist1 = CalculateDistance(pt1, ptr3.IntersectionPoint);
//        intersectiondist2 = CalculateDistance(ptr2.StartWaypoint, pt2) + CalculateDistance(pt2, ptr3.IntersectionPoint);
//
//        //calculate the vectors when robot changes the direction
//        v1.Set((PointChangeDirection.GetX() - ptr2.StartWaypoint.GetX()), (PointChangeDirection.GetY() - ptr2.StartWaypoint.GetY()));
//        v2.Set((PointAtSafeDistance.GetX() - PointChangeDirection.GetX()), (PointAtSafeDistance.GetY() - PointChangeDirection.GetY()));
//
//        vec1length = v1.Length();
//        vec2length = v2.Length();
//
//        argos::Real dot_product = v1.DotProduct(v2);
//
//        argos::CRadians Theta = ACos((dot_product/(vec1length * vec2length)));
//        TurningAngle = ToDegrees(Theta).GetValue();
//
//
//        IntersectionTime1 = GetTicksToWait(intersectiondist1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime + timetoturn1;
//        IntersectionTime2 = GetTicksToWait(intersectiondist2, ptr2.fLinearWheelSpeed) +
//                            GetTimeToTurn(TurningAngle, ptr2.fBaseAngularWheelSpeed)+ ptr2.StopTurningTime + timetoturn2;
//    }
//
//    ptr1.Intial_TurningWaitTime = TicksToWaitforSafedistance;
//    ptr2.Intial_TurningWaitTime = TicksToWaitforSafedistance;
//
//    ptr1.IntersectionTime = IntersectionTime1;
//    ptr2.IntersectionTime = IntersectionTime2;
//
//    // if both the robots reach the intersection point in the same time, then there is a chance of collision
//    if(abs(IntersectionTime1 - IntersectionTime2) <= (TicksToWaitforSafedistance + 10))
//    {
//
//        // if distance to intersection point of robot1 is less than that of robot2 and time to reach the
//        // intersection pt for robot 1 is less than that of robot 2
//        if((intersectiondist1 < intersectiondist2) and (IntersectionTime1 < IntersectionTime2))
//        {
//
//            IntersectionDistance = intersectiondist1;
//            TimeToIntersection = IntersectionTime1 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = ptr2.fLinearWheelSpeed;
//                ptr2.StopTurningTime += (2 * (TimeToIntersection + 10));
//
//            }
//
//            ptr2.fLinearWheelSpeed = AdjustedVelocity;
//        }
//        // if distance to intersection point of robot2 is less than that of robot1 and time to reach the
//        // intersection pt for robot 2 is less than that of robot 1
//        else if((intersectiondist2 < intersectiondist1) and (IntersectionTime2 < IntersectionTime1))
//        {
//            IntersectionDistance = intersectiondist2;
//            TimeToIntersection = IntersectionTime2 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = ptr1.fLinearWheelSpeed;
//                ptr1.StopTurningTime += (2*(TimeToIntersection + 10));
//
//            }
//
//            ptr1.fLinearWheelSpeed = AdjustedVelocity;
//        }
//        // if distance and time criteria are not satisfied
//        else
//        {
//            // time to reach the intersection for robot 1 is greater than robot2, adjust velocity and time for robot1
//            if(IntersectionTime1 < IntersectionTime2)
//            {
//                IntersectionDistance = intersectiondist2;
//                TimeToIntersection = IntersectionTime2 + TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = ptr1.fLinearWheelSpeed;
//                    ptr1.StopTurningTime += (2*(TimeToIntersection + 10));
//
//                }
//
//                ptr1.fLinearWheelSpeed = AdjustedVelocity;
//            }
//            // time to reach the intersection for robot 2 is greater than robot1, adjust velocity and time for robot2
//            else
//            {
//                IntersectionDistance = intersectiondist1;
//                TimeToIntersection = IntersectionTime1 + TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = ptr2.fLinearWheelSpeed;
//                    ptr2.StopTurningTime += (2*(TimeToIntersection + 10));
//
//                }
//
//                ptr2.fLinearWheelSpeed = AdjustedVelocity;
//            }
//        }// end of outer else
//    } // end of outer if
//
//    /* Reset the flag */
//    ptr3.Intersection_flag = 0;
//
//}
//



REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
