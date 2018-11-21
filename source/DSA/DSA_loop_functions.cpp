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
    NestRadius(1),
//    NestRadiusSquared(0.0625),
    NestRadiusSquared(1),
    NestElevation(0.01),
    SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
    ticks_per_second(0),
    sim_time(0),
    score(0),
    PrintFinalScore(0),
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
        
//        RobotReachedWayPoint |= stRobotDataCopy.WaypointReached;
        
        NewWayPointAdded |= stRobotDataCopy.Waypoint_Added;
        
        if(FirstCheck == 0)
        {
            Result_Checked &= stRobotDataCopy.Checked;
        }
       
    }
    FirstCheck = Result_Checked;
    
    
    /* check collinearity and intersection if target reached or new waypoint added or its the start of code */
//    if(FirstCheck == 1 and (RobotReachedWayPoint == 1 or NewWayPointAdded == 1))
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
                
//                cController.SetHardStopMovement();
//                cController1.SetHardStopMovement();
                
                sRobotDataprevious.StartWaypoint = cController.GetPosition();
                sRobotDatanext.StartWaypoint = cController1.GetPosition();
                
                /* check if robot's end waypoint is collinear in other robot's start and end waypoint */
                CheckCollinearity(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
               
                if(sRobotDataprevious.CollinearFlag == 1 and !sRobotDatanext.WaypointStack.empty())
                {
                    sRobotDatanext.AddedPoint = sRobotDatanext.WaypointStack.top();

                    cController1.SetTarget(sRobotDatanext.AddedPoint);
//                    cController1.SetMovement();
//                    cController1.SetHardStopMovement();
                    sRobotDatanext.WaypointStack.pop();

                }
               
            
                if(sRobotDataprevious.CollinearFlag !=1)
                {

                    /* find intersection between paths of robots */
                    Find_Intersection(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);


                    if(sIntersectionDatanext.Intersection_flag == 1)
                    {
                        /* take actions to avoid intersections */
                        IntersectionCollisionCheck(sRobotDataprevious, sRobotDatanext, sIntersectionDatanext);
//                        cController.SetHardStopMovement();
//                        cController1.SetHardStopMovement();
//                        if(sRobotDatanext.Intersection_Adjustment == 1)
//                        {
//                            cController1.SetMovement();
//                        }
                    }


                }
//                cController.SetHardStopMovement();
//                cController1.SetHardStopMovement();
                
            }

//            sRobotDataprevious.pathcheck = true;
//            cController.SetMovement();
            
        }
        
        for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
            it5 != m_cFootbots.end();
            ++it5)
        {
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
            BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
            BaseController::RobotData& stRobotData_1 = cController5.GetRobotData();
            
            stRobotData_1.pathcheck = true;
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
void DSA_loop_functions::Find_Intersection(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                             BaseController::IntersectionData &ptr3){
    
    argos::Real x_inter;
    argos::Real y_inter;
    argos::Real A1, A2, B1, B2, C1, C2, det;
    
    /* get the start and target position of robot 1 */
    argos::CVector2 StartPosition_Robot1 = ptr1.StartWaypoint;
    argos::CVector2 TargetPosition_Robot1 = ptr1.TargetWaypoint;
    
    /* get the start and target position of robot 2 */
    argos::CVector2 StartPosition_Robot2 = ptr2.StartWaypoint;
    argos::CVector2 TargetPosition_Robot2 = ptr2.TargetWaypoint;
    
    /*A1 = Robot1_goal_y - Robot1_start_y*/
    A1 = TargetPosition_Robot1.GetY() - StartPosition_Robot1.GetY();
    
    
    
    /*B1 = Robot1_start_x - Robot1_goal_x*/
    B1 = StartPosition_Robot1.GetX() - TargetPosition_Robot1.GetX();
    
    
    
    /* C1 = A1 * Robot1_start_x + B1 * Robot1_start_y */
    C1 = A1 * StartPosition_Robot1.GetX() + B1 * StartPosition_Robot1.GetY();
    
    
    /*A2 = Robot2_goal_y - Robot2_start_y*/
    A2 = TargetPosition_Robot2.GetY() - StartPosition_Robot2.GetY();
    
    /*B2 = Robot2_start_x - Robot2_goal_x*/
    B2 = StartPosition_Robot2.GetX() - TargetPosition_Robot2.GetX();
    
    /* C2 = A2 * Robot2_start_x + B2 * Robot2_start_y */
    C2 = A2 * StartPosition_Robot2.GetX() + B2 * StartPosition_Robot2.GetY();
    
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
        if((x_inter < std::max(std::min(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
                               std::min(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))) or
           (x_inter > std::min(std::max(StartPosition_Robot1.GetX(), TargetPosition_Robot1.GetX()),
                               std::max(StartPosition_Robot2.GetX(), TargetPosition_Robot2.GetX()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else if((y_inter < std::max(std::min(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
                                    std::min(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))) or
                (y_inter > std::min(std::max(StartPosition_Robot1.GetY(), TargetPosition_Robot1.GetY()),
                                    std::max(StartPosition_Robot2.GetY(), TargetPosition_Robot2.GetY()))))
        {
            ptr3.Intersection_flag = 0;
        }
        else
        {
            ptr3.Intersection_flag = 1;
            ptr3.IntersectionPoint.Set(x_inter, y_inter);
            ptr3.Robot_ID_Intersectingwith = ptr1.id_robot;
        }
    }
}

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
void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                                      BaseController::IntersectionData &ptr3){
    
    argos::UInt16 TicksToWait_Robot1, TicksToWait_Robot2, TicksToWaitforSafedistance, TimeToIntersection, TimeDiff;
    argos::Real DistanceToIntersection_Robot1, DistanceToIntersection_Robot2, IntersectionDistance;
    argos::Real AdjustedVelocity;
    bool Intersection_flag;
    
    ptr1.Intersection_Adjustment = 0;
    ptr2.Intersection_Adjustment = 0;
    
    /* Get the distance between start point and intersection point */
    DistanceToIntersection_Robot1 = CalculateDistance(ptr1.StartWaypoint, ptr3.IntersectionPoint);
    DistanceToIntersection_Robot2 = CalculateDistance(ptr2.StartWaypoint, ptr3.IntersectionPoint);
    
    /* calculate the time required to reach the intersection point */
//    TicksToWait_Robot1 = GetTicksToWait(DistanceToIntersection_Robot1, ptr1.fLinearWheelSpeed) + ptr1.StopTurningTime;
    TicksToWait_Robot1 = CalculateTotalTime(ptr1, ptr3.IntersectionPoint, ptr1.StartWaypoint);
    TicksToWait_Robot2 = CalculateTotalTime(ptr2, ptr3.IntersectionPoint, ptr2.StartWaypoint);
//    TicksToWait_Robot2 = GetTicksToWait(DistanceToIntersection_Robot2, ptr2.fLinearWheelSpeed)+ ptr2.StopTurningTime;
    
    
    TimeDiff = abs(TicksToWait_Robot1 - TicksToWait_Robot2);
    
    TicksToWaitforSafedistance = GetTicksToWait(Safedistance , MaxLinearSpeed);
    
    
    //if the difference between the times is equal to safe distance time between two robots
    if(TimeDiff <= TicksToWaitforSafedistance)
    {
        
        /* there is a chance of collision */
        /* slow down the velocity of robot 2 as its priority is lower */
        
        if(DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2)
        {
            IntersectionDistance = DistanceToIntersection_Robot2;
            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
            
            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MinLinearSpeed;
                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                Real stop_time = abs(Time - TimeToIntersection);
                ptr2.StopTurningTime += stop_time;
            }
            ptr2.fLinearWheelSpeed = AdjustedVelocity;
            ptr2.Intersection_Adjustment = 1;
            
        }
        /* slow down the velocity of robot 1 as its priority is lower */
        else if(DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1)
        {
            IntersectionDistance = DistanceToIntersection_Robot1;
            TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
            if(AdjustedVelocity < MinLinearSpeed)
            {
                AdjustedVelocity = MinLinearSpeed;
                Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                Real stop_time = abs(Time - TimeToIntersection);
                ptr1.StopTurningTime += stop_time;
            }
            ptr1.fLinearWheelSpeed = AdjustedVelocity;
            ptr1.Intersection_Adjustment = 1;
        }
        else{
            if(ptr1.GoingToNest)
            {
                IntersectionDistance = DistanceToIntersection_Robot2;
                TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/ TimeToIntersection) * SimulatorTicksperSec;
                
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MinLinearSpeed;
                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                    Real stop_time = abs(Time - TimeToIntersection);
                    ptr2.StopTurningTime += stop_time;
                }
                ptr2.fLinearWheelSpeed = AdjustedVelocity;
                ptr2.Intersection_Adjustment = 1;
            }
            else{
                IntersectionDistance = DistanceToIntersection_Robot1;
                TimeToIntersection = TicksToWait_Robot2 + TicksToWaitforSafedistance;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MinLinearSpeed;
                    Real Time = GetTicksToWait(IntersectionDistance , MinLinearSpeed);
                    Real stop_time = abs(Time - TimeToIntersection);
                    ptr1.StopTurningTime += stop_time;
                }
                ptr1.fLinearWheelSpeed = AdjustedVelocity;
                ptr1.Intersection_Adjustment = 1;
            }
        }
        
    }
    
    /* Reset the flag */
    ptr3.Intersection_flag = 0;
    
}

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

void DSA_loop_functions::AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)

{
    argos::CVector2 AddedWaypoint;
    argos::Real diff, x, y;
    
    argos::CVector2 du;
    
    argos::Real Factor = 0.3;
    x = 0;
    y= 0;
    diff = 0;

    
    
    
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
    
    


}

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
/* Function to check if robot start and target points are collinear */
/*****************************************************************************************************************/
void DSA_loop_functions::CheckCollinearity(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
                                           BaseController::IntersectionData &ptr3)
{
    
    argos::CVector2 AddedWaypoint;
    argos::Real Angle, Distance, distBetweenTargets, distBetweenStartPoints, dist1, dist2;
    
    ptr1.CollinearFlag = 0;
    ptr2.CollinearFlag = 0;

//    distBetweenTargets = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//    distBetweenStartPoints = CalculateDistance(ptr1.StartWaypoint, ptr2.StartWaypoint);
//
//    dist1 = CalculateDistance(ptr1.StartWaypoint, NestPosition);
//    dist2 = CalculateDistance(ptr2.StartWaypoint, NestPosition);
    
    /* check if robot 1 start, target and robot 2 target points are collinear */
//    Real Area_R1R2Goal =  ptr1.StartWaypoint.GetX() * (ptr2.TargetWaypoint.GetY() - ptr1.TargetWaypoint.GetY()) +
//    ptr2.TargetWaypoint.GetX() * (ptr1.TargetWaypoint.GetY() - ptr1.StartWaypoint.GetY()) +
//    ptr1.TargetWaypoint.GetX() * (ptr1.StartWaypoint.GetY() - ptr2.TargetWaypoint.GetY());
//
//
//    /* check if robot 2 start, target and robot 1 target points are collinear */
//    Real Area_R2R1Goal =  ptr1.TargetWaypoint.GetX() * (ptr2.StartWaypoint.GetY() - ptr2.TargetWaypoint.GetY()) +
//    ptr2.StartWaypoint.GetX() * (ptr2.TargetWaypoint.GetY() - ptr1.TargetWaypoint.GetY()) +
//    ptr2.TargetWaypoint.GetX() * (ptr1.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY());
    
    
    CheckRobotHeadingCourse(ptr1, ptr2, ptr3);

    if(ptr1.CollinearFlag!= 1)
    {
//        if((ptr1.TargetWaypoint == ptr2.TargetWaypoint) ||
//            distBetweenTargets < 0.5)
//        {
//
//             AddNewWayPoint(ptr1, ptr2);
//        }

//        if((ptr1.TargetWaypoint == ptr2.TargetWaypoint) ||
//           (ptr1.TargetWaypoint == NestPosition && dist2 < Safedistance) ||
//           (ptr2.TargetWaypoint == NestPosition && dist1 < Safedistance))
        if((ptr1.TargetWaypoint == ptr2.TargetWaypoint))
        {
                AddNewWayPoint(ptr1, ptr2);

        }
     }
//    else if(dist1 < NestRadiusSquared or dist2 < NestRadiusSquared)
//    {
//        AdjustTimeToReachNest(ptr1, ptr2);
//    }


//    if(0 <= abs(Area_R1R2Goal) and abs(Area_R1R2Goal) <= 0.1)
//    {
//
//        /* check if robot 2 target point is between robot 1 start and target point */
//
//        argos::Real distance_1 = CalculateDistance(ptr1.StartWaypoint, ptr2.TargetWaypoint);
//        argos::Real distance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//        argos::Real distance_Total = CalculateDistance(ptr1.StartWaypoint, ptr1.TargetWaypoint);
//
//
//        if(((distance_Total - 0.01) <= distance_1 + distance_2) and ((distance_Total + 0.01) >= distance_1 + distance_2))
//        {
//            ptr1.CollinearFlag = 1;
//            if(ptr1.id_robot < ptr2.id_robot)
//            {
//                ptr1.CollinearFlag = 1;
//
//                AddNewWayPoint(ptr1, ptr2);
//
//            }
//            // add waypoint for 1st robot
//            else{
//
//                CalculateWaitTime(ptr1, ptr2);
//            }
//
//        }
//
//    }
//
//    else if(0 <= abs(Area_R2R1Goal) and abs(Area_R2R1Goal) <= 0.1)
//    {
//
//        /* check if robot 1 target point is between robot 2 start and target point */
//
//        argos::Real ddistance_1 = CalculateDistance(ptr2.StartWaypoint, ptr1.TargetWaypoint);
//        argos::Real ddistance_2 = CalculateDistance(ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//        argos::Real ddistance_Total = CalculateDistance(ptr2.StartWaypoint, ptr2.TargetWaypoint);
//
//
//        if(((ddistance_Total- 0.01) <= ddistance_1 + ddistance_2) and ((ddistance_Total + 0.01) >= ddistance_1 + ddistance_2))
//        {
//            ptr1.CollinearFlag = 1;
//
//            if(ptr1.id_robot < ptr2.id_robot)
//            {
//                ptr1.CollinearFlag = 1;
//                AddNewWayPoint(ptr1, ptr2);
//
//            }
//            // add waypoint for 1st robot
//            else{
//                 AddNewWayPoint(ptr1, ptr2);
//
//            }
//
//        }
//    }

//    else
//    {
//
//        CheckRobotHeadingCourse(ptr1, ptr2, ptr3);
//    }
    
}


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
    argos::CVector2 WaypointAdd;
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
    distance[0] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.StartWaypoint);
    // dist between robot 1 start, goal and robot2 goal
    distance[1] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.TargetWaypoint);
    
    // dist between robot 2 start, goal and robot1 start
    distance[2] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.StartWaypoint);
    
    // dist between robot 2 start, goal and robot2 goal
    distance[3] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.TargetWaypoint);
    
    
    ptr1.Priority = shortest_dist;
    ptr2.Priority = shortest_dist;
    
    for(i = 0; i< 4; i++)
    {
        if(distance[i] < min)
        {
            min = distance[i];
        }
        shortest_dist = min;
    }
    
    argos:: Real dist1 = (ptr1.StartWaypoint - NestPosition).SquareLength();
    //        argos:: Real dist2 = (ptr1.TargetWaypoint - NestPosition).SquareLength();
    argos:: Real dist3 = (ptr2.StartWaypoint - NestPosition).SquareLength();
    //        argos:: Real dist4 = (ptr2.TargetWaypoint - NestPosition).SquareLength();

    // if the lines are almost parallel
    if(0 <= RobotCourseAngle and RobotCourseAngle <= OverlappingCourseAngle)
    {
        // if lines are very close to each other
        if(shortest_dist <= 0.40)
        {
//            if(((dist1 < NestRadiusSquared) or (ptr1.TargetWaypoint == NestPosition)) &&
//               ((ptr2.TargetWaypoint == NestPosition) or (dist3 < NestRadiusSquared)))
            if(((dist1 < Safedistance) or (ptr1.TargetWaypoint == NestPosition)) &&
               ((ptr2.TargetWaypoint == NestPosition) or (dist3 < Safedistance)))
//            if(ptr1.TargetWaypoint == NestPosition or ptr2.TargetWaypoint == NestPosition)
            {
//                    ptr1.CollinearFlag = 1;
                    AddNewWayPoint(ptr1, ptr2);

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
    
    argos::CRadians headingToTargetR1 = (ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle();
    argos::CRadians headingToTargetR2 = (ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle();

    CDegrees angle1 = ToDegrees(headingToTargetR1);
    CDegrees angle2 = ToDegrees(headingToTargetR2);

    ptr1.HeadingAngle = ToDegrees(headingToTargetR1);

    ptr2.HeadingAngle = ToDegrees(headingToTargetR2);
    
    CRadians res_angle_radians = (headingToTargetR1 - headingToTargetR2).SignedNormalize();
//    argos::Real res_angle = abs(abs((ToDegrees(headingToTargetR1).GetValue())) - abs((ToDegrees(headingToTargetR2).GetValue())));
//    CDegrees res_angle_degrees = ToDegrees(res_angle_radians);
//    argos::Real AngleRobotCourse = res_angle;
    argos::Real AngleRobotCourse = ToDegrees(res_angle_radians).GetValue();
    
    AngleRobotCourse  = abs(AngleRobotCourse);
//    if(AngleRobotCourse < 0)
//    {
//        AngleRobotCourse = -1 * AngleRobotCourse;
//    }
//    argos::Real AngleRobotCourse = ToDegrees((headingToTargetR1.GetValue() - headingToTargetR2.GetValue()).SignedNormalize()).GetValue();
    return AngleRobotCourse;
    
}





REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
