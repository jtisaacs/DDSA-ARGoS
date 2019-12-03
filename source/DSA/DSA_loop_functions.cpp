 #include "DSA_loop_functions.h"

DSA_loop_functions::DSA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
    ResourceDensityDelay(0),
    SimCounter(0),
    MaxSimCounter(1),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(4),
    DrawIDs(1),
    DrawTrails(0),
    DrawTargetRays(0),
    FoodDistribution(1),
    FoodItemCount(256),
//    NumberOfClusters(4),
    NumberOfClusters(1),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),
    NestRadius(0.5),
    NestRadiusSquared(0.25),
    NestElevation(0.01),
    SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
    ticks_per_second(0),
    sim_time(0),
    score(0),
    PrintFinalScore(1),
    Searcher_Gap(0.18),
    SimulatorTicksperSec(0),
    Average_Collision_Avoidance(0.0),
    AnchorDistance(0.54),
    algoactivated(false),
    MaxLinearSpeed(8.0),
    speedcheck_flag(false),
    m_pcRNG(NULL)
{}

void DSA_loop_functions::Init(TConfigurationNode& node)
{
    CSimulator     *simulator     = &GetSimulator();
    Random_Seed = simulator->GetRandomSeed();
    CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("default");
    ticks_per_second = physicsEngine->GetInverseSimulationClockTick();
    argos::TConfigurationNode DDSA_node = argos::GetNode(node, "DDSA");
    argos::GetNodeAttribute(DDSA_node, "PrintFinalScore",                   PrintFinalScore);
    argos::GetNodeAttribute(DDSA_node, "FoodDistribution",                  FoodDistribution);
    argos::GetNodeAttribute(DDSA_node, "FoodItemCount",                  FoodItemCount);
    argos::GetNodeAttribute(DDSA_node, "NestRadius",                 NestRadius);
    argos::GetNodeAttribute(DDSA_node, "FoodBoundsWidth",                 FoodBoundsWidth);
    argos::GetNodeAttribute(DDSA_node, "FoodBoundsHeight",                 FoodBoundsHeight);
    argos::GetNodeAttribute(DDSA_node, "NeighborRadius",      Neighbor_Radius);
    argos::GetNodeAttribute(DDSA_node, "ResultsPath",      file_path);
    argos::GetNodeAttribute(DDSA_node, "ClusterCenter",      ClusterPos);
    argos::GetNodeAttribute(DDSA_node, "RotationAngle",      Rotation_Angle);
    argos::GetNodeAttribute(DDSA_node, "WaypointDist",      WaypointDistance);
    
    TargetRayList.clear();
    TargetRayColorList.clear();
    
    NestRadiusSquared = NestRadius*NestRadius;
    
    // Initialization of AnchorPoints for collinear path collision avoidance
    First_QuadrantAnchorPoint.Set(0.0, -AnchorDistance);
    Second_QuadrantAnchorPoint.Set(AnchorDistance, 0.0);
    Third_QuadrantAnchorPoint.Set(0.0, AnchorDistance);
    Fourth_QuadrantAnchorPoint.Set(-AnchorDistance, 0.0);

    // calculate the forage range and compensate for the robot's radius of 0.085m
    argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
    
    argos::Real rangeX1 = (ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY1 = (ArenaSize.GetY() / 2.0) - 0.085;
    ForageRangeX_1.Set(-rangeX1, rangeX1);
    ForageRangeY_1.Set(-rangeY1, rangeY1);
    
    /* arena distance */
    MaxArenaDistance = sqrt(rangeX1 * rangeX1  + rangeY1 *rangeY1);
  
    argos::Real rangeX = FoodBoundsWidth/2.0;//(ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = FoodBoundsHeight/2.0;//(ArenaSize.GetY() / 2.0) - 0.085;  
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    TotalRobots = footbots.size();
    SimulatorTicksperSec = GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();
    
    CSpace::TMapPerType::iterator it;
    CSpace::TMapPerType::iterator it_other;
    
	for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        DSA_controller& c2 = dynamic_cast<DSA_controller&>(c);

        c2.SetLoopFunctions(this);
	}

    for(it_other = footbots.begin(); it_other != footbots.end(); it_other++)
    {
        /* Get handle to foot-bot entity and controller of the current robot */
        BaseController::RobotData* stRobotDataCopy2 = NULL;
        CFootBotEntity& cFootBotCopy2 = *any_cast<CFootBotEntity*>(it_other->second);
        BaseController& cControllerCopy2 = dynamic_cast<BaseController&>(cFootBotCopy2.GetControllableEntity().GetController());
        cControllerCopy2.SetNestPosition(NestPosition);
        stRobotDataCopy2 = &cControllerCopy2.GetRobotData();
        cControllerCopy2.SetNestRadius(NestRadius);
        if(speedcheck_flag == false and FirstCheck == false)
        {
            MaxLinearSpeed = stRobotDataCopy2->fLinearWheelSpeed;
            speedcheck_flag = true;
        }
    }
    if(CLUSTERCONFIGONLY == true)
    {
        FoodItemCount = 64;
    }
	SetFoodDistribution();
   
    RobotReachedWayPoint = 0;
    FirstCheck = 0;
    
    TestValue = 10;
    TestVariable = 11;
    LeftMostRobotID = 60;
    Result_Checked = 1;
    // Name the results file with the current time and date
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    stringstream ss;
    

    
//    ss <<"DSA-LogFile-"<<now->tm_min << '-'
//    <<  now->tm_sec <<".csv";
//
//    file_name = ss.str();
//    full_path = file_path+"/"+file_name;
//
//    ofstream results_output_stream;
//    results_output_stream.open(full_path, ios::app);
//    results_output_stream <<"Random Seed, " <<"Number_Of_Robots, "<<"Simulator_Clock, "
//    << "Number_Of_Targets, "<<"Neighbor_Radius, "<<"Average_Collision_Avoidance "<<endl;
//    results_output_stream.close();
    
}


double DSA_loop_functions::Score()
{  
  return score;
}


void DSA_loop_functions::setScore(double s)
{
    score = s;
    SetAverageCollision();
    
//    ofstream results_output_stream;
//    results_output_stream.open(full_path, ios::app);
//    results_output_stream<< Random_Seed << "," <<TotalRobots << ", "
//    << getSimTimeInSeconds() << ", "
//    << score << "," << Neighbor_Radius << ","<< Average_Collision_Avoidance <<endl;
//    results_output_stream.close();
    
  if (score >= FoodItemCount)
    {
      PostExperiment();
      exit(0);
    }
}

void DSA_loop_functions::SetAverageCollision()
{
    argos::Real TotalNumberOfCollisions = 0.0;
    Average_Collision_Avoidance = 0;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
        it5 != m_cFootbots.end();
        ++it5)
    {
        BaseController::RobotData* stRobotData_1 = NULL;
        std::vector<BaseController::IntersectionData>* stIntersectionData = NULL;
        
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
        BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
        
        stRobotData_1 = &cController5.GetRobotData();
        
        TotalNumberOfCollisions += stRobotData_1->CollisionCounter;
    }
    
    Average_Collision_Avoidance = TotalNumberOfCollisions/m_cFootbots.size();
}

void DSA_loop_functions::PostExperiment() 
{
  
    
    SetAverageCollision();
    
    if (PrintFinalScore == 1) printf("%d, %f, %f, %f\n", TotalRobots, getSimTimeInSeconds(), score, Average_Collision_Avoidance);
    
//    ofstream results_output_stream;
//    results_output_stream.open(full_path, ios::app);
//    results_output_stream<< Random_Seed << "," <<TotalRobots << ","
//    << getSimTimeInSeconds() << ","
//    << score << "," << Neighbor_Radius << "," << Average_Collision_Avoidance <<endl;
//    results_output_stream.close();
}


void DSA_loop_functions::PreStep() 
{
    argos::UInt16 time1 = getSimTimeInSeconds();
    argos::UInt16 counter = 0;
    argos::Real RobotCourseAngle = 0;
    
    /* reset the RobotReachedWayPoint flag */
    RobotReachedWayPoint = 0;
    
    /* reset the RobotReachedWayPoint flag */
    NewWayPointAdded = 0;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    
    // clear the vector before using for the next check
    RobotResource.clear();
 
    //Reset the Result_Checked variable
    Result_Checked = 1;
    
    
    /* Get the hadndle to each robot and check if any one waypoint reached*/
    for(CSpace::TMapPerType::iterator it2 = m_cFootbots.begin();
        it2 != m_cFootbots.end();
        ++it2)
    {
        BaseController::RobotData *stRobotDataCopy = NULL;
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(it2->second);
        BaseController& cController2 = dynamic_cast<BaseController&>(cFootBot2.GetControllableEntity().GetController());
        
        stRobotDataCopy = &cController2.GetRobotData();

        
        NewWayPointAdded |= stRobotDataCopy->Waypoint_Added;
        if(stRobotDataCopy->Waypoint_Added == true)
        {
            
            RobotResource.push_back(stRobotDataCopy->id_robot);
        }
        
        if(FirstCheck == 0)
        {
            Result_Checked &= stRobotDataCopy->Checked;
            
        }
        
    }
    FirstCheck = Result_Checked;
    
    argos::CSpace::TMapPerType& m_cFootbots1 = GetSpace().GetEntitiesByType("foot-bot");
    
    if(algoactivated == false)
    {
        for(CSpace::TMapPerType::iterator it_1 = m_cFootbots1.begin();
            it_1 != m_cFootbots1.end();
            ++it_1)
        {
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBotCopy1 = *any_cast<CFootBotEntity*>(it_1->second);
            BaseController& cControllerCopy1 = dynamic_cast<BaseController&>(cFootBotCopy1.GetControllableEntity().GetController());
            cControllerCopy1.SetAlgorithmActivated(FirstCheck);
        }
        algoactivated = FirstCheck;
    }
    

    /* check collinearity and intersection if target reached or new waypoint added or its the start of code */
    if(FirstCheck == 1 and  NewWayPointAdded == 1)
    {
        
        if(Neighbor_Radius != 0)
        {

            /* Stop all the robots before checking for collision*/
            StopAllRobots();

             /* Clear the vector data of all the robots before checking for collision */
            ClearRobotVectorData();

            /* Get the neighbors of the robots who have collected the resource*/
            GetNeighbors();
            
            
            CollectiveCollinearCheck();
            
            
             /* Check the collision and avoid collision for inersection and collinearity */
            CheckCollisionAndConsistency();
        }

        /* set the flags to indicate that collision checks have been completed */
        CheckComplete();
        
 
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

/*****
 *
 *****/
void DSA_loop_functions::ClusterFoodDistribution() {
    FoodList.clear();
    
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = FoodItemCount;//Wayne: Changed since no longer necessary
	size_t          foodPlaced = 0;
    argos::CVector2 placementPosition;
    
    //apurva
//    argos::CVector2 placementPositionArr[1] = {{-2, -1}};
    
    if(CLUSTERCONFIGONLY == true)
    {
        ClusterLengthY = 8;
        ClusterWidthX = 8;
    }
    
    //apurva
    if(CLUSTERCONFIGONLY == false)
    {
        FindClusterLengthWidth();//Wayne: sets cluster sides (X,Y)
    }

    //-----Wayne: Creates vector of number of food in each cluster
    size_t index = 0;
    size_t ClusterFoodCount = 0;
    size_t foodCount = 0;
    vector <size_t> FoodClusterCount;
    
    //initialize vector
    for (int i = 0; i < NumberOfClusters; i++)
    {
        FoodClusterCount.push_back(0);
    }
    
    //add food
    while (foodCount < FoodItemCount)
    {
        FoodClusterCount[index] = FoodClusterCount[index]+ 1;
        foodCount++;
        index++;
        if (index == NumberOfClusters) index = 0;
        
    }
    
    //make vector cumulative in food
    for (int i = 1; i < NumberOfClusters; i++)
    {
        FoodClusterCount[i] += FoodClusterCount[i - 1];
    }
    //------Wayne: end of vector creation
    
	for(size_t i = 0; i < NumberOfClusters; i++)
    {
        if(CLUSTERCONFIGONLY == false)
        {
            //apurva
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
            while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX))
            {
                placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
    
            }
        }
        else if(CLUSTERCONFIGONLY == true)
        {
//            placementPosition = placementPositionArr[i];
            placementPosition = ClusterPos;
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

/*****
 *
 *****/
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

/*****
 *
 *****/

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
/* Function to reset all robot vector data */
/****************************************************************************************************************/
void DSA_loop_functions::ClearRobotVectorData()
{
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
        it5 != m_cFootbots.end();
        ++it5)
    {
        BaseController::RobotData* stRobotData_1 = NULL;
        std::vector<BaseController::IntersectionData>* stIntersectionData = NULL;
        
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
        BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
        
        stRobotData_1 = &cController5.GetRobotData();
//        stIntersectionData = &cController5.GetIntersectionData();
        stIntersectionData = cController5.GetIntersectionData();
        
        //clear neighbor matrix
        stRobotData_1->NeighborsMatrix.clear();
        stRobotData_1->Neighbors.clear();
        // clear the intersection vector
        stIntersectionData->clear();
        stRobotData_1->CollinearRobotGoingToNestList.clear();
        stRobotData_1->CollinearRobotGoingAwayNestList.clear();
        
//        stRobotData_1->StopTurningTime = 0;
    }
        
}



/****************************************************************************************************************/
/* Function to reset all robot data */
/****************************************************************************************************************/
void DSA_loop_functions::CheckComplete()

{
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
        it5 != m_cFootbots.end();
        ++it5)
    {
        BaseController::RobotData* stRobotData_1 = NULL;

        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
        BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
        
        stRobotData_1 = &cController5.GetRobotData();
        
        stRobotData_1->pathcheck = true;
        stRobotData_1->WaypointStackpopped =  false;
        stRobotData_1->CollinearFlag =  false;
        
 
        cController5.SetMovement();
    }
}


/****************************************************************************************************************/
/* Function to stop all robots */
/****************************************************************************************************************/
void DSA_loop_functions::StopAllRobots()
{
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it)
    {
        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        BaseController& cController = dynamic_cast<BaseController&>(cFootBot.GetControllableEntity().GetController());
        
        cController.SetHardStopMovement();
        cController.SetStopMovement();
        
    }
}

/* *********************************************************************************************/
// Function to get the neighbors of a particular robot
/* *********************************************************************************************/
void DSA_loop_functions::GetNeighbor_ThisRobot(BaseController::RobotData *ptr)
{
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataOther = NULL;
    argos::UInt16 i;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();

    
        /* get handle to every other robot */
        for(iterator2 = m_cFootbots.begin(); iterator2 != m_cFootbots.end(); ++iterator2)
        {
            
            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBotOther = *any_cast<CFootBotEntity*>(iterator2->second);
            BaseController& cControllerOther = dynamic_cast<BaseController&>(cFootBotOther.GetControllableEntity().GetController());
            stRobotDataOther = &cControllerOther.GetRobotData();
            stRobotDataOther->StartWaypoint = cControllerOther.GetPosition();
            
            //avoid comparing the robot to itself
            if(stRobotDataOther->id_robot != ptr->id_robot)
            {
                DistanceBetweenRobots = CalculateDistance(ptr->StartWaypoint, stRobotDataOther->StartWaypoint);
                // if the distance between two robots is less than the user configured
                if(DistanceBetweenRobots <= Neighbor_Radius)
                {
                    ptr->Neighbors.push_back(stRobotDataOther->id_robot);
                    stRobotDataOther->Neighbors.push_back(stRobotDataThis->id_robot);
                }
            }
        }
        
        
  

}
/****************************************************************************************************************/
/* Function to get the neighbors of the robot */
/****************************************************************************************************************/
void DSA_loop_functions::GetNeighbors()
{
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataOther = NULL;
    argos::UInt16 i;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    
//    for(i = (RobotResource.size()-1); i > 0; i--)
    for(i = 0; i < RobotResource.size(); i++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, RobotResource[i]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        stRobotDataThis = &cControllerThis.GetRobotData();
        stRobotDataThis->StartWaypoint = cControllerThis.GetPosition();
        
        /* get handle to every other robot */
        for(iterator2 = m_cFootbots.begin(); iterator2 != m_cFootbots.end(); ++iterator2)
        {

            /* Get handle to foot-bot entity and controller of the current robot */
            CFootBotEntity& cFootBotOther = *any_cast<CFootBotEntity*>(iterator2->second);
            BaseController& cControllerOther = dynamic_cast<BaseController&>(cFootBotOther.GetControllableEntity().GetController());
            stRobotDataOther = &cControllerOther.GetRobotData();
            stRobotDataOther->StartWaypoint = cControllerOther.GetPosition();

            //avoid comparing the robot to itself
            if(stRobotDataOther->id_robot != stRobotDataThis->id_robot)
            {
                DistanceBetweenRobots = CalculateDistance(stRobotDataThis->StartWaypoint, stRobotDataOther->StartWaypoint);
                // if the distance between two robots is less than the user configured
                if(DistanceBetweenRobots <= Neighbor_Radius)
                {
                    stRobotDataThis->Neighbors.push_back(stRobotDataOther->id_robot);
                    stRobotDataOther->Neighbors.push_back(stRobotDataThis->id_robot);
                }
            }
        }
      
        
    }

}


/****************************************************************************************************************/
/* Function to Initialize matrix */
/****************************************************************************************************************/
void DSA_loop_functions::InitializeMatrix(BaseController::RobotData *ptr1, argos::UInt8 dimension)
{
    ptr1->NeighborsMatrix.resize(dimension , vector<argos::UInt8>( dimension , 0));
}

/****************************************************************************************************************/
/* Function to set a matrix [row][column]  and [column][row] element to passed value*/
/****************************************************************************************************************/
void DSA_loop_functions::InitializeMatrixElementAndTransformElement(BaseController::RobotData *ptr,argos::UInt8 row,
                                                                    argos::UInt8 column, argos::UInt8 value)
{
    ptr->NeighborsMatrix[row][column] = value;
    ptr->NeighborsMatrix[column][row] = value;
}

/****************************************************************************************************************/
/* Function for a module that all possibilities of intersection of two robots */
/* BaseController::RobotData *ptr: Ptr to the matrix
 * BaseController::RobotData *ptr1: Ptr to the 1st robot
 * BaseController::RobotData *ptr2: Ptr to the 2nd robot
 std::vector<BaseController::IntersectionData> *ptr3: Ptr to intersection data of 1st robot
 std::vector<BaseController::IntersectionData> *ptr4: Ptr to intersection data of 2nd robot
 argos::UInt8 ptr1_index : Matrix index of the 1st robot
 argos::UInt8 ptr2_index: Matrix index of the 2nd robot */
 
/****************************************************************************************************************/
void DSA_loop_functions::IntersectionCheckModule(BaseController::RobotData *ptr, BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                 std::vector<BaseController::IntersectionData> *ptr3,
                                                 std::vector<BaseController::IntersectionData> *ptr4,
                                                 argos::UInt8 ptr1_index,argos::UInt8 ptr2_index)
{

    argos::UInt8 IntersectionFlag_ret = 0;
    
    RoboId1 = ptr1->id_robot;
    RoboId2 = ptr2->id_robot;
    
    // get target points at a safer distance for neighbor robot only if its not going in/ out of nest
    if(ptr1->GoingToOrFromNest == true and ptr2->GoingToOrFromNest == false)
    {
        GetPointAtSafeDistance(ptr2);
        IntersectionFlag_ret = Find_Intersection(ptr1->StartWaypoint, ptr1->TargetWaypoint,
                                                 ptr2->StartWaypoint, ptr2->IntersectionPt1, ptr3, ptr4,
                                                 ptr1, ptr2, false);
        
        // if the robots are intersecting, update the matrix
        if(IntersectionFlag_ret == 1)
        {
            IntersectionLoopValue = 1;
            InitializeMatrixElementAndTransformElement(ptr,ptr1_index,ptr2_index,INTERSECTION1);
        }
        else
        {
            
            IntersectionFlag_ret = Find_Intersection(ptr1->StartWaypoint, ptr1->TargetWaypoint,
                                                     ptr2->IntersectionPt1, ptr2->IntersectionPt2,
                                                     ptr3, ptr4, ptr1, ptr2, false);
            
            // if the robots are intersecting, update the matrix
            if(IntersectionFlag_ret == 1)
            {
                IntersectionLoopValue = 2;
                InitializeMatrixElementAndTransformElement(ptr,ptr1_index,ptr2_index,INTERSECTION2);
            }
        }
        
    }
    // get two points to check intersection of other robot
    else if(ptr2->GoingToOrFromNest == true and ptr1->GoingToOrFromNest == false)
    {
        GetPointAtSafeDistance(ptr1);
//        IntersectionFlag_ret = Find_Intersection(ptr2->StartWaypoint, ptr2->TargetWaypoint,
//                                                 ptr1->StartWaypoint, ptr1->IntersectionPt1, ptr3, ptr2->id_robot);
        
        IntersectionFlag_ret = Find_Intersection(ptr2->StartWaypoint, ptr2->TargetWaypoint,
                                                 ptr1->StartWaypoint, ptr1->IntersectionPt1, ptr4 ,ptr3,
                                                 ptr2, ptr1, false);

        
        // if the robots are intersecting, update the matrix
        if(IntersectionFlag_ret == 1)
        {
            IntersectionLoopValue = 3;
            InitializeMatrixElementAndTransformElement(ptr,ptr1_index,ptr2_index,INTERSECTION1);
        }
        else
        {
            IntersectionFlag_ret = Find_Intersection(ptr2->StartWaypoint, ptr2->TargetWaypoint,
                                                     ptr1->IntersectionPt1, ptr1->IntersectionPt2, ptr4, ptr3,
                                                     ptr2, ptr1, false);
            
            // if the robots are intersecting, update the matrix
            if(IntersectionFlag_ret == 1)
            {
                IntersectionLoopValue = 4;
                InitializeMatrixElementAndTransformElement(ptr,ptr1_index,ptr2_index,INTERSECTION2);
            }
        
        }
    }

}


/****************************************************************************************************************/
/* Function to reset all robot data */
/****************************************************************************************************************/
void DSA_loop_functions::CheckCollisionWithNeighbors(bool CheckOnlyCollinearity)
{
    argos::UInt16 NeighborsCount = 0;
    argos::UInt8 robotresourceindex, robotneighborindex, neighbor, nextneighbor;
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNeighbor = NULL;
    BaseController::RobotData *stRobotDataNeighbor1st = NULL;
    BaseController::RobotData *stRobotDataNeighbor2nd = NULL;
    
//    BaseController::IntersectionData *stIntersectionDataThis = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataThis = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor1st = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor2nd = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator3 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator4 = m_cFootbots.begin();
    
    // update neighbor matrix for every robot that has collected resource
    for(robotresourceindex = 0; robotresourceindex < RobotResource.size(); robotresourceindex++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, RobotResource[robotresourceindex]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        // Get Intersection Data
        stIntersectionDataThis = cControllerThis.GetIntersectionData();

        
        // count the number of neighbors for the robot
        NeighborsCount = stRobotDataThis->Neighbors.size();
        // if the robot has neighbors
        if(NeighborsCount > 0)
        {
            /* if neighbors matrix exist, clear it before reinitializig the matrix */
            if(stRobotDataThis->NeighborsMatrix.size() > 0)
            {
                stRobotDataThis->NeighborsMatrix.clear();
            }
            // resize the matrix depending on the neighbor size
            // add 1 to get the row and column for robot itself
            InitializeMatrix(stRobotDataThis,(NeighborsCount + 1));
            
            // update the matrix by checking the collinearity and intersection of robot with its neighbors
            for(robotneighborindex = 0; robotneighborindex < stRobotDataThis->Neighbors.size(); robotneighborindex++)
            {
                iterator2 = m_cFootbots.begin();
                
                // get the handle to neighbor robot
                std::advance(iterator2, stRobotDataThis->Neighbors[robotneighborindex]);
                CFootBotEntity& cFootBotNeighbor = *any_cast<CFootBotEntity*>(iterator2->second);
                BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbor.GetControllableEntity().GetController());
                stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();
                stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();

                stRobotDataThis->CollinearFlag = false;
                stRobotDataNeighbor->CollinearFlag = false;

                // check if the robots are collinear
                // ptr to the robot data of both the robots
                if(CheckOnlyCollinearity == true)
                {
                    /* Do nothing as collinearity check already implemented */
                }
                // check for intersection
                else if(CheckOnlyCollinearity == false)
                {

                   // ptr for matrix to update, ptr to robot data of both robots, matrix indices of both the robots
                   IntersectionCheckModule(stRobotDataThis,stRobotDataThis, stRobotDataNeighbor,
                                           stIntersectionDataThis,stIntersectionDataNeighbor, 0,(robotneighborindex+1));

                }
            }// end of for robotneighborindex
        
            
            // fill the matrix for neighbors vs neighbors
            for(neighbor = 0; neighbor < stRobotDataThis->Neighbors.size(); neighbor++)
            {
                iterator3 = m_cFootbots.begin();
                std::advance(iterator3, stRobotDataThis->Neighbors[neighbor]);
                CFootBotEntity& cFootBotNeighbor1st = *any_cast<CFootBotEntity*>(iterator3->second);
                BaseController& cControllerNeighbor1st = dynamic_cast<BaseController&>(cFootBotNeighbor1st.GetControllableEntity().GetController());
                stRobotDataNeighbor1st = &cControllerNeighbor1st.GetRobotData();
                stIntersectionDataNeighbor1st = cControllerNeighbor1st.GetIntersectionData();
                
                //  get the data for next neighbor and check for intersection or collision
                for(nextneighbor = neighbor+1; nextneighbor <= (stRobotDataThis->Neighbors.size() - 1); nextneighbor++)
                {
                    iterator4 = m_cFootbots.begin();
                    std::advance(iterator4, stRobotDataThis->Neighbors[nextneighbor]);
                    
                    CFootBotEntity& cFootBotNeighbor2nd = *any_cast<CFootBotEntity*>(iterator4->second);
                    BaseController& cControllerNeighbor2nd = dynamic_cast<BaseController&>(cFootBotNeighbor2nd.GetControllableEntity().GetController());
                    stRobotDataNeighbor2nd = &cControllerNeighbor2nd.GetRobotData();
                    stIntersectionDataNeighbor2nd = cControllerNeighbor2nd.GetIntersectionData();
                    
                    stRobotDataNeighbor1st->CollinearFlag = false;
                    stRobotDataNeighbor2nd->CollinearFlag = false;

                    if(CheckOnlyCollinearity == true)
                    {
                        /* Do nothing as collinearity check already implemented */
                    }
                    else if(CheckOnlyCollinearity == false)
                    {

                        // Update the matrix for intersection of robots
                        IntersectionCheckModule(stRobotDataThis,stRobotDataNeighbor1st, stRobotDataNeighbor2nd,
                                                stIntersectionDataNeighbor1st, stIntersectionDataNeighbor2nd, (neighbor+1),(neighbor+2));
                    }
                    
                
                }
            } // end of outer for for  neighbors vs neighbors
            
        } // end of if
    } // end of for for robotresourceindex
         
  
    
}




/****************************************************************************************************************/
/* Function to check if all elemnts in multidimensional matrix are zeros */
/****************************************************************************************************************/
bool DSA_loop_functions::IsMatrixConsistent(BaseController::RobotData *ptr)
{
    argos::UInt16 rows, columns, row_count, column_count;
    bool IsNotConsistent = false;
    
    // get number of rows
    rows = ptr->NeighborsMatrix.size();
    // get number of columns
    if(rows > 0)
    {
        columns = ptr->NeighborsMatrix[0].size();
    }
    // check each row
    for(row_count = 0; row_count<rows; row_count++)
    {
        // check column in each row
        for(column_count= 0; column_count<columns; column_count++)
        {
            IsNotConsistent |= ptr->NeighborsMatrix[row_count][column_count];
        }
    }
    
    //returns true if not consistent, returns false if consistent
    return IsNotConsistent;
}



/************************************************************************************************************************************************************************************/
/* Function to check which anchor point is closest to the collinear robots */
/************************************************************************************************************************************************************************************/
argos::Real DSA_loop_functions::CheckDirection(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
    /* To determine which side of the line from
    𝐴=(𝑥1,𝑦1) to 𝐵=(𝑥2,𝑦2) a point 𝑃=(𝑥,𝑦)
    
    falls on you need to compute the value:-
    𝑑=(𝑥−𝑥1)(𝑦2−𝑦1)−(𝑦−𝑦1)(𝑥2−𝑥1)
    
    
    If 𝑑<0  then the point lies on one side of the line, and if
        𝑑>0 then it lies on the other side. If 𝑑=0 then the point lies exactly line. */
    
    argos::Real direction;
    
    direction = (ptr2->StartWaypoint.GetX() - ptr1->StartWaypoint.GetX())*(ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()) -
        (ptr2->StartWaypoint.GetY() - ptr1->StartWaypoint.GetY())*(ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX());
    
    return direction;
}

/**********************************************************************************************************************************************************************************************************/
/* Function to determine if the calculated direction is left or right */
/**********************************************************************************************************************************************************************************************************/
argos::UInt8 DSA_loop_functions::DecideLeftOrRight(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, argos::Real direction)
{
    argos::Real dir_right, dir_left;
    argos::CVector2 LeftPoint, RightPoint;
    argos::UInt8 actual_direction;
    
    RightPoint.Set(ptr2->StartWaypoint.GetX(), ptr2->StartWaypoint.GetY() - 0.01);
    LeftPoint.Set(ptr2->StartWaypoint.GetX(), ptr2->StartWaypoint.GetY() + 0.01);
    
    dir_right = (RightPoint.GetX() - ptr1->StartWaypoint.GetX())*(ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()) -
    (RightPoint.GetY() - ptr1->StartWaypoint.GetY())*(ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX());
    
    dir_left = (LeftPoint.GetX() - ptr1->StartWaypoint.GetX())*(ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()) -
    (LeftPoint.GetY() - ptr1->StartWaypoint.GetY())*(ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX());
    
    /* direction is right */
    if(dir_right < 0.0 and direction < 0.0)
    {
        actual_direction = 1;
    }
    /* direction is left */
    else if(dir_left > 0.0 and  direction > 0.0)
    {
        actual_direction = 2;
    }
    /* direction is right */
    else if(dir_right > 0.0 and direction > 0.0)
    {
        actual_direction = 1;
    }
      /* direction is left */
    else if(dir_left < 0.0 and direction < 0.0)
    {
         actual_direction = 2;
    }
    else{
        actual_direction = 0;
    }
    
    return actual_direction;
}

/************************************************************************************************************************************************************************************/
argos::CVector2 DSA_loop_functions::GetWayPointClusterMode(BaseController::RobotData *ptr1)
{
    
    argos::CRadians VectorAngle, theta, CosAngle;
    argos::Real TurnAngle, Angle1, ret_direction;
    argos::Real VectorLength, Vector1Length;
    argos::UInt8 LeftorRight;
    argos::CVector2 ResultantVector, Waypoint, RotatedVector, PointOnNest;

    argos::Real distance, dotprod;
    
    argos::CVector2 PerpendicularVec;
    
    
    Waypoint.Set(0,0);


    // get a point on the nest circumference
    PointOnNest = CalculatePointAtDistanceAlongVectorDirection(ptr1->TargetWaypoint, ptr1->StartWaypoint, NestRadius);
    
    
    ResultantVector = (PointOnNest - NestPosition);
    VectorLength = (ResultantVector).Length();


    VectorAngle = (ResultantVector).Angle();

    TurnAngle = ToDegrees(VectorAngle).GetValue() + Rotation_Angle;


    theta = ToRadians(CDegrees(TurnAngle));
    
    
    Waypoint.SetX((NestRadius + WaypointDistance) * Cos(theta));
    Waypoint.SetY((NestRadius + WaypointDistance) * Sin(theta));
    
    
    return Waypoint;
}

/********************************************************************************************************************************************************************************************************************/
argos::CVector2 DSA_loop_functions::CalculateWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, UInt8 turntype)
{
    argos::Real AngleRotationValue = 70.0f;
    argos::CRadians VectorAngle, theta;
    argos::Real TurnAngle, ret_direction;
    argos::Real VectorLength;
    argos::UInt8 LeftorRight;
    argos::CVector2 ResultantVector, Waypoint, RotatedVector, PointOnNest;
    argos::Real Waypt_dist;
    argos::Real distance;
    
    Waypt_dist = 0.65;
    

    if(turntype == 1)
    {
        if(CLUSTERCONFIGONLY == false)
        {
            ResultantVector = (ptr2->StartWaypoint - ptr2->TargetWaypoint);
        }
        else
        {
            ResultantVector = (ptr2->TargetWaypoint - ptr2->StartWaypoint);
        }
        VectorAngle = (ResultantVector).Angle();
        VectorLength = (ResultantVector).Length();

        TurnAngle = ToDegrees(VectorAngle).GetValue() + AngleRotationValue;
     
        theta = ToRadians(CDegrees(TurnAngle));
        
        // get a point on the nest circumference
        PointOnNest = CalculatePointAtDistanceAlongVectorDirection(ptr2->TargetWaypoint, ptr2->StartWaypoint, NestRadius);
        
        
        Waypoint.SetX((Waypt_dist) * Cos(theta) + PointOnNest.GetX());
        Waypoint.SetY((Waypt_dist) * Sin(theta) + PointOnNest.GetY());


        
    }
    else if(turntype == 2)
    {
        ResultantVector = (ptr1->TargetWaypoint - ptr1->StartWaypoint);
        VectorAngle = (ResultantVector).Angle();
        VectorLength = (ResultantVector).Length();
        
        TurnAngle = ToDegrees(VectorAngle).GetValue() + AngleRotationValue;
        theta = ToRadians(CDegrees(TurnAngle));
        
        Waypoint.SetX(Waypt_dist * Cos(theta) + ptr1->StartWaypoint.GetX());
        Waypoint.SetY(Waypt_dist * Sin(theta) + ptr1->StartWaypoint.GetY());
    }
    // robot is away from nest
    else if(turntype == 3)
    {
        AngleRotationValue = 30.0f;
        ResultantVector = (ptr1->TargetWaypoint - ptr1->StartWaypoint);

        VectorAngle = (ResultantVector).Angle();
        VectorLength = (ResultantVector).Length();

        TurnAngle = ToDegrees(VectorAngle).GetValue() + AngleRotationValue;

        theta = ToRadians(CDegrees(TurnAngle));
        
        distance = (ptr1->StartWaypoint - NestPosition).Length();

//        Waypoint.SetX((0.25) * Cos(theta) + ptr1->StartWaypoint.GetX());
//        Waypoint.SetY((0.25) * Sin(theta) + ptr1->StartWaypoint.GetY());
        
        Waypoint.SetX((0.4) * Cos(theta) + NestPosition.GetX());
        Waypoint.SetY((0.4) * Sin(theta) + NestPosition.GetY());
  
        
    }
    else
    {
        ResultantVector = (ptr1->TargetWaypoint - ptr1->StartWaypoint);
        VectorAngle = (ResultantVector).Angle();
        VectorLength = (ResultantVector).Length();
        ret_direction = CheckDirection(ptr2, ptr1);

        LeftorRight = DecideLeftOrRight(ptr2, ptr1, ret_direction);

        // if ret_direction is 2, the robot that needs to got to waypoint is to the right
        if(LeftorRight == 1)
        {
            //turn the robot to right
            TurnAngle = (ToDegrees(VectorAngle).GetValue() - AngleRotationValue);
            theta = ToRadians(CDegrees(TurnAngle));


        }
        // if ret_direction is 2, the robot that needs o got to waypoint is to the left
        else if(LeftorRight == 2)
        {
            //turn the robot to left
            TurnAngle = ToDegrees(VectorAngle).GetValue() + AngleRotationValue;
            theta = ToRadians(CDegrees(TurnAngle));
        }

        
        Waypoint.SetX(Waypt_dist * Cos(theta) + ptr1->StartWaypoint.GetX());
        Waypoint.SetY(Waypt_dist * Sin(theta) + ptr1->StartWaypoint.GetY());
    }
    
    return Waypoint;
    
    
}

/*****************************************************************************************************************/
/* Function To Calculate Waypoint */
/*****************************************************************************************************************/
argos::CVector2 DSA_loop_functions::CalculatePointAtDistanceAlongVectorDirection(argos::CVector2 Point1, argos::CVector2 Point2, argos::Real Distance)
{
    
    argos::CVector2 VectorAtDistance;
    
    argos::CVector2 ResultantVector = (Point2 - Point1);
    argos::Real vectorlength = (ResultantVector).Length();
    
    argos::CVector2 NormalizedVector = ResultantVector.Normalize();
    argos::CVector2 du = NormalizedVector * Distance;
    
    VectorAtDistance = du + Point1;
    return VectorAtDistance;
}


/*********************************************************************************************************************************************/
/* Function To Calculate Points Along the circle */
/*********************************************************************************************************************************************/
void DSA_loop_functions::CalculatePointsAlongCircle(argos::CVector2 *Array, BaseController::RobotData *ptr, argos::Real radius, bool CentreNest)
{
    argos::CVector2 &RefArray = *Array;
    argos::CRadians Angle;
    argos::CVector2 Horizontal_Point, Centre, Diagonal_Point, VerticalPoint;


    
    if(CentreNest == true)
    {
        Centre.Set(0,0);
    }
    else
    {
        Centre.Set(ptr->StartWaypoint.GetX(), ptr->StartWaypoint.GetY());
    }
    
    Horizontal_Point.SetX(radius * Cos(ToRadians(argos::CDegrees(0.0f))) + Centre.GetX());
    Horizontal_Point.SetX(radius * Sin(ToRadians(argos::CDegrees(0.0f))) + Centre.GetY());
    
    VerticalPoint.SetX(radius * Cos(ToRadians(argos::CDegrees(90.0f))) + Centre.GetX());
    VerticalPoint.SetX(radius * Sin(ToRadians(argos::CDegrees(90.0f))) + Centre.GetY());
    
    Diagonal_Point.SetX(radius * Cos(ToRadians(argos::CDegrees(45.0f))) + Centre.GetX());
    Diagonal_Point.SetX(radius * Sin(ToRadians(argos::CDegrees(45.0f))) + Centre.GetY());
    
    Array[0].Set(Horizontal_Point.GetX(), Horizontal_Point.GetY());
    Array[1].Set(Horizontal_Point.GetX(), -Horizontal_Point.GetY());
    Array[2].Set(VerticalPoint.GetX(), VerticalPoint.GetY());
    Array[3].Set(-VerticalPoint.GetX(), VerticalPoint.GetY());
    Array[4].Set(Diagonal_Point.GetX(), Diagonal_Point.GetY());
    Array[5].Set(Diagonal_Point.GetX(), -Diagonal_Point.GetY());
    Array[6].Set(-Diagonal_Point.GetX(), -Diagonal_Point.GetY());
    Array[7].Set(-Diagonal_Point.GetX(), Diagonal_Point.GetY());
}



/*************************************************************************************************************************************************************************************************************/
/* Function to check which anchor point is closest to the collinear robots */
/*************************************************************************************************************************************************************************************************************/
argos::CVector2 DSA_loop_functions::FindClosestAnchorPoint(BaseController::RobotData *ptr, BaseController::RobotData *Otherptr, bool AtNest)
{
    argos::Real Distance;
    argos::Real DistancePoint1, DistancePoint2, DistancePoint3, DistancePoint4, MaxDistance;
    argos::Real SmallerDistance, CurrentDistance, RadiusDistance;
    argos::CVector2  ClosestAnchorPoint, Waypoint;
    bool CollinearFlag1, CollinearFlag2;
    argos::Real DistanceArray[5];
    argos::UInt8 CollinearArray[5];
    
    argos::CVector2 AnchorPointArray[8];
    argos::UInt8 index, index1, index2, SmallestDistanceIndex, i;
    int sign_y, sign_x, anchorpoint_sign_x, anchorpoint_sign_y;
    MaxDistance = 1000;
    
    RadiusDistance = 0.27;
    
    
    if(ptr->StartWaypoint.GetY() >= 0)
    {
        sign_y = 1;
    }
    else
    {
        sign_y = -1;
    }
    
    
    CollinearFlag1= false;
    CollinearFlag2 = false;
    
    
    if(ptr->TargetWaypoint.GetY() < 0)
    {
        for(i=0; i < 5; i++)
        {
            Waypoint = CalculatePointAtDistanceAlongVectorDirection(NestPosition, AnchorPointsHalfNeg[i], RadiusDistance);

            AnchorPointArray[i] = Waypoint;
        }

    }
    else
    {
        for(i=0; i < 5; i++)
        {
            Waypoint = CalculatePointAtDistanceAlongVectorDirection(NestPosition, AnchorPointsHalfPos[i], RadiusDistance);
            AnchorPointArray[i] = Waypoint;
        }
        
    }
    
    // calculate the distance
    for(index = 0; index< 5; index++)
    {
        Distance = CalculateDistance(ptr->StartWaypoint, AnchorPointArray[index]);

        DistanceArray[index] = Distance;
    }
    
    // Check if any of the anchor point is collinear
    for(index1 = 0; index1 < 5; index1++)
    {
        if(AnchorPointArray[index1].GetX() >= 0)
        {
            anchorpoint_sign_x = 1;
        }
        else{
            anchorpoint_sign_x = -1;
        }
        
        /* if the waypoint is in the same quadtrant as the target point of the other robot */
        if(anchorpoint_sign_x != sign_x or AnchorPointArray[index1].GetX() == 0)
        {
            
            CollinearFlag1 = WayPointCollinearityCheck(ptr->StartWaypoint, AnchorPointArray[index1] , Otherptr->StartWaypoint, Otherptr->TargetWaypoint,
                                                       ptr, Otherptr);
            
            CollinearFlag2 = WayPointCollinearityCheck(AnchorPointArray[index1], ptr->TargetWaypoint , Otherptr->StartWaypoint, Otherptr->TargetWaypoint,
                                                       ptr, Otherptr);
            
            
            
            if(CollinearFlag1 | CollinearFlag2)
            {
                CollinearArray[index1] = 1;
                CollinearFlag1= false;
                CollinearFlag2 = false;
                
            }
            else
            {
                CollinearArray[index1] = 0;
                
            }
        }
        else{
            CollinearArray[index1] = 1;
        }
    }
    
    SmallerDistance = 1000;
    SmallestDistanceIndex = 0;
    
    
    // Check which is the smallest distance
    for(index2 = 0; index2 < 5; index2++)
    {
        CurrentDistance = DistanceArray[index2];
        
        
        // if the current index distance is smaller
        if(CurrentDistance < SmallerDistance)
        {
            if(CollinearArray[index2] == 0)
            {
                SmallestDistanceIndex = index2;
                SmallerDistance = CurrentDistance;
            }
            else
            {
                /* Do nothing */
            }
        }
        // it is greater
        else
        {
            /* Do nothing */
        }
        
        
    }
    ClosestAnchorPoint = AnchorPointArray[SmallestDistanceIndex];
    
    
    return ClosestAnchorPoint;
    
}

/****************************************************************************************************************/
/* Function to add waypoint while coming out of nest */
/****************************************************************************************************************/
void DSA_loop_functions::AddWayPoint(BaseController::RobotData *ptr)
{
    
    if(ptr->WaypointStack.empty())
    {
        ptr->WaypointStack.push(ptr->TargetWaypoint);

    }

    
    
    
}

/***************************************************************************************************************************************************************************************************/
/* Function to add waypoint while coming out of nest */
/***************************************************************************************************************************************************************************************************/
bool DSA_loop_functions::MatrixCollinearityIntersectionEquation(BaseController::RobotData *ptr, argos::UInt8 row_size, argos::UInt8 column_size)
{
    argos::UInt8 rows, columns, Value;
    bool collinearvalue, intersectionvalue;
    bool CollinearCheck;
    
    CollinearCheck = false;
    collinearvalue = false;
    intersectionvalue = false;
    for(rows = 0; rows< row_size; rows++)
    {
        for(columns = 0; columns< column_size; columns++)
        {
            Value = ptr->NeighborsMatrix[rows][columns];
            if(Value == COLLINEAR)
            {
                CollinearCheck = true;
            }

        }
    }
    
    return CollinearCheck;
    
}

/************************************************************************************************************************************************/
/* Function to check if three points are collinear */
/************************************************************************************************************************************************/
bool DSA_loop_functions::ThreePointsCollinear(CVector2 Point1, CVector2 Point2, CVector2 Point3)
{
    argos::UInt16 Area;
    bool Collinear = false;
    
    Area = Point1.GetX() * (Point2.GetY() - Point3.GetY()) +
           Point2.GetX() * (Point3.GetY() - Point1.GetY()) +
           Point3.GetX() * (Point1.GetY() - Point2.GetY());
    
    // if area is 0, then points are collinear
    if(0 < Area and Area > 0.1)
    {
        Collinear = true;
    }
    
    return Collinear;
}


/********************************************************************************************************************************************************************************************/
/* Function to check if three points are collinear */
/********************************************************************************************************************************************************************************************/
void DSA_loop_functions::IntersectionHandling_ForRobotsGoingOutOfNest_Collinear(BaseController::RobotData *Robot1, BaseController:: RobotData *Robot2,
                                                                     std::vector<BaseController::IntersectionData> *stRobotIntersectionData1,
                                                                     std::vector<BaseController::IntersectionData> *stRobotIntersectionData2)
{
    
    argos::UInt16 TimeToReachTarget1, TimeToReachTarget2;
    argos::UInt8 RobotToStop, IntersectionType;
    argos::UInt8 IntersectionFlag, TimeToTurn1, TimeToTurn2, TimeToGoStraight;
    argos::Real Distance;
    BaseController::IntersectionData* RobotIntersectionData = NULL;
    BaseController::IntersectionData* NeighborRobotIntersectionData = NULL;
    BaseController::RobotData *ptr = NULL;
    BaseController::RobotData *ptr2 = NULL;
    
    std::vector<BaseController::IntersectionData> *RobotIntersectionData1 = NULL;
    std::vector<BaseController::IntersectionData> *RobotIntersectionData2 = NULL;
    
    
    IntersectionType = COLLINEAR_INTERSECTION;
   

    // check if the robot is intersecting with other robots
    IntersectionFlag = Find_Intersection(Robot1->StartWaypoint, Robot1->TargetWaypoint, Robot2->StartWaypoint ,
                                         Robot2->TargetWaypoint,
                                         stRobotIntersectionData1, stRobotIntersectionData2, Robot1, Robot2, true);
    
    if(IntersectionFlag == 1)
    {
        
        
        RobotIntersectionData = GetIntersectionDataFromVector(stRobotIntersectionData1,
                                                              Robot2->id_robot, IntersectionType);
        
        
        NeighborRobotIntersectionData = GetIntersectionDataFromVector(stRobotIntersectionData2,
                                                                      Robot1->id_robot, IntersectionType);
        
        
        IntersectionCollisionCheck(Robot1, Robot2, RobotIntersectionData, NeighborRobotIntersectionData,
                                   INTERSECTION1, 3);

    }
    else if(IntersectionFlag == 2)
    {
        if(CalculateDistance(Robot1->StartWaypoint, Robot1->TargetWaypoint) < CalculateDistance(Robot2->StartWaypoint, Robot2->TargetWaypoint))
        {
            ptr = Robot1;
            ptr2 = Robot2;
        }
        else
        {
            ptr = Robot2;
            ptr2 = Robot1;
        }
        /* Add stop time to the robot going towards nest */
        argos::CRadians Headingangle = (ptr->StartWaypoint - ptr->TargetWaypoint).Angle();
        argos::CRadians TurningAngle = (ptr->Orientation - Headingangle).SignedNormalize();
        argos::Real TurningAngle_Deg = ToDegrees(TurningAngle).GetValue();
        
        if(TurningAngle_Deg < 0.0f)
        {
            TurningAngle_Deg = -TurningAngle_Deg;
        }
        else
        {
            TurningAngle_Deg = TurningAngle_Deg;
        }
        TimeToTurn1 = GetTimeToTurn(TurningAngle_Deg, ptr->fBaseAngularWheelSpeed) + 20;
        
        TimeToTurn2 = GetTimeToTurn(90, ptr->fBaseAngularWheelSpeed) + 20;
        
        Distance = CalculateDistance(ptr->StartWaypoint, ptr->AddedPoint);
        TimeToGoStraight = GetTicksToWait(Distance, ptr->fLinearWheelSpeed) + 20;
        
        ptr2->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
        
    }/* end of else if(IntersectionFlag == 2) */
    
    else
    {
        TimeToReachTarget1 = GetTicksToWait((CalculateDistance(Robot1->StartWaypoint, Robot1->TargetWaypoint)),
                                            Robot1->fLinearWheelSpeed) + 50;
        
        TimeToReachTarget2 = GetTicksToWait((CalculateDistance(Robot2->StartWaypoint, Robot2->TargetWaypoint)),
                                            Robot2->fLinearWheelSpeed) + 50;
        
        // robot1 is reaching the target 1st
        if(TimeToReachTarget1 < TimeToReachTarget2)
        {
            RobotToStop = 1;
            ptr = Robot1;
            ptr2 = Robot2;
            RobotIntersectionData1 = stRobotIntersectionData1;
            RobotIntersectionData2 = stRobotIntersectionData2;
        }
        else
        {
            RobotToStop = 2;
            ptr = Robot2;
            ptr2 = Robot1;
            RobotIntersectionData1 = stRobotIntersectionData2;
            RobotIntersectionData2 = stRobotIntersectionData1;
            
            
        }
        
        // get the points to check intersection
        GetPointAtSafeDistance(ptr);
        
        // check if the robot is intersecting with other robots
        IntersectionFlag = Find_Intersection(ptr->TargetWaypoint, ptr->IntersectionPt1, ptr2->StartWaypoint ,
                                             ptr2->TargetWaypoint,
                                             RobotIntersectionData1, RobotIntersectionData2, ptr, ptr2, true);
        
        if(IntersectionFlag == 1)
        {
            RobotIntersectionData = GetIntersectionDataFromVector(RobotIntersectionData1,
                                                                  ptr2->id_robot, IntersectionType);
            
            
            NeighborRobotIntersectionData = GetIntersectionDataFromVector(RobotIntersectionData2,
                                                                          ptr->id_robot, IntersectionType);
            
            IntersectionCollisionCheck(ptr, ptr2, RobotIntersectionData, NeighborRobotIntersectionData,
                                       INTERSECTION1, RobotToStop);
            
        }
        else if(IntersectionFlag == 2 )
        {
            /* Add stop time to the robot going towards nest */
            argos::CRadians Headingangle = (ptr->StartWaypoint - ptr->TargetWaypoint).Angle();
            argos::CRadians TurningAngle = (ptr->Orientation - Headingangle).SignedNormalize();
            argos::Real TurningAngle_Deg = ToDegrees(TurningAngle).GetValue();
            
            if(TurningAngle_Deg < 0.0)
            {
                TurningAngle_Deg = -TurningAngle_Deg;
            }
            else
            {
                TurningAngle_Deg = TurningAngle_Deg;
            }
            
            TimeToTurn1 = GetTimeToTurn(TurningAngle_Deg, ptr->fBaseAngularWheelSpeed) + 20;
            
            TimeToTurn2 = GetTimeToTurn(90, ptr->fBaseAngularWheelSpeed) + 20;
            
            Distance = CalculateDistance(ptr->StartWaypoint, ptr->TargetWaypoint);
            TimeToGoStraight = GetTicksToWait(Distance, ptr->fLinearWheelSpeed) + 20;
            
            ptr2->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
            
        }
        else
        {
            // check if the robot is intersecting with other robots
            IntersectionFlag = Find_Intersection(ptr->IntersectionPt1, ptr->IntersectionPt2, ptr2->StartWaypoint ,
                                                 ptr2->TargetWaypoint,
                                                 RobotIntersectionData1, RobotIntersectionData1, ptr, ptr2, true);
            if(IntersectionFlag == 1)
            {
                RobotIntersectionData = GetIntersectionDataFromVector(RobotIntersectionData1,
                                                                      ptr2->id_robot, IntersectionType);
                
                
                NeighborRobotIntersectionData = GetIntersectionDataFromVector(RobotIntersectionData2,
                                                                              ptr->id_robot, IntersectionType);
                
              
                IntersectionCollisionCheck(ptr, ptr2, RobotIntersectionData, NeighborRobotIntersectionData,
                                       INTERSECTION2, RobotToStop);
           
            }
            else if(IntersectionFlag == 2)
            {
                /* Add stop time to the robot going towards nest */
                argos::CRadians Headingangle = (ptr->StartWaypoint - ptr->TargetWaypoint).Angle();
                argos::CRadians TurningAngle = (ptr->Orientation - Headingangle).SignedNormalize();
                
                argos::Real TurningAngle_Deg = ToDegrees(TurningAngle).GetValue();
                
                if(TurningAngle_Deg < 0.0)
                {
                    TurningAngle_Deg = -TurningAngle_Deg;
                }
                else
                {
                    TurningAngle_Deg = TurningAngle_Deg;
                }
                
                
                TimeToTurn1 = GetTimeToTurn(TurningAngle_Deg, ptr->fBaseAngularWheelSpeed) + 20;
                
                TimeToTurn2 = GetTimeToTurn(90, ptr->fBaseAngularWheelSpeed) + 20;
                
                Distance = CalculateDistance(ptr->StartWaypoint, ptr->TargetWaypoint);
                TimeToGoStraight = GetTicksToWait(Distance, ptr->fLinearWheelSpeed) + 20;
                
                ptr2->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
                
            }
        }
        
        
    }
    
}



/************************************************************************************************************************************************************************************************************/
/* Function to check if three points are collinear */
/************************************************************************************************************************************************************************************************************/
void DSA_loop_functions::IntersectionCheck_ForRobots_GoingOutOfNest(BaseController::RobotData *Robot1, BaseController:: RobotData *Robot2,
                                                                                std::vector<BaseController::IntersectionData> *stRobotIntersectionData1,
                                                                                std::vector<BaseController::IntersectionData> *stRobotIntersectionData2,
                                                                    argos::UInt8 index_1, argos::UInt8 index_2, BaseController::RobotData *Baseptr)
{
    
    argos::UInt16 TimeToReachTarget1, TimeToReachTarget2;
    argos::UInt8 RobotToStop, IntersectionType;
    argos::UInt8 IntersectionFlag, TimeToTurn1, TimeToTurn2, TimeToGoStraight;
    argos::Real Distance;
    BaseController::IntersectionData* RobotIntersectionData = NULL;
    BaseController::IntersectionData* NeighborRobotIntersectionData = NULL;
    BaseController::RobotData *ptr = NULL;
    BaseController::RobotData *ptr2 = NULL;
    
    std::vector<BaseController::IntersectionData> *RobotIntersectionData1 = NULL;
    std::vector<BaseController::IntersectionData> *RobotIntersectionData2 = NULL;
    
    
    
    
    
    // check if the robot is intersecting with other robots
    IntersectionFlag = Find_Intersection(Robot1->StartWaypoint, Robot1->TargetWaypoint, Robot2->StartWaypoint ,
                                         Robot2->TargetWaypoint,
                                         stRobotIntersectionData1, stRobotIntersectionData2, Robot1, Robot2, false);
    
    if(IntersectionFlag == 1)
    {
        InitializeMatrixElementAndTransformElement(Baseptr,index_1,index_2,INTERSECTION1);

    }
    
    else
    {
        
        argos::CRadians Headingangle = (Robot1->StartWaypoint - Robot1->TargetWaypoint).Angle();
        argos::CRadians TurningAngle = (Robot1->Orientation - Headingangle).SignedNormalize();
        
        argos::Real TurningAngle_Deg = ToDegrees(TurningAngle).GetValue();
        
        if(TurningAngle_Deg < 0.0f)
        {
            TurningAngle_Deg = -TurningAngle_Deg;
        }
        else
        {
            TurningAngle_Deg = TurningAngle_Deg;
        }
        
        
        TimeToTurn1 = GetTimeToTurn(TurningAngle_Deg, Robot1->fBaseAngularWheelSpeed) + 20;
        
        TimeToReachTarget1 = GetTicksToWait((CalculateDistance(Robot1->StartWaypoint, Robot1->TargetWaypoint)),
                                            Robot1->fLinearWheelSpeed) + TimeToTurn1;
        
        argos::CRadians Headingangle1 = (Robot2->StartWaypoint - Robot2->TargetWaypoint).Angle();
        argos::CRadians TurningAngle1 = (Robot2->Orientation - Headingangle1).SignedNormalize();
        
        argos::Real TurningAngle1_Deg = ToDegrees(TurningAngle1).GetValue();
        
        if(TurningAngle1_Deg < 0.0f)
        {
            TurningAngle1_Deg = -TurningAngle1_Deg;
        }
        else
        {
            TurningAngle1_Deg = TurningAngle1_Deg;
        }
        
        
        TimeToTurn2 = GetTimeToTurn(TurningAngle1_Deg, Robot2->fBaseAngularWheelSpeed) + 20;
        
        TimeToReachTarget2 = GetTicksToWait((CalculateDistance(Robot2->StartWaypoint, Robot2->TargetWaypoint)),
                                            Robot2->fLinearWheelSpeed) + TimeToTurn2;
        
        // robot1 is reaching the target 1st
        if(TimeToReachTarget1 < TimeToReachTarget2)
        {
            RobotToStop = 1;
            ptr = Robot1;
            ptr2 = Robot2;
            RobotIntersectionData1 = stRobotIntersectionData1;
            RobotIntersectionData2 = stRobotIntersectionData2;
        }
        else
        {
            RobotToStop = 2;
            ptr = Robot2;
            ptr2 = Robot1;
            RobotIntersectionData1 = stRobotIntersectionData2;
            RobotIntersectionData2 = stRobotIntersectionData1;
            
            
        }
        
        // get the points to check intersection
        GetPointAtSafeDistance(ptr);
        
        // check if the robot is intersecting with other robots
        IntersectionFlag = Find_Intersection(ptr->TargetWaypoint, ptr->IntersectionPt1, ptr2->StartWaypoint ,
                                             ptr2->TargetWaypoint,
                                             RobotIntersectionData1, RobotIntersectionData2, ptr, ptr2, false);
        
        if(IntersectionFlag == 1)
        {
            
            InitializeMatrixElementAndTransformElement(Baseptr,index_1,index_2,INTERSECTION2);
        }
       
        else
        {
            // check if the robot is intersecting with other robots
            IntersectionFlag = Find_Intersection(ptr->IntersectionPt1, ptr->IntersectionPt2, ptr2->StartWaypoint ,
                                                 ptr2->TargetWaypoint,
                                                 RobotIntersectionData1, RobotIntersectionData1, ptr, ptr2, false);
            if(IntersectionFlag == 1)
            {
                
                 InitializeMatrixElementAndTransformElement(Baseptr,index_1,index_2,INTERSECTION2);
            }
            
        }
        
        
    }
    
}






/****************************************************************************************************************************************************************************************************************/
/* Function to handle intersection if a waypoint is added to te robot going out of nest in case of collinearity */
/****************************************************************************************************************************************************************************************************************/
void DSA_loop_functions::IntersectionHandlingForCollinearity(BaseController::RobotData *RobotDataptr,
                                                             std::vector<BaseController::IntersectionData> *stRobotIntersectionData)
{
    argos::CVector2 StartPoint, EndPoint;
    argos::UInt8 IntersectionFlag, TimeToTurn1, TimeToTurn2, TimeToGoStraight;
    argos::UInt8 NeighborSize, RobotToStop;
    argos::Real Distance;
    BaseController::RobotData *stRobotDataNeighbor = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor = NULL;
    
    BaseController::IntersectionData* RobotIntersectionData = NULL;
    BaseController::IntersectionData* NeighborRobotIntersectionData = NULL;
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    
  
    IntersectionFlag = 0;
    
    // if the robot is going to nest
    if(RobotDataptr->GoingToOrFromNest == false and RobotDataptr->GoingToNest == true)
    {
        RobotToStop = 1;
    }
    // robot is coming out of nest
    else if(RobotDataptr->GoingToOrFromNest == true and RobotDataptr->GoingToNest == false)
    {
        RobotToStop = 2;
    }
    else{
         RobotToStop = 3;
    }
    if(RobotDataptr->Neighbors.size() == 0)
    {
        GetNeighbor_ThisRobot(RobotDataptr);
    }
    
    if(RobotDataptr->Neighbors.size() > 0)

    {
        for(NeighborSize = 0; NeighborSize < RobotDataptr->Neighbors.size(); NeighborSize++)

        
        {
            iterator1 = m_cFootbots.begin();
            // get the handle to particular robot
            std::advance(iterator1, RobotDataptr->Neighbors[NeighborSize]);

            
            CFootBotEntity& cFootBotNeighbor = *any_cast<CFootBotEntity*>(iterator1->second);
            BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbor.GetControllableEntity().GetController());
            
            // Get the robot data
            stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();
            
            // Get Intersection Data
            stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();
            
                // if the neighbor is not going in/ out of the nest
                if(stRobotDataNeighbor->GoingToOrFromNest == false)
                {
                    // get the points to check intersection
                    GetPointAtSafeDistance(stRobotDataNeighbor);
                    
                    // check if the robot is intersecting with other robots
                    IntersectionFlag = Find_Intersection(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint,stRobotDataNeighbor->StartWaypoint , stRobotDataNeighbor->IntersectionPt1,
                                                         stRobotIntersectionData, stIntersectionDataNeighbor, RobotDataptr, stRobotDataNeighbor, true);
                    
                    // if the robots are intersecting
                    if(IntersectionFlag == 1)
                    {

                        
                        RobotIntersectionData = GetIntersectionDataFromVector(stRobotIntersectionData,
                                                                              stRobotDataNeighbor->id_robot, COLLINEAR_INTERSECTION);

                        
                        NeighborRobotIntersectionData = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                      RobotDataptr->id_robot, COLLINEAR_INTERSECTION);

                        
                        IntersectionCollisionCheck(RobotDataptr, stRobotDataNeighbor, RobotIntersectionData, NeighborRobotIntersectionData,
                                                   INTERSECTION1, RobotToStop);
                        
                    }
                    /* neighbor robot is collinear to the current robots waypoint */
                    else if(IntersectionFlag == 2)
                    {
//                        /* Add stop time if the robot is in the outer loop of the robot coming out of nest */
//                        argos::Real Dist1 = CalculateDistance(stRobotDataNeighbor->TargetWaypoint, RobotDataptr->StartWaypoint);
//                        argos::Real Dist2 = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//
//                        if(Dist1 > Dist2)
//                        {
//                            /* Add stop time to the robot going towards nest */
//                            argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->TargetWaypoint).Angle();
//                            argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
//                            TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                            TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                            Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//                            TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
//
//                            stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
//                        }
                    }
                    /* neighbor robot is going in/ out of nest */
                    else
                    {
                        // check if the robot with second point is intersecting with other robots
                        IntersectionFlag = Find_Intersection(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint, stRobotDataNeighbor->IntersectionPt1 ,
                                                             stRobotDataNeighbor->IntersectionPt2, stRobotIntersectionData, stIntersectionDataNeighbor, RobotDataptr, stRobotDataNeighbor, true);
                        
                        // if the robots are intersecting
                        if(IntersectionFlag == 1)
                        {

                            
                            RobotIntersectionData = GetIntersectionDataFromVector(stRobotIntersectionData,
                                                                                  stRobotDataNeighbor->id_robot, COLLINEAR_INTERSECTION);

                            
                            NeighborRobotIntersectionData = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                          RobotDataptr->id_robot, COLLINEAR_INTERSECTION);

                            IntersectionCollisionCheck(RobotDataptr, stRobotDataNeighbor, RobotIntersectionData, NeighborRobotIntersectionData,
                                                       INTERSECTION2, RobotToStop);
                        }
                        
                        /* neighbor robot is collinear to the current robots waypoint */
                        else if(IntersectionFlag == 2)
                        {
                            
//                            /* Add stop time if the robot is in the outer loop of the robot coming out of nest */
//                            argos::Real Dist1 = CalculateDistance(stRobotDataNeighbor->TargetWaypoint, RobotDataptr->StartWaypoint);
//                            argos::Real Dist2 = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//
//                            if(Dist1 > Dist2)
//                            {
//
//                                /* Add stop time to the robot going towards nest */
//                                argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->TargetWaypoint).Angle();
//                                argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
//                                TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                                TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                                Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//                                TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
//
//                                stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
//                            }
                        }
                        
                    }
                    
                    
                    
                }
                /* if the neighbor is going in/ ut of nest*/
                else
                {
                    // if both the robots are going out of nest
                    if((RobotDataptr->GoingToNest == false and RobotDataptr->GoingToOrFromNest == true) and
                       (stRobotDataNeighbor->GoingToNest == false and stRobotDataNeighbor->GoingToOrFromNest == true))
                    {
                        
//                        IntersectionHandling_ForRobotsGoingOutOfNest_Collinear(RobotDataptr, stRobotDataNeighbor,
//                                                                    stRobotIntersectionData, stIntersectionDataNeighbor);
//
                    }
                    else
                    {
                        // check if the robot is intersecting with other robots
                        IntersectionFlag = Find_Intersection(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint, stRobotDataNeighbor->StartWaypoint ,
                                                             stRobotDataNeighbor->TargetWaypoint,
                                                             stRobotIntersectionData, stIntersectionDataNeighbor, RobotDataptr, stRobotDataNeighbor, true);
                        
                        if(IntersectionFlag == 1)
                        {

                            
                            RobotIntersectionData = GetIntersectionDataFromVector(stRobotIntersectionData,
                                                                                  stRobotDataNeighbor->id_robot, COLLINEAR_INTERSECTION);
                            
                            
                            NeighborRobotIntersectionData = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                          RobotDataptr->id_robot, COLLINEAR_INTERSECTION);

                            
                            IntersectionCollisionCheck(RobotDataptr, stRobotDataNeighbor, RobotIntersectionData, NeighborRobotIntersectionData,
                                                       INTERSECTION1,RobotToStop);
                        }
                        else if(IntersectionFlag == 2)
                        {
//                            /* Add stop time if the robot is in the outer loop of the robot coming out of nest */
//                            argos::Real Dist1 = CalculateDistance(stRobotDataNeighbor->TargetWaypoint, RobotDataptr->StartWaypoint);
//                            argos::Real Dist2 = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//
//
//                            if(stRobotDataNeighbor->GoingToNest == true and (Dist1 > Dist2))
//                            {
//                                /* Add stop time to the robot going towards nest */
//                                argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->TargetWaypoint).Angle();
//                                argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
//                                TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                                TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
//
//                                Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->TargetWaypoint);
//                                TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
//
//                                stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
//                            }
//                            else
//                            {
//                                /* Do nothing */
//                            }
                        }/* end of else if(IntersectionFlag == 2) */
                    }
                    
                }
      
        }// end of for
    }// end of if(BaseRobotDataptr->Neighbors.size() > 0)
    
    
}

/**********************************************************************************************************************************************************************/
/* Function to get the Intersection data from the intersection vcector */
/**********************************************************************************************************************************************************************/
BaseController::IntersectionData* DSA_loop_functions::GetIntersectionDataFromVector(std::vector<BaseController::IntersectionData> *ptr,
                                                                                    argos::UInt16 RobotId, argos::UInt8 IntersectionType)
{
    BaseController::IntersectionData* RobotIntersectionData = NULL;
    argos::UInt8 i;
    
    // get the intersection data for robot1
    for(i=0; i < ptr->size(); i++)
    {
        RobotIntersectionData = &(ptr->at(i));

        if(RobotIntersectionData->Robot_ID_Intersectingwith == RobotId and RobotIntersectionData->Intersection_Type == IntersectionType)
        {
            TestValue = 7;
            IntersectionStructIndex = i;
            
            break;
           
        }
        
        RobotIntersectionData = NULL;
    }

    return RobotIntersectionData;
}

/**********************************************************************************************************************************************************************/
/* Function to get the Intersection data from the intersection vcector */
/**********************************************************************************************************************************************************************/
void DSA_loop_functions::QuadrantGroupType(std::vector<argos::UInt8>* ptr, argos::UInt8 size)
{
    int i, temp;
//    argos::UInt8 Group;
    
    BaseController::RobotData *stRobotDataThis = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    
    
    // dereference the pased vector pointer to easily access the vector elements
    std::vector<argos::UInt8>& vecRef = *ptr;
    
    for(i=0; i < size; i++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, i);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        //if the point has coordinates(+x,+y) or (+x,-y)
        if((stRobotDataThis->StartWaypoint.GetX() >= 0 and stRobotDataThis->StartWaypoint.GetY() >= 0) or
           (stRobotDataThis->StartWaypoint.GetX() >= 0 and stRobotDataThis->StartWaypoint.GetY() <= 0))
        {
//            Group = 1;
            stRobotDataThis->QuadrantGroup = 1;
        }
        //if the point has coordinates(+x,-y) or (-x,-y)
        else if((stRobotDataThis->StartWaypoint.GetX() >= 0 and stRobotDataThis->StartWaypoint.GetY() <= 0) or
                (stRobotDataThis->StartWaypoint.GetX() <= 0 and stRobotDataThis->StartWaypoint.GetY() <= 0))
        {
//            Group = 2;
            stRobotDataThis->QuadrantGroup = 2;
        }
        //if the point has coordinates(-x,-y) or (-x,+y)
        else if((stRobotDataThis->StartWaypoint.GetX() <= 0 and stRobotDataThis->StartWaypoint.GetY() <= 0) or
                (stRobotDataThis->StartWaypoint.GetX() <= 0 and stRobotDataThis->StartWaypoint.GetY() >= 0))
        {
//            Group = 3;
            stRobotDataThis->QuadrantGroup = 3;
        }
        //if the point has coordinates(-x,+y) or (+x,+y)
        else if((stRobotDataThis->StartWaypoint.GetX() <= 0 and stRobotDataThis->StartWaypoint.GetY() >= 0) or
                (stRobotDataThis->StartWaypoint.GetX() >= 0 and stRobotDataThis->StartWaypoint.GetY() >= 0))
        {
//            Group = 4;
            stRobotDataThis->QuadrantGroup = 4;
        }
        else
        {
            /* Do Nothing */
        }
    }
//    return Group;
}

/*******************************************************************************************************************************/
void DSA_loop_functions::SortRobotForward(std::vector<argos::UInt8>* ptr, argos::UInt8 size, argos::UInt8 ToNestDirection)
{
    int i, j, k, temp;
    argos::Real dist1, dist2;
    argos::UInt8 StopTime = 20;
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNext = NULL;
    BaseController::RobotData *stRobotDataNow = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator3 = m_cFootbots.begin();
    
    
    
    
    // dereference the pased vector pointer to easily access the vector elements
    std::vector<argos::UInt8>& vecRef = *ptr;
    
    for(i=0; i < size; i++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, vecRef[i]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        dist1 = (NestPosition - stRobotDataThis->StartWaypoint).Length();
        
        
        for(j = i+1; j <= size-1; j++)
        {
            iterator2 = m_cFootbots.begin();
            // get the handle to particular robot
            std::advance(iterator2, vecRef[j]);
            CFootBotEntity& cFootBotNext = *any_cast<CFootBotEntity*>(iterator2->second);
            BaseController& cControllerNext = dynamic_cast<BaseController&>(cFootBotNext.GetControllableEntity().GetController());
            
            // Get the robot data
            stRobotDataNext = &cControllerNext.GetRobotData();
            
            dist2 = (NestPosition - stRobotDataNext->StartWaypoint).Length();
            
            // arrange the robot from nest in ascending order
            if(dist2 < dist1)
            {
                temp = vecRef[i];
                vecRef[i] = vecRef[j];
                vecRef[j] = temp;
            }
        }
    }
        
}



/**********************************************************************************************************************************************************************/
/* Function to get the Intersection data from the intersection vcector */
/**********************************************************************************************************************************************************************/
void DSA_loop_functions::SortforLeftMostRobot(std::vector<argos::UInt8>* ptr, argos::UInt8 size)
{
    int i, j, temp;
    UInt8 ret_Group;
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNext = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    
    
    // check in what quadrant does the robot lie
//    ret_Group = QuadrantGroupType(ptr, size);
    QuadrantGroupType(ptr, size);
    
    // dereference the pased vector pointer to easily access the vector elements
    std::vector<argos::UInt8>& vecRef = *ptr;
    
    for(i=0; i < size; i++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, vecRef[i]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        // check if the next robot in the vector is to the left of the current robot
        for(j = i+1; j <= size-1; j++)
        {
            iterator2 = m_cFootbots.begin();
            // get the handle to particular robot
            std::advance(iterator2, vecRef[j]);
            CFootBotEntity& cFootBotNext = *any_cast<CFootBotEntity*>(iterator2->second);
            BaseController& cControllerNext = dynamic_cast<BaseController&>(cFootBotNext.GetControllableEntity().GetController());
            
            // Get the robot data
            stRobotDataNext = &cControllerNext.GetRobotData();
            
            
//            if(ret_Group == 1)
            //coordinates(+x,+y) or (+x,-y)
            if(stRobotDataThis->QuadrantGroup == 1 and stRobotDataNext->QuadrantGroup == 1)
            {
                if((stRobotDataNext->StartWaypoint.GetY() > stRobotDataThis->StartWaypoint.GetY()) or
                   (stRobotDataNext->StartWaypoint.GetY() == stRobotDataThis->StartWaypoint.GetY() and
                    stRobotDataNext->StartWaypoint.GetX() < stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
//            else if(ret_Group == 2)
             //if the point has coordinates(+x,-y) or (-x,-y)
            else if(stRobotDataThis->QuadrantGroup == 2 and stRobotDataNext->QuadrantGroup == 2)
            {
                if((stRobotDataNext->StartWaypoint.GetY() > stRobotDataThis->StartWaypoint.GetY()) or
                   (stRobotDataNext->StartWaypoint.GetY() == stRobotDataThis->StartWaypoint.GetY() and
                    stRobotDataNext->StartWaypoint.GetX() > stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
                
            }
//            else if(ret_Group == 3)
            //if the point has coordinates(-x,-y) or (-x,+y)
            else if(stRobotDataThis->QuadrantGroup == 3 and stRobotDataNext->QuadrantGroup == 3)
            {
                if((stRobotDataNext->StartWaypoint.GetY() < stRobotDataThis->StartWaypoint.GetY()) or
                   (stRobotDataNext->StartWaypoint.GetY() == stRobotDataThis->StartWaypoint.GetY() and
                    stRobotDataNext->StartWaypoint.GetX() > stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
                
            }
            //if the point has coordinates(-x,+y) or (+x,+y)
//            else if(ret_Group == 4)
            else if(stRobotDataThis->QuadrantGroup == 4 and stRobotDataNext->QuadrantGroup == 4)
            {
                if((stRobotDataNext->StartWaypoint.GetY() < stRobotDataThis->StartWaypoint.GetY()) or
                   (stRobotDataNext->StartWaypoint.GetY() == stRobotDataThis->StartWaypoint.GetY() and
                    stRobotDataNext->StartWaypoint.GetX() < stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
            else if((stRobotDataThis->QuadrantGroup == 1 and stRobotDataNext->QuadrantGroup == 2) or
                    (stRobotDataThis->QuadrantGroup == 2 and stRobotDataNext->QuadrantGroup == 1))
            {
                if((stRobotDataNext->StartWaypoint.GetY() >= stRobotDataThis->StartWaypoint.GetY()) and
                   (stRobotDataNext->StartWaypoint.GetX() >= stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
            else if((stRobotDataThis->QuadrantGroup == 2 and stRobotDataNext->QuadrantGroup == 3) or
                    (stRobotDataThis->QuadrantGroup == 3 and stRobotDataNext->QuadrantGroup == 2))
            {
                if((stRobotDataNext->StartWaypoint.GetX() >= stRobotDataThis->StartWaypoint.GetX()) and
                   (stRobotDataNext->StartWaypoint.GetY() <= stRobotDataThis->StartWaypoint.GetY()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
            else if((stRobotDataThis->QuadrantGroup == 3 and stRobotDataNext->QuadrantGroup == 4) or
                    (stRobotDataThis->QuadrantGroup == 4 and stRobotDataNext->QuadrantGroup == 3))
            {
                if((stRobotDataNext->StartWaypoint.GetX() <= stRobotDataThis->StartWaypoint.GetX()) and
                   (stRobotDataNext->StartWaypoint.GetY() <= stRobotDataThis->StartWaypoint.GetY()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
            else if((stRobotDataThis->QuadrantGroup == 1 and stRobotDataNext->QuadrantGroup == 4) or
                    (stRobotDataThis->QuadrantGroup == 4 and stRobotDataNext->QuadrantGroup == 1))
            {
                if((stRobotDataNext->StartWaypoint.GetY() >= stRobotDataThis->StartWaypoint.GetY()) and
                   (stRobotDataNext->StartWaypoint.GetX() <= stRobotDataThis->StartWaypoint.GetX()))
                {
                    temp = vecRef[i];
                    vecRef[i] = vecRef[j];
                    vecRef[j] = temp;
                }
            }
            else
            {
                /* Do nothing */
            }

        }// end of for(j)
    }// end of for( i)
}

/***************************************************************************************************************************************************/
void DSA_loop_functions::CollinearPathPlanning(BaseController::RobotData *stRobotDataThis)
{
    argos::UInt8 NeighborsCount, robotneighborindex;
    
    BaseController::RobotData *stRobotDataNeighbor = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();

    // count the number of neighbors for the robot
    NeighborsCount = stRobotDataThis->Neighbors.size();
    
    // if the robot has neighbors
    if(NeighborsCount > 0)
    {
        /* if neighbors matrix exist, clear it before reinitializig the matrix */
        if(stRobotDataThis->NeighborsMatrix.size() > 0)
        {
            stRobotDataThis->NeighborsMatrix.clear();
        }
        // resize the matrix depending on the neighbor size
        // add 1 to get the row and column for robot itself
        InitializeMatrix(stRobotDataThis,(NeighborsCount + 1));
        
        // update the matrix by checking the collinearity and intersection of robot with its neighbors
        for(robotneighborindex = 0; robotneighborindex < stRobotDataThis->Neighbors.size(); robotneighborindex++)
        {
            iterator2 = m_cFootbots.begin();
            
            // get the handle to neighbor robot
            std::advance(iterator2, stRobotDataThis->Neighbors[robotneighborindex]);
            CFootBotEntity& cFootBotNeighbor = *any_cast<CFootBotEntity*>(iterator2->second);
            BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbor.GetControllableEntity().GetController());
            stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();
            stRobotDataNeighbor->StartWaypoint = cControllerNeighbor.GetPosition();
  
            
            stRobotDataThis->CollinearFlag = false;
            stRobotDataNeighbor->CollinearFlag = false;
            
            // check if the robots are collinear
            // ptr to the robot data of both the robots
            
            // if both the robots are going to nest/ going away from nest
            if(stRobotDataThis->GoingToOrFromNest == true and stRobotDataNeighbor->GoingToOrFromNest == true)
            {
            
                if(((stRobotDataThis->GoingToOrFromNest == true and stRobotDataThis->GoingToNest == false) and
                    (stRobotDataNeighbor->GoingToOrFromNest == true and stRobotDataNeighbor->GoingToNest == false))
                   and (stRobotDataThis->WaypointCounter == WayPointMaxCount and stRobotDataNeighbor->WaypointCounter == WayPointMaxCount))
                {
                    /* Don't check for collinearity */
                    InitializeMatrixElementAndTransformElement(stRobotDataThis,0,(robotneighborindex+1),CONSISTENT);
                }

                else
                {
                    if(stRobotDataNeighbor->WaypointCounter == WayPointMaxCount)
                    {
                        /* Do Nothing */
                    }
                    else
                    {
                        CheckCollinearity(stRobotDataThis, stRobotDataNeighbor);
                        
                        // update the matrix if the robots are collinear
                        if(stRobotDataThis->CollinearFlag == true)
                        {
                            // ptr for matrix to update, matrix indices of the robots, value to update
                            InitializeMatrixElementAndTransformElement(stRobotDataThis,0,(robotneighborindex+1),COLLINEAR);
                            
                            // Reset the collinearity flag
                            stRobotDataThis->CollinearFlag = false;
                            stRobotDataNeighbor->CollinearFlag = false;
                            
                        }
                    }
                }
            }
        }
    }
}

/************************************************************************************************************************************************************************************************/
argos::UInt8 DSA_loop_functions::FindRobotGoingToWayPt(std::vector<argos::UInt8>* ptr, argos::UInt8 size)
{
    argos::UInt8 index, id;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    
    BaseController::RobotData *stRobotDataThis = NULL;
    
    // dereference the pased vector pointer to easily access the vector elements
    std::vector<argos::UInt8>& vecRef = *ptr;
    
    id = 255;
    for(index = 0; index < size; index++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to neighbor robot indicated by column
        std::advance(iterator1, vecRef[index]);
        CFootBotEntity& cFootBotDataThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerBotThis = dynamic_cast<BaseController&>(cFootBotDataThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerBotThis.GetRobotData();
        
        stRobotDataThis->StartWaypoint = cControllerBotThis.GetPosition();
        
        if(stRobotDataThis->AddWaypoint == true and stRobotDataThis->WaypointCounter == 1)
        {
            id = stRobotDataThis->id_robot;
//            break;
        }
        
    }
    return id;

}


/******************************************************************************************************************************************/
void DSA_loop_functions::ClusterModeOperation()
{
    argos::UInt8 robotresourceindex;
    argos::CVector2 WayPoint;
    
    
    
    BaseController::RobotData *stRobotDataThis = NULL;
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    
    for(robotresourceindex = 0; robotresourceindex < RobotResource.size(); robotresourceindex++)
    {
        WayPoint.Set(0,0);
        iterator1 = m_cFootbots.begin();
//        stRobotDataThis = NULL;
        std::advance(iterator1, RobotResource[robotresourceindex]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        // if the robot is going away from nest
        if(stRobotDataThis->GoingToNest == false and stRobotDataThis->GoingToOrFromNest == true)
        {
//            WayPoint = CalculateWayPoint(stRobotDataThis, stRobotDataThis, 1);
            WayPoint = GetWayPointClusterMode(stRobotDataThis);
            
            if((stRobotDataThis->WaypointCounter < WayPointMaxCount) and
               (CalculateDistance(stRobotDataThis->StartWaypoint, NestPosition) < MaxArenaDistance))

            {

                AddWayPoint(stRobotDataThis);
                stRobotDataThis->AddedPoint = WayPoint;
                stRobotDataThis->AddWaypoint = true;
                stRobotDataThis->TargetWaypoint = stRobotDataThis->AddedPoint;
                stRobotDataThis->WaypointCounter++;
                cControllerThis.SetTarget(stRobotDataThis->TargetWaypoint);
                cControllerThis.SetHardStopMovement();

            }
        }
    }
}

/********************************************************************************************************************/
void DSA_loop_functions::CollectiveCollinearCheck()
{
    
    argos::UInt16 NeighborsCount = 0;
    argos::CVector2 TargetPoint;
    argos::UInt8 row_size, column_size, row_index, column_index, Matrix_Value, index1, index2, index3, index, indexrobo;
    argos::UInt8 robotresourceindex, robotneighborindex, robotnextneighborindex;
    argos::UInt8 row1, col1;
    argos::Real CurrentDistance;
    bool collinearity_flag, NoNeedToCalculateWaypt, Firt_Entry;

    
    argos::UInt8 LeftMostRobotGoingToNest;
    argos::UInt8 Neighbor_Count;
    
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNeighbor = NULL;
    BaseController::RobotData *stRobotDataNeighborNext = NULL;
    BaseController::RobotData *stRobotDataLeftMostTowardsNest = NULL;
    BaseController::RobotData *stRobotDataGoingAwayNest= NULL;
    BaseController::RobotData *RobotData_Previous = NULL;
    BaseController::RobotData *stRobotDataAwayFromNest = NULL;
    BaseController::RobotData *stRobotTowardsNest = NULL;
    BaseController::RobotData *stRobotNeighbor = NULL;


    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator3 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator4 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator5 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator6 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator7 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator8 = m_cFootbots.begin();


    std::vector<argos::UInt8> CollinearRobots_GoingTowardsNestCopy;

    argos::CVector2 WayPoint;
    argos::CVector2 PreviousWaypoint;

    argos::UInt8 Counter = 0;
    LeftMostRobotID = 203;

    argos::UInt16 StopTime = 0;
    argos::UInt16 STOP_TIME_MULTIPLE = 20;
    Firt_Entry = false;
    collinearity_flag = false;
    NoNeedToCalculateWaypt = false;
    RobotWithWaypt = false;
    
    for(robotresourceindex = 0; robotresourceindex < RobotResource.size(); robotresourceindex++)
    {
        WaypointType = 1;
        Counter = 0;
        column_size = 0;

        RobotWithWaypt = false;
        Firt_Entry = false;

        collinearity_flag = false;
        NoNeedToCalculateWaypt = false;
  

        if(CollinearRobots_GoingAwayFromNest.size() > 0)
        {
            CollinearRobots_GoingAwayFromNest.clear();
        }
        if(CollinearRobots_GoingTowardsNest.size() > 0)
        {
            CollinearRobots_GoingTowardsNest.clear();
        }

        iterator1 = m_cFootbots.begin();
        iterator2 = m_cFootbots.begin();
        iterator3 = m_cFootbots.begin();
        iterator4 = m_cFootbots.begin();
        iterator5 = m_cFootbots.begin();
        iterator6 = m_cFootbots.begin();
        iterator7 = m_cFootbots.begin();
        iterator8 = m_cFootbots.begin();

        // get the handle to particular robot
        std::advance(iterator1, RobotResource[robotresourceindex]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());

        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();

        // Get the robot data
        stRobotDataThis->StartWaypoint = cControllerThis.GetPosition();

        // get the collinearity matrix
        CollinearPathPlanning(stRobotDataThis);

        row_size = stRobotDataThis->NeighborsMatrix.size();

        // get number of columns
        if(row_size > 0)
        {
            column_size = stRobotDataThis->NeighborsMatrix[0].size();
        }

        // Get the robot data
        stRobotDataThis->StartWaypoint = cControllerThis.GetPosition();


        if(column_size > 0)
        {
            // check with every neighbor
            for(column_index=0; column_index< (column_size-1); column_index++)
            {
                /* get the data of the neighbor robot */

                iterator2 = m_cFootbots.begin();
                // get the handle to neighbor robot indicated by column
                std::advance(iterator2, stRobotDataThis->Neighbors[column_index]);
                CFootBotEntity& cFootBotNextNeighbor = *any_cast<CFootBotEntity*>(iterator2->second);
                BaseController& cControllerNextNeighbor = dynamic_cast<BaseController&>(cFootBotNextNeighbor.GetControllableEntity().GetController());

                // Get the robot data
                stRobotDataNeighborNext = &cControllerNextNeighbor.GetRobotData();

                stRobotDataNeighborNext->StartWaypoint = cControllerNextNeighbor.GetPosition();

                Matrix_Value = stRobotDataThis->NeighborsMatrix[0][column_index+1];

                if(Matrix_Value == COLLINEAR)
                {
                    /* if this robot is going away from nest */
                    if(stRobotDataNeighborNext->GoingToOrFromNest == true and stRobotDataNeighborNext->GoingToNest == false)
                    {
                        CollinearRobots_GoingAwayFromNest.push_back(stRobotDataNeighborNext->id_robot);
                        // add the id of collinear robot to the collinear list of this robot
                        stRobotDataThis->CollinearRobotGoingAwayNestList.push_back(stRobotDataNeighborNext->id_robot);
                    }
                    // robot going to nest
                    else if(stRobotDataNeighborNext->GoingToNest == true)
                    {
                        CollinearRobots_GoingTowardsNest.push_back(stRobotDataNeighborNext->id_robot);
                        // add the id of collinear robot to the collinear list of this robot
                        stRobotDataThis->CollinearRobotGoingToNestList.push_back(stRobotDataNeighborNext->id_robot);

                    }
                    if(stRobotDataThis->GoingToOrFromNest == true and stRobotDataThis->GoingToNest == false)
                    {
                        stRobotDataNeighborNext->CollinearRobotGoingAwayNestList.push_back(stRobotDataThis->id_robot);
                    }
                    else if(stRobotDataThis->GoingToNest == true)
                    {
                        // add the id of this robot to the colinear list of other collinear robot
                        stRobotDataNeighborNext->CollinearRobotGoingToNestList.push_back(stRobotDataThis->id_robot);
                    }

                    if(Firt_Entry == false)
                    {
                        if(stRobotDataThis->GoingToOrFromNest == true and stRobotDataThis->GoingToNest == false)
                        {
                            CollinearRobots_GoingAwayFromNest.push_back(stRobotDataThis->id_robot);
                        }
                        // robot going to nest
                        else if(stRobotDataThis->GoingToNest == true)
                        {
                            CollinearRobots_GoingTowardsNest.push_back(stRobotDataThis->id_robot);
                        }
                        Firt_Entry = true;
                    }

                    InitializeMatrixElementAndTransformElement(stRobotDataThis,0, (column_index+1), CONSISTENT);
                }// end of if(Matrix_Value == COLLINEAR)


            } //end of for(column_index=0; column_index< column_size; column_index++)
        } // end of if(column_size > 0)

        
        
        stRobotDataThis->rows = CollinearRobots_GoingAwayFromNest.size();
        stRobotDataThis->cols = CollinearRobots_GoingTowardsNest.size();

        if(CollinearRobots_GoingAwayFromNest.size() > 0)
        {
            SortforLeftMostRobot(&CollinearRobots_GoingAwayFromNest, CollinearRobots_GoingAwayFromNest.size());
        }

        if(CollinearRobots_GoingTowardsNest.size() > 0)
        {
            SortforLeftMostRobot(&CollinearRobots_GoingTowardsNest, CollinearRobots_GoingTowardsNest.size());
        }
        
        // this if is not used as working on just one cluster
        if(CLUSTERCONFIGONLY == false)
        {
            if(CollinearRobots_GoingAwayFromNest.size() > 0)
            {
                Counter = 0;

                if(CollinearRobots_GoingTowardsNest.size() > 0)
                {
                    iterator4 = m_cFootbots.begin();
                    LeftMostRobotGoingToNest = CollinearRobots_GoingTowardsNest[0];
                    // get the handle to neighbor robot indicated by column
                    std::advance(iterator4, LeftMostRobotGoingToNest);
                    CFootBotEntity& cFootBotLeftMostTowardsNest = *any_cast<CFootBotEntity*>(iterator4->second);
                    BaseController& cControllerLeftMostTowardsNest = dynamic_cast<BaseController&>(cFootBotLeftMostTowardsNest.GetControllableEntity().GetController());

                    // Get the robot data
                    stRobotDataLeftMostTowardsNest = &cControllerLeftMostTowardsNest.GetRobotData();

                    stRobotDataLeftMostTowardsNest->StartWaypoint = cControllerLeftMostTowardsNest.GetPosition();
                    WaypointType = 1;
                }
                else
                {
                    iterator4 = m_cFootbots.begin();
                    LeftMostRobotGoingToNest = CollinearRobots_GoingAwayFromNest[0];
                    // get the handle to neighbor robot indicated by column
                    std::advance(iterator4, LeftMostRobotGoingToNest);
                    CFootBotEntity& cFootBotLeftMostTowardsNest = *any_cast<CFootBotEntity*>(iterator4->second);
                    BaseController& cControllerLeftMostTowardsNest = dynamic_cast<BaseController&>(cFootBotLeftMostTowardsNest.GetControllableEntity().GetController());

                    // Get the robot data
                    stRobotDataLeftMostTowardsNest = &cControllerLeftMostTowardsNest.GetRobotData();

                    stRobotDataLeftMostTowardsNest->StartWaypoint = cControllerLeftMostTowardsNest.GetPosition();

                    WaypointType = 2;

                }

                for(index2 = 0; index2 < CollinearRobots_GoingAwayFromNest.size(); index2++)
                {
                    iterator3 = m_cFootbots.begin();

                    std::advance(iterator3, CollinearRobots_GoingAwayFromNest[index2]);
                    CFootBotEntity& cFootBotGoingAwayNest = *any_cast<CFootBotEntity*>(iterator3->second);
                    BaseController& cControllerGoingAwayNest = dynamic_cast<BaseController&>(cFootBotGoingAwayNest.GetControllableEntity().GetController());

                    // Get the robot data
                    stRobotDataGoingAwayNest = &cControllerGoingAwayNest.GetRobotData();
                    stRobotDataGoingAwayNest->CollectiveCollinearChecked = true;
                    stRobotDataGoingAwayNest->StartWaypoint = cControllerGoingAwayNest.GetPosition();


                    stRobotDataGoingAwayNest->distance = (stRobotDataGoingAwayNest->TargetPosition - NestPosition).Length() - NestRadius;


                    CurrentDistance = stRobotDataGoingAwayNest->distance - (CalculateDistance(stRobotDataGoingAwayNest->StartWaypoint, stRobotDataGoingAwayNest->TargetPosition));

                    // if the robot is not at the nest
                    if((stRobotDataGoingAwayNest->StartWaypoint - NestPosition).Length() > (NestRadius + 0.05))
                    {

                        if(CurrentDistance >= (0.75 * stRobotDataGoingAwayNest->distance))
                        {
                            WaypointType = 3;
                        }
                    }


                    WayPoint = CalculateWayPoint(stRobotDataGoingAwayNest, stRobotDataLeftMostTowardsNest, WaypointType);


                    Neighbor_Count = stRobotDataGoingAwayNest->Neighbors.size();

                    // check if the robot is at nest
                    if((stRobotDataGoingAwayNest->StartWaypoint - NestPosition).Length() < (NestRadius + 0.05))
                    {
                        // check if there is any robot that is going to the waypoint near any neighbors
                        if(Neighbor_Count > 0)
                        {
                            for(index3 = 0; index3 < Neighbor_Count; index3++ )
                            {
                                iterator5 = m_cFootbots.begin();

                                std::advance(iterator5, stRobotDataGoingAwayNest->Neighbors[index3]);
                                CFootBotEntity& cFootBotNeighbor = *any_cast<CFootBotEntity*>(iterator5->second);
                                BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbor.GetControllableEntity().GetController());

                                // Get the robot data
                                stRobotNeighbor = &cControllerNeighbor.GetRobotData();

                                stRobotNeighbor->StartWaypoint = cControllerNeighbor.GetPosition();


                                if((stRobotNeighbor->GoingToNest == false and stRobotNeighbor->GoingToOrFromNest == true) and
                                   (stRobotNeighbor->WaypointCounter == 1))
                                {

                                    if((stRobotNeighbor->StartWaypoint - WayPoint).Length() < 0.18)
                                    {
                                        Counter++;
                                    } //  end of if((stRobotNeighbor->StartWaypoint - WayPoint).Length() < 0.18)
                                } // if((stRobotNeighbor->GoingToNest == false and stRobotNeighbor->GoingToOrFromNest == true) and

                            }// end of for(index3 = 0; index3 < Neighbor_Count; index3++ )
                        }// end of if(Neighbor_Count > 0)
                    } // end of if((stRobotDataGoingAwayNest->StartWaypoint - NestPosition).Length() < (NestRadius + 0.05))
                    else
                    {
                        StopTime = 0;
                    }



                    if((stRobotDataGoingAwayNest->WaypointCounter < WayPointMaxCount) and
                       (CalculateDistance(stRobotDataGoingAwayNest->StartWaypoint, NestPosition) < 8) and
                       NoNeedToCalculateWaypt == false and CurrentDistance < 0.36)


                    {

                        AddWayPoint(stRobotDataGoingAwayNest);
                        stRobotDataGoingAwayNest->AddedPoint = WayPoint;
                        stRobotDataGoingAwayNest->AddWaypoint = true;
                        stRobotDataGoingAwayNest->TargetWaypoint = stRobotDataGoingAwayNest->AddedPoint;
                        stRobotDataGoingAwayNest->WaypointCounter += 1;
                        cControllerGoingAwayNest.SetTarget(stRobotDataGoingAwayNest->TargetWaypoint);
                        cControllerGoingAwayNest.SetHardStopMovement();
                        //                    stRobotDataGoingAwayNest->StopTurningTime = StopTime;

                    }
                }// end of for(index2 = 0; index2 < CollinearRobots_GoingAwayFromNest.size(); index2++)

            }// end of if(CollinearRobots_GoingAwayFromNest.size() > 0)

        
            }// end of if(CLUSTERCONFIGONLY == false)
            else
            {
                ClusterModeOperation();
            }
        
        }// end of for(robotresourceindex = 0; robotresourceindex < RobotResource.size(); robotresourceindex++)
        
            
        
    
    
}

/*****************************************************************************************************************************************************************/
void DSA_loop_functions::GetRectangleCoordinates(argos::CVector2 StartPoint, argos::CVector2 EndPoint)
{
    argos::UInt8 Direction, Axis;
    
    argos::Real dist, length, width;
    
    /* slope = (y2-y1)/(x2-x1)
     * if slope is 0 i.e. y2-y1 = 0 -> line is horizontal i.e. parallel to x-axis
     * if slope is undefined i.e. x2-x1 = 0 -> line is vertical, parallel to y- axis */
    
    // the line is parallel to y -axis
    if(StartPoint.GetX() - EndPoint.GetX() == 0)
    {
        Axis = 1;
        // the y coordinate is increasing: : add the radius
        if(StartPoint.GetY() < EndPoint.GetY())
        {
            Direction = 1;
        }
        // the y-coordinate is decreasing : subtract the radius
        else{
            Direction = 0;
        }
    }
    // line is parallel to x- axis
    else if(StartPoint.GetY() - EndPoint.GetY() == 0)
    {
        Axis = 0;
        // the x coordinate is increasing: : add the radius
        if(StartPoint.GetX() < EndPoint.GetX())
        {
            Direction = 1;
        }
        // the x-coordinate is decreasing : subtract the radius
        else{
            Direction = 0;
        }
    }
    
    dist = CalculateDistance(StartPoint, EndPoint);
    length = dist + FOOTBOT_RADIUS;
    width = 2 * FOOTBOT_RADIUS;
    
    // calculate the 4 coordinates of rectangle
    
}


/****************************************************************************************************************/
/* Function to avoid robot robot collision */
/****************************************************************************************************************/
void DSA_loop_functions::Avoid_Collision()
{

    argos::UInt16 NeighborsCount = 0;
    argos::CVector2 TargetPoint;
    argos::UInt8 row_size, column_size, row_index, column_index, Matrix_Value;
    argos::UInt8 robotresourceindex, robotneighborindex, robotnextneighborindex;
    
    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNeighbor = NULL;
    BaseController::RobotData *stRobotDataNeighborNext = NULL;
    
  
    std::vector<BaseController::IntersectionData>* stIntersectionDataThis = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighborNext = NULL;
    
    
    BaseController::IntersectionData* InersectionDataRobot1 = NULL;
    BaseController::IntersectionData* InersectionDataRobot2 = NULL;
    
    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    CSpace::TMapPerType::iterator iterator3 = m_cFootbots.begin();
    
//    IntersectionLoopValue = 5;
    
    // update neighbor matrix for every robot that has collected resource
    for(robotresourceindex = 0; robotresourceindex < RobotResource.size(); robotresourceindex++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, RobotResource[robotresourceindex]);
        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
        
        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        // Get Intersection Data
        stIntersectionDataThis = cControllerThis.GetIntersectionData();
        
        row_size = stRobotDataThis->NeighborsMatrix.size();
        
        // get number of columns
        if(row_size > 0)
        {
            column_size = stRobotDataThis->NeighborsMatrix[0].size();
        }


        // implement collision avoidance by checking each element of the matrix
        for(row_index=0; row_index< row_size; row_index++)
        {
            for(column_index=0; column_index< column_size; column_index++)
            {
                // get the value of the matrix avoiding redundancy (eg: don't check for 1,0 as 0,1 is already checked)
                if((row_index != column_index) and (column_index > row_index))
                {
                    Matrix_Value = stRobotDataThis->NeighborsMatrix[row_index][column_index];
                    IntersectionLoopValue = Matrix_Value;
                    
                    if(Matrix_Value == CONSISTENT)
                    {
                        /* Do Nothing */
                      
                    }
                    else
                    {

                        // robot index will be the neighbor index if the robot is not itself
                        if(row_index != 0)
                        {
                            robotneighborindex = stRobotDataThis->Neighbors[row_index - 1];

                        }
                        else
                        {
                            robotneighborindex = stRobotDataThis->id_robot;
                        }
                        robotnextneighborindex = stRobotDataThis->Neighbors[column_index - 1];

                        iterator2 = m_cFootbots.begin();
                        // get the handle to neighbor robot indicated by row
                        std::advance(iterator2, robotneighborindex);
                        CFootBotEntity& cFootBotNeighbhor = *any_cast<CFootBotEntity*>(iterator2->second);
                        BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbhor.GetControllableEntity().GetController());

                        // Get the robot data
                        stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();

                        stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();
                        
                        stRobotDataNeighbor->StartWaypoint = cControllerNeighbor.GetPosition();

                        iterator3 = m_cFootbots.begin();
                        // get the handle to neighbor robot indicated by column
                        std::advance(iterator3, robotnextneighborindex);
                        CFootBotEntity& cFootBotNextNeighbor = *any_cast<CFootBotEntity*>(iterator3->second);
                        BaseController& cControllerNextNeighbor = dynamic_cast<BaseController&>(cFootBotNextNeighbor.GetControllableEntity().GetController());

                        // Get the robot data
                        stRobotDataNeighborNext = &cControllerNextNeighbor.GetRobotData();

                        stIntersectionDataNeighborNext = cControllerNextNeighbor.GetIntersectionData();
                        
                        stRobotDataNeighborNext->StartWaypoint = cControllerNextNeighbor.GetPosition();
                        
                        InersectionDataRobot1 = NULL;
                        InersectionDataRobot2 = NULL;
                        
                        // implement collinear collision avoidance
                       if(Matrix_Value == COLLINEAR)
                       {
                            /* Do Nothing */

                       }
                        // implement intersection collision avoidance
                       else if(Matrix_Value == INTERSECTION1 or Matrix_Value == INTERSECTION2)
                       {

                           if(stRobotDataNeighbor->GoingToOrFromNest == false and stRobotDataNeighborNext->GoingToOrFromNest == true)
                           {

                              InersectionDataRobot1 = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                    stRobotDataNeighborNext->id_robot, Matrix_Value);


                              InersectionDataRobot2 = GetIntersectionDataFromVector(stIntersectionDataNeighborNext,
                                                                                     stRobotDataNeighbor->id_robot, Matrix_Value);


                                                              IntersectionCollisionCheck(stRobotDataNeighbor, stRobotDataNeighborNext, InersectionDataRobot1, InersectionDataRobot2,
                                                                                         Matrix_Value, 3);



                           }
                           else if(stRobotDataNeighborNext->GoingToOrFromNest == false and stRobotDataNeighbor->GoingToOrFromNest == true)
                           {

                               InersectionDataRobot1 = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                     stRobotDataNeighborNext->id_robot, Matrix_Value);


                               InersectionDataRobot2 = GetIntersectionDataFromVector(stIntersectionDataNeighborNext,
                                                                                     stRobotDataNeighbor->id_robot, Matrix_Value);

                               IntersectionCollisionCheck(stRobotDataNeighbor, stRobotDataNeighborNext, InersectionDataRobot1, InersectionDataRobot2,
                                                          Matrix_Value, 3);


                           }
                           else
                           {

                               InersectionDataRobot1 = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                     stRobotDataNeighborNext->id_robot, Matrix_Value);

                               InersectionDataRobot2 = GetIntersectionDataFromVector(stIntersectionDataNeighborNext,
                                                                                     stRobotDataNeighbor->id_robot, Matrix_Value);

                               IntersectionCollisionCheck(stRobotDataNeighbor, stRobotDataNeighborNext, InersectionDataRobot1, InersectionDataRobot2,
                                                          Matrix_Value, 3);


                           }
                           
                        
                          
                       }
                    } // end of else for if(Matrix_Value == CONSISTENT)
                }// end of if(row_index != 0 and column_index > row_index)
            }// end of for -> column
        }// end of for -> rows
    } // end of for -> loop for robots that have collected resources
        

    
}

/*******************************************************************************************************************************************/
bool DSA_loop_functions::Check_CollinearVectors(argos::CVector2 Vec1, argos::CVector2 Vec2, argos::CVector2 Vec3, argos::CVector2 Vec4)
{
    bool Collinear_Flag;
    argos::Real vec1length, vec2length,shortest_dist, CosTheta, dot_product;
    argos::Real epsilon = 0.001;
    argos::Real CosTheta_UpperRange = 1.0;
    argos::Real CosTheta_LowerRange = 0.93;
    argos::CVector2 vector1, vector2;
    

    
    Collinear_Flag = false;
    
    vector1.Set((Vec2.GetX() - Vec1.GetX()), (Vec2.GetY() - Vec1.GetY()));
    vector2.Set((Vec4.GetX() - Vec3.GetX()), (Vec4.GetY() - Vec3.GetY()));
    
    vec1length = vector1.Length();
    vec2length = vector2.Length();
    
   
    dot_product = vector1.DotProduct(vector2);
    
    
    CosTheta = (dot_product/(vec1length * vec2length));
    
    // get the shortest distance between two vectors

    shortest_dist = ShortestDistTwoSegments(Vec1,Vec2 , Vec3, Vec4);
    
   
    //angle ->180 to 152.87 and 0 to 27.12
    if(((fabs(CosTheta) - CosTheta_UpperRange) <= epsilon or fabs(CosTheta) < CosTheta_UpperRange ) and
       ((fabs(CosTheta) - CosTheta_LowerRange) <= epsilon or fabs(CosTheta) > CosTheta_LowerRange))
    {

            if((shortest_dist <= CollinearDistanceGap))
            {
                Collinear_Flag = true;
            }

    }
    
  
    return Collinear_Flag;
}

/***********************************************************************************************************************************************************************************/
/* Function to avoid intersection of robots */
/***********************************************************************************************************************************************************************************/
void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                    BaseController::IntersectionData* Robot1IntersectionData, BaseController::IntersectionData* Robot2IntersectionData,
                                                    argos::UInt8 IntersectionValue, argos::UInt8 index)
{
    
    argos::UInt16 TimeToIntersection1, TimeToIntersection2, TicksToWaitRobot1, TicksToWaitRobot2, TicksToWaitforSafedistance,
                  TimeToTurn1, TimeToTurn2, TimeFromStartPoint1, TimeFromStartPoint2, TimeToIntersectionRobot1, TimeToIntersectionRobot2,
                  TimeDiff, TimeToIntersection, i, j;
    argos::Real DistanceToIntersection1, DistanceToIntersection2, IntersectionDistance, AdjustedVelocity, Difference_Dist;
    
    argos::UInt16 SafeTurningTime;
    
    argos::Real SafeTurningAngle = 70.0f;
    
    argos::UInt16 Bufferime = SimulatorTicksperSec + 20;

    argos::UInt16 CollisionDetectionTime;
    SafeTurningTime = GetTimeToTurn(SafeTurningAngle, ptr1->fBaseAngularWheelSpeed);
    
    
    if(Robot1IntersectionData != NULL and Robot2IntersectionData != NULL)
    {
        if(IntersectionValue == INTERSECTION1)
        {
            // caluclate for Robot 1

            // calculate the time required by robot 1 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle1 = (Robot1IntersectionData->IntersectionPoint - Robot1IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle1 = (ptr1->Orientation - Headingangle1).SignedNormalize();
            argos::Real TurningAngle1_Deg = ToDegrees(TurningAngle1).GetValue();

            if(TurningAngle1_Deg < 0.0)
            {
                TurningAngle1_Deg = -TurningAngle1_Deg;
            }
            else
            {
                TurningAngle1_Deg = TurningAngle1_Deg;
            }

            TimeToTurn1 = GetTimeToTurn(TurningAngle1_Deg, ptr1->fBaseAngularWheelSpeed) + SafeTurningTime;

            DistanceToIntersection1 = CalculateDistance(Robot1IntersectionData->StartPoint, Robot1IntersectionData->IntersectionPoint);
            TimeFromStartPoint1 = GetTicksToWait(DistanceToIntersection1, ptr1->fLinearWheelSpeed);
            TimeToIntersectionRobot1 = TimeToTurn1 + TimeFromStartPoint1 + ptr1->StopTurningTime + SafeTurningTime;

            // caluclate for Robot 2
            argos::CRadians Headingangle2 = (Robot2IntersectionData->IntersectionPoint - Robot2IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle2 = (ptr2->Orientation - Headingangle2).SignedNormalize();

            argos::Real TurningAngle2_Deg = ToDegrees(TurningAngle2).GetValue();

            if(TurningAngle2_Deg < 0.0)
            {
                TurningAngle2_Deg = -TurningAngle2_Deg;
            }
            else
            {
                TurningAngle2_Deg = TurningAngle2_Deg;
            }


            TimeToTurn2 = GetTimeToTurn(TurningAngle2_Deg, ptr2->fBaseAngularWheelSpeed) + SafeTurningTime;

            DistanceToIntersection2 = CalculateDistance(Robot2IntersectionData->StartPoint,Robot2IntersectionData->IntersectionPoint);

            TimeFromStartPoint2 = GetTicksToWait(DistanceToIntersection2, ptr2->fLinearWheelSpeed);

            TimeToIntersectionRobot2 = TimeToTurn2 + TimeFromStartPoint2 + ptr2->StopTurningTime + SafeTurningTime;

            TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + Bufferime);
        }

        else
        {
            // calculate the time required by robot 1 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle1 = (Robot1IntersectionData->IntersectionPoint - Robot1IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle1 = (ptr1->Orientation - Headingangle1).SignedNormalize();

            argos::Real TurningAngle1_Deg = ToDegrees(TurningAngle1).GetValue();

            if(TurningAngle1_Deg < 0.0)
            {
                TurningAngle1_Deg = -TurningAngle1_Deg;
            }
            else
            {
                TurningAngle1_Deg = TurningAngle1_Deg;
            }

            TimeToTurn1 = GetTimeToTurn(TurningAngle1_Deg, ptr1->fBaseAngularWheelSpeed) + SafeTurningTime;

            DistanceToIntersection1 = CalculateDistance(Robot1IntersectionData->StartPoint, Robot1IntersectionData->IntersectionPoint) +
                                      CalculateDistance(ptr1->StartWaypoint, Robot1IntersectionData->StartPoint);

            TimeFromStartPoint1 = GetTicksToWait(DistanceToIntersection1, ptr1->fLinearWheelSpeed);
            TimeToIntersectionRobot1 = TimeFromStartPoint1 + TimeToTurn1 + ptr1->StopTurningTime;


            // calculate the time required by robot 2 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle2 = (Robot2IntersectionData->IntersectionPoint - Robot2IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle2 = (ptr2->Orientation - Headingangle2).SignedNormalize();

            argos::Real TurningAngle2_Deg = ToDegrees(TurningAngle2).GetValue();

            if(TurningAngle2_Deg < 0.0)
            {
                TurningAngle2_Deg = -TurningAngle2_Deg;
            }
            else
            {
                TurningAngle2_Deg = TurningAngle2_Deg;
            }

            TimeToTurn2 = GetTimeToTurn(TurningAngle2_Deg, ptr2->fBaseAngularWheelSpeed) + SafeTurningTime;

            DistanceToIntersection2 = CalculateDistance(Robot2IntersectionData->StartPoint, Robot2IntersectionData->IntersectionPoint) +
                                      CalculateDistance(ptr2->StartWaypoint, Robot2IntersectionData->StartPoint);
            TimeFromStartPoint2 = GetTicksToWait(DistanceToIntersection2, ptr2->fLinearWheelSpeed);

            TimeToIntersectionRobot2 = TimeFromStartPoint2 + TimeToTurn2 + ptr2->StopTurningTime;

            TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + (GetTimeToTurn(180, ptr1->fBaseAngularWheelSpeed)) + SafeTurningTime);
        }

        Time_1Int = TimeToIntersectionRobot1;
        Time_2Int = TimeToIntersectionRobot2;
        Timesafe = TicksToWaitforSafedistance;

        TimeDiff = abs(TimeToIntersectionRobot1 - TimeToIntersectionRobot2);
        Difference_Dist = DistanceToIntersection2 - DistanceToIntersection1;

        // collision detection compensation
        if(Difference_Dist < 0)
        {
            Difference_Dist = -1 * Difference_Dist;
        }
        else
        {
            Difference_Dist = 1 * Difference_Dist;
        }
        if(Difference_Dist <= 0.18)
        {
           CollisionDetectionTime = 2 * (GetTimeToTurn(37.5, ptr1->fBaseAngularWheelSpeed) + SimulatorTicksperSec);
        }
        else
        {
            CollisionDetectionTime  = 0;
        }

        if(TimeDiff <= TicksToWaitforSafedistance)
        {
            // add stop time to robot1
            if(index == 1)
            {
                IntersectionDistance = DistanceToIntersection2;
                TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot2;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                if(AdjustedVelocity < ptr1->fLinearWheelSpeed)
                {
                    AdjustedVelocity = ptr1->fLinearWheelSpeed;;
                    ptr1->StopTurningTime += ((TimeToIntersection + Bufferime + CollisionDetectionTime));

                }

                ptr1->fLinearWheelSpeed = AdjustedVelocity;
            }
            // add stop time to robot2
            else if(index == 2)
            {
                IntersectionDistance = DistanceToIntersection1;
                TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot1;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                if(AdjustedVelocity < ptr2->fLinearWheelSpeed)
                {
                    AdjustedVelocity = ptr2->fLinearWheelSpeed;;
                    ptr2->StopTurningTime += ((TimeToIntersection + Bufferime + CollisionDetectionTime));

                }

                ptr2->fLinearWheelSpeed = AdjustedVelocity;
            }
            // add stop time to robot that will take time to reach the intersction point
            else
            {
                // time required for robot1 is less than that for robot2, add stop time to robot2
                if(TimeToIntersectionRobot1 < TimeToIntersectionRobot2)
                {
                    IntersectionDistance = DistanceToIntersection1;
                    TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot1;
                    AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                    if(AdjustedVelocity < ptr2->fLinearWheelSpeed)
                    {
                        AdjustedVelocity = ptr2->fLinearWheelSpeed;
                        ptr2->StopTurningTime += ((TimeToIntersection + Bufferime + CollisionDetectionTime));

                    }

                    ptr2->fLinearWheelSpeed = AdjustedVelocity;
                }
                // time required for robot2 is less than that for robot1, add stop time to robot1
                else
                {
                    IntersectionDistance = DistanceToIntersection2;
                    TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot2;
                    AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                    if(AdjustedVelocity < ptr1->fLinearWheelSpeed)
                    {
                        AdjustedVelocity = ptr1->fLinearWheelSpeed;;
                        ptr1->StopTurningTime += ((TimeToIntersection + Bufferime + CollisionDetectionTime));

                    }

                    ptr1->fLinearWheelSpeed = AdjustedVelocity;
                } // end of else

            }// end of else: add stop time to either of robot by evaluating

        }// end of if(TimeDiff < TimeToWaitForSafeDistance)
        
    }// if(Robot1IntersectionData != NULL and Robot2IntersectionData != NULL)
    
    
}

/****************************************************************************************************************/
/* Function to check intersection and collonearity of robots and handle it */
/****************************************************************************************************************/
void DSA_loop_functions::CheckCollisionAndConsistency()
{
    // check collision for intersection as well as collinearity
    CheckCollisionWithNeighbors(false);
    
    // do the collision handling
    Avoid_Collision();
    
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

/*************************************************************************************************************************/
/* Function to find the shortest distance between two segments */
/************************************************************************************************************************/
argos:: Real DSA_loop_functions::ShortestDistTwoVectors(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
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
    distance[0] = DistancePointSegment(ptr1->StartWaypoint, ptr1->TargetWaypoint, ptr2->StartWaypoint);
    //    distance[0] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, ptr2.StartWaypoint);
    
    // dist between robot 1 start, goal and robot2 goal
    distance[1] = DistancePointSegment(ptr1->StartWaypoint, ptr1->TargetWaypoint, ptr2->TargetWaypoint);
    //    distance[1] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, TargetPoint2);
    
    // dist between robot 2 start, goal and robot1 start
    distance[2] = DistancePointSegment(ptr2->StartWaypoint, ptr2->TargetWaypoint, ptr1->StartWaypoint);
    //    distance[2] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, ptr1.StartWaypoint);
    
    // dist between robot 2 start, goal and robot2 goal
    distance[3] = DistancePointSegment(ptr2->StartWaypoint, ptr2->TargetWaypoint, ptr1->TargetWaypoint);
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
void DSA_loop_functions::CheckCollinearity(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
//    argos::CVector3 v1, v2, cross_product, AddedWaypoint, ZeroVector;
//    argos::Real v1length, v2length, dot_product, DegreeAngle, shortest_dist, CosTheta;
    argos::Real vec1length, vec2length,shortest_dist, CosTheta, dot_product;
    argos::UInt16 SafeTurningTime, Time1, Time2, TimeToTurn1, TimeToTurn2, TotalTime1, TotalTime2, TicksToWaitforSafedistance;
    argos::Real DistanceToTarget1, DistanceToTarget2;
    argos::Real epsilon = 0.001;
    argos::Real CosTheta_UpperRange = 1.0;
    argos::Real CosTheta_LowerRange = 0.93;
    argos::UInt16 Bufferime = 5;
    argos::Real SafeTurningAngle = 40;
//    argos::Real CosTheta_LowerRange = 0.95;
    
    argos::CVector2 vec1, vec2;
    
    argos::UInt16 timetowait;
    
    ptr1->CosTheta = 0;
    ptr2->CosTheta = 0;
    
    ptr1->HeadingAngle = argos::CDegrees(0.0f);
    ptr2->HeadingAngle = argos::CDegrees(0.0f);
//    argos::UInt16 CollinearityStopTime = 10;
    
    
    vec1.Set((ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX()), (ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()));
    vec2.Set((ptr2->TargetWaypoint.GetX() - ptr2->StartWaypoint.GetX()), (ptr2->TargetWaypoint.GetY() - ptr2->StartWaypoint.GetY()));
   
    vec1length = vec1.Length();
    vec2length = vec2.Length();
    
    
    ptr1->HeadingAngle = ToDegrees(vec1.Angle());
    ptr2->HeadingAngle = ToDegrees(vec2.Angle());
//    ptr1.vect1 = vec1;
//    ptr2.vect2 = vec2;
    
    dot_product = vec1.DotProduct(vec2);
    
    
    
    CosTheta = (dot_product/(vec1length * vec2length));
    
    ptr1->CosTheta = CosTheta;
    ptr2->CosTheta = CosTheta;
    
    // get the shortest distance between two vectors
    shortest_dist = ShortestDistTwoVectors(ptr1, ptr2);
    

    // angle = 0 to 21 deg celsius or 180 to 158
//    if(abs(CosTheta) <= 1 and abs(CosTheta) >= 0.93)
    
    
    //angle ->180 to 152.87 and 0 to 27.12
    if(((fabs(CosTheta) - CosTheta_UpperRange) <= epsilon or fabs(CosTheta) < CosTheta_UpperRange ) and
       ((fabs(CosTheta) - CosTheta_LowerRange) <= epsilon or fabs(CosTheta) > CosTheta_LowerRange))
    {
        if(ptr1->GoingToOrFromNest == true and ptr2->GoingToOrFromNest == true)
        {
            if((shortest_dist <= CollinearDistanceGap))
            {
                ptr1->CollinearFlag = true;
                ptr2->CollinearFlag = true;
                
            }
        }
    }
    
    // if both are collinear, check the timing to reach the target point
    if(ptr1->CollinearFlag == true and ptr2->CollinearFlag == true)
    {
        // check if both the robots are going away from nest
        if((ptr1->GoingToNest == false and ptr1->GoingToOrFromNest == true) and
           (ptr2->GoingToNest == false and ptr2->GoingToOrFromNest == true))
        {
            SafeTurningTime = GetTimeToTurn(SafeTurningAngle, ptr1->fBaseAngularWheelSpeed);
            
            argos::CRadians Headingangle1 = (ptr1->TargetWaypoint - ptr1->StartWaypoint).Angle();
            argos::CRadians TurningAngle1 = (ptr1->Orientation - Headingangle1).SignedNormalize();
            argos::Real TurningAngle1_Deg = ToDegrees(TurningAngle1).GetValue();
            
            if(TurningAngle1_Deg < 0.0)
            {
                TurningAngle1_Deg = -TurningAngle1_Deg;
            }
            else
            {
                TurningAngle1_Deg = TurningAngle1_Deg;
            }
            
            TimeToTurn1 = GetTimeToTurn(TurningAngle1_Deg, ptr1->fBaseAngularWheelSpeed) + SafeTurningTime;
            
            DistanceToTarget1 = CalculateDistance(ptr1->StartWaypoint, ptr1->TargetWaypoint);
            Time1 = GetTicksToWait(DistanceToTarget1, ptr1->fLinearWheelSpeed);
            
            TotalTime1 = TimeToTurn1 + Time1 + ptr1->StopTurningTime + SafeTurningTime;
            
            
            argos::CRadians Headingangle2 = (ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle();
            argos::CRadians TurningAngle2 = (ptr2->Orientation - Headingangle2).SignedNormalize();
            argos::Real TurningAngle2_Deg = ToDegrees(TurningAngle2).GetValue();
            
            if(TurningAngle2_Deg < 0.0)
            {
                TurningAngle2_Deg = -TurningAngle2_Deg;
            }
            else
            {
                TurningAngle2_Deg = TurningAngle2_Deg;
            }
            
            TimeToTurn2 = GetTimeToTurn(TurningAngle2_Deg, ptr2->fBaseAngularWheelSpeed) + SafeTurningTime;
            
            DistanceToTarget2 = CalculateDistance(ptr2->StartWaypoint, ptr2->TargetWaypoint);
            Time2 = GetTicksToWait(DistanceToTarget2, ptr2->fLinearWheelSpeed);
            
            TotalTime2 = TimeToTurn2 + Time2 + ptr2->StopTurningTime + SafeTurningTime;
            
            TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , ptr1->fLinearWheelSpeed) + Bufferime);
            
            // reset the collinear flag if the the collinear robots have safe time to reach their respective targets
            if(abs(TotalTime1 - TotalTime2) > TicksToWaitforSafedistance)
            {
                ptr1->CollinearFlag = false;
                ptr2->CollinearFlag = false;
            }
     
            
        }
    }// end of if(ptr1->CollinearFlag == true and ptr2->CollinearFlag == true)

    
}



/*************************************************************************************************************************************************************************************************/
/* Function to check if robot start and target points are collinear */
/*************************************************************************************************************************************************************************************************/
bool DSA_loop_functions::WayPointCollinearityCheck(argos::CVector2 Pt1,argos::CVector2 Pt2 , argos::CVector2 Pt3, argos::CVector2 Pt4,
                                                   BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
    //    argos::CVector3 v1, v2, cross_product, AddedWaypoint, ZeroVector;
    //    argos::Real v1length, v2length, dot_product, DegreeAngle, shortest_dist, CosTheta;
    argos::Real vec1length, vec2length,shortest_dist, CosTheta, dot_product;
    argos::CVector2 vec1, vec2;
    bool CollinearFlag;
    argos::UInt16 timetowait;
    
    argos::Real epsilon = 0.001;
    argos::Real CosTheta_UpperRange = 1.0;
    argos::Real CosTheta_LowerRange = 0.93;
    
    
    CollinearFlag = false;
    vec1.Set((Pt1.GetX() - Pt2.GetX()), (Pt1.GetY() - Pt2.GetY()));
    vec2.Set((Pt3.GetX() - Pt4.GetX()), (Pt3.GetY() - Pt4.GetY()));
    
    vec1length = vec1.Length();
    vec2length = vec2.Length();
    
    dot_product = vec1.DotProduct(vec2);
    
    CosTheta = (dot_product/(vec1length * vec2length));
    
    // get the shortest distance between two vectors
    shortest_dist = ShortestDistTwoSegments(Pt1, Pt2, Pt3, Pt4);
    
    
    // angle = 0 to 21 deg celsius or 180 to 158
    //    if(abs(CosTheta) <= 1 and abs(CosTheta) >= 0.93)
    
    //angle ->180 to 152.87 and 0 to 27.12
    if(((fabs(CosTheta) - CosTheta_UpperRange) <= epsilon or fabs(CosTheta) < CosTheta_UpperRange ) and
       ((fabs(CosTheta) - CosTheta_LowerRange) <= epsilon or fabs(CosTheta) > CosTheta_LowerRange))
    {
        if(ptr1->GoingToOrFromNest == true and ptr2->GoingToOrFromNest == true)
        {
            if((shortest_dist <= CollinearDistanceGap))
            {
                CollinearFlag = true;

            }
        }
    }
    
    
    return CollinearFlag;
    
    
}


/************************************************************************************************************************************************************************/
/* Function to find the shortest distance between two segments */
/***********************************************************************************************************************************************************************/
argos::Real DSA_loop_functions::ShortestDistTwoSegments(argos::CVector2 Pt1, argos::CVector2 Pt2 , argos::CVector2 Pt3,
                                                        argos::CVector2 Pt4)
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
    distance[0] = DistancePointSegment(Pt1, Pt2, Pt3);
    //    distance[0] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, ptr2.StartWaypoint);
    
    // dist between robot 1 start, goal and robot2 goal
    distance[1] = DistancePointSegment(Pt1, Pt2, Pt4);
    //    distance[1] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, TargetPoint2);
    
    // dist between robot 2 start, goal and robot1 start
    distance[2] = DistancePointSegment(Pt3, Pt4, Pt1);
    //    distance[2] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, ptr1.StartWaypoint);
    
    // dist between robot 2 start, goal and robot2 goal
    distance[3] = DistancePointSegment(Pt3, Pt4, Pt2);
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



/************************************************************************************************************************************************************************************/
/* Function to check if robot course is very close */
/************************************************************************************************************************************************************************************/
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



/*****************************************************************************************************************/
/* Function to find the target point on the circle */
/*****************************************************************************************************************/
void DSA_loop_functions::GetPointAtSafeDistance(BaseController::RobotData *ptr)
{
  
    argos::Real x, y;
    argos::UInt8 i;
    char previous_direction, direction;
    argos::CVector2 PointAtSafeDistance;
    argos::CVector2 PointChangeDirection;
    argos::UInt16 length;
    
    argos::UInt8 MaxLoopValue;
    
    x=0;
    y=0;
    i=1;
    PointChangeDirection.Set(0,0);
    PointAtSafeDistance.Set(0,0);
    
    x = ptr->TargetWaypoint.GetX();
    y = ptr->TargetWaypoint.GetY();

    
    PointChangeDirection.Set(x, y);
    
    // third spiral is 0.54 dist...however, the diagonal length is sqrt((0.54)^2 + (0.54)^2)
    if((ptr->StartWaypoint - ptr->TargetWaypoint).Length() >= 0.7)
    {
        MaxLoopValue = 8;
    }
    else{
        MaxLoopValue = 5;
    }
    
    
    if(ptr->pattern.size() > 0)
    {
        length = ptr->pattern.size();
        previous_direction = ptr->pattern[length - 1];
        
        for(i = 1; i < MaxLoopValue; i++)
        {
            if((length - i) > 0)
            {
                direction = ptr->pattern[length - i];

                switch(direction)
                {
                    case 'N':

                        if(previous_direction != direction)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x + Searcher_Gap;
                        y = y;
                        previous_direction = direction;
                        break;
                        
                    case 'S':
                        if(previous_direction != direction)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x - Searcher_Gap;
                        y = y;
                        previous_direction = direction;
                        break;
                        
                    case 'E':
                        if(previous_direction != direction)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x;
                        y = y - Searcher_Gap;

                        previous_direction = direction;
                        break;
                        
                    case 'W':
                        if(previous_direction != direction)
                        {
                            PointChangeDirection.Set(x,y);
                        }
                        x = x;
                        y = y + Searcher_Gap;

                        previous_direction = direction;
                        break;
                        
                    default:
                        break;
                } // end of switch
            }// end of if (length-i > 0)
        } // end of for

        PointAtSafeDistance.Set(x, y);
//        PointAtSafeDistance.Set(x, y);
    }// end of if
    
    else if(ptr->pattern.size() == 0)
    {
        /* do nothing */
        PointAtSafeDistance.Set(x, y);

    }
 
    ptr->IntersectionPt1 = PointChangeDirection;
    ptr->IntersectionPt2 = PointAtSafeDistance;
}



/*************************************************************************************************************************************************************/
/* Function to check if robot paths intersect */
/* CVector2 pt1, CVector2 pt2: Start point and end point of robot1
 * CVector2 pt3, CVector2 pt4: Start point and end point of robot2
 * std::vector<BaseController::IntersectionData> *ptr3 : Intersection data for robot1
 * std::vector<BaseController::IntersectionData> *ptr4 : Intersection data for robot2
 * argos::UInt16 RobotId1, argos::UInt16 RobotId2: Robot Ids of robot 1 and robot2 */
/*************************************************************************************************************************************************************/
argos::UInt8 DSA_loop_functions::Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
                                           std::vector<BaseController::IntersectionData> *ptr3,
                                           std::vector<BaseController::IntersectionData> *ptr4,
                                           BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, bool CollinearIntersection)
{
    argos::Real x_inter;
    argos::Real y_inter;
    argos::Real A1, A2, B1, B2, C1, C2, det;
    UInt8 Intersection_flag = 0;
    CVector2 IntersectionPoint;
    argos::Real IntersectionDist1, IntersectionDist2;
    argos::UInt8 IntersectionType;
    
    x_inter = 0;
    y_inter = 0;
    IntersectionPoint.Set(0,0);
    
    /* get the start and target position of robot 1 */
    argos::CVector2 Point1 = pt1;
    argos::CVector2 Point2 = pt2;
    
    /* get the start and target position of robot 2 */
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
        Intersection_flag = 2;
    }
    
    /* Lines intersect and find the intersection point */
    else
    {
        x_inter = (B2*C1 - B1*C2)/det;
        
        y_inter = (A1*C2 - A2*C1)/det;
        
        /* Check if intersection point is out of bound for the line segment */
        if((x_inter < std::max(std::min(Point1.GetX(), Point2.GetX()),
                               std::min(Point3.GetX(), Point4.GetX()))) or
           (x_inter > std::min(std::max(Point1.GetX(), Point2.GetX()),
                               std::max(Point3.GetX(), Point4.GetX()))))
            
        {
      
            Intersection_flag = 0;
        }
        else if((y_inter < std::max(std::min(Point1.GetY(), Point2.GetY()),
                                    std::min(Point3.GetY(), Point4.GetY()))) or
                (y_inter > std::min(std::max(Point1.GetY(), Point2.GetY()),
                                    std::max(Point3.GetY(), Point4.GetY()))))
        {
            Intersection_flag = 0;
        }
        else
        {
            Intersection_flag = 1;
       
            
        }
    }
    IntersectionPoint.Set(x_inter, y_inter);
    

    // TODO: check how to push only a particular element to a vector top data.
    // start point will be the Startwaypoint and distance from intersection can be calculated from the ptr to the robot data
    if(Intersection_flag == 1)
    {
        IntersectionPointLF.Set(x_inter, y_inter);
        if(ptr1->StartWaypoint == pt1)
        {
            // if pt1 is startpoint, then distance to intersection is distance between starpoint and intersection point
            IntersectionDist1 = CalculateDistance(ptr1->StartWaypoint, IntersectionPoint);
            if(CollinearIntersection == true)
            {
                IntersectionType = COLLINEAR_INTERSECTION;
            }
            else
            {
                IntersectionType = INTERSECTION1;
            }
            
        }
        else
        {
            // if pt1 is not the startpoint, then distance to intersection is distance between starpoint and IntersectionPoint1 and
            // distance between IntersectionPoint1 and IntersectionPoint
            IntersectionDist1 = CalculateDistance(ptr1->StartWaypoint, ptr1->IntersectionPt1) +
                                CalculateDistance(ptr1->IntersectionPt1, IntersectionPoint);
            
            if(CollinearIntersection == true)
            {
                IntersectionType = COLLINEAR_INTERSECTION;
            }
            else
            {
                IntersectionType = INTERSECTION2;
            }
        }
        
        
        if(ptr2->StartWaypoint == pt3)
        {
            // if pt3 is startpoint, then distance to intersection is distance between starpoint and intersection point
            IntersectionDist2 = CalculateDistance(ptr2->StartWaypoint, IntersectionPoint);
            
            if(CollinearIntersection == true)
            {
                IntersectionType = COLLINEAR_INTERSECTION;
            }
            else
            {
                IntersectionType = INTERSECTION1;
            }
        }
        else
        {
            // if pt3 is not the startpoint, then distance to intersection is distance between starpoint and IntersectionPoint1 and
            // distance between IntersectionPoint1 and IntersectionPoint
            IntersectionDist2 = CalculateDistance(ptr2->StartWaypoint, ptr1->IntersectionPt1) +
                                CalculateDistance(ptr2->IntersectionPt1, IntersectionPoint);
            if(CollinearIntersection == true)
            {
                IntersectionType = COLLINEAR_INTERSECTION;
            }
            else
            {
                IntersectionType = INTERSECTION2;
            }
        }
        ptr3->push_back({ptr2->id_robot,Intersection_flag, IntersectionType,0,IntersectionDist1, pt1, pt2, IntersectionPoint});
        ptr4->push_back({ptr1->id_robot,Intersection_flag, IntersectionType,0,IntersectionDist2, pt3, pt4, IntersectionPoint});
        
    }
    
    
    return Intersection_flag;
}





REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
