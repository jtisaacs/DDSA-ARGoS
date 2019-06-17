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
    PrintFinalScore(1),
    Searcher_Gap(0.18),
    SimulatorTicksperSec(0),
    // third circuit
    AnchorDistance(0.54),
    m_pcRNG(NULL)
{}

void DSA_loop_functions::Init(TConfigurationNode& node) {
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
 argos::GetNodeAttribute(DDSA_node, "RobotNum",      RobotNumber);
 argos::GetNodeAttribute(DDSA_node, "NeighborRadius",      Neighbor_Radius);
 argos::GetNodeAttribute(DDSA_node, "ResultsPath",      file_path);

    

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
  
    argos::Real rangeX = FoodBoundsWidth/2.0;//(ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = FoodBoundsHeight/2.0;//(ArenaSize.GetY() / 2.0) - 0.085;  
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    TotalRobots = footbots.size();
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
   
    RobotReachedWayPoint = 0;
    FirstCheck = 0;
    
    TestValue = 10;
    TestVariable = 11;
    
    Result_Checked = 1;
    // Name the results file with the current time and date
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    stringstream ss;
    
    ss << "DSA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
    << (now->tm_year) << '-'
    << (now->tm_mon + 1) << '-'
    <<  now->tm_mday << '-'
    <<  now->tm_hour << '-'
    <<  now->tm_min << '-'
    <<  now->tm_sec << ".csv";
    
    file_name = ss.str();
    full_path = file_path+"/"+file_name;
    
    ofstream results_output_stream;
    results_output_stream.open(full_path, ios::app);
    results_output_stream <<"Random Seed, " <<"Number_Of_Robots, "<<"Simulator_Clock, "
    << "Number_Of_Targets"<< endl;
    results_output_stream.close();
    
}


double DSA_loop_functions::Score()
{  
  return score;
}


void DSA_loop_functions::setScore(double s)
{
  score = s;


//    ofstream results_output_stream;
//    results_output_stream.open(full_path, ios::app);
//    results_output_stream<< Random_Seed << "," << TotalRobots << ", "
//    << getSimTimeInSeconds() << ", "
//    << score << endl;
//    results_output_stream.close();
    
  if (score >= FoodItemCount)
    {
      PostExperiment();
      exit(0);
    }
}

void DSA_loop_functions::PostExperiment() 
{
  if (PrintFinalScore == 1) printf("%f, %f\n", getSimTimeInSeconds(), score);
    
   
    ofstream results_output_stream;
    results_output_stream.open(full_path, ios::app);
    results_output_stream<< Random_Seed << "," <<TotalRobots << ", "
    << getSimTimeInSeconds() << ", "
    << score << endl;
    results_output_stream.close();
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
    
//    BaseController::RobotData *stRobotDataCopy1 = NULL;
//    CSpace::TMapPerType::iterator it10 = m_cFootbots.begin();
//    advance(it10,3);
//    CFootBotEntity& cFootBottrial  = *any_cast<CFootBotEntity*>(it10->second);
//    BaseController& cControllertrial = dynamic_cast<BaseController&>(cFootBottrial.GetControllableEntity().GetController());
//    stRobotDataCopy1 = &cControllertrial.GetRobotData();
//    RobotIDTrial = stRobotDataCopy1->id_robot;

    
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
  
    /* Get the hadndle to each robot and check if any one waypoint reached*/
   
//    /CSpace::TMapPerType::iterator it10 =
    
    /* check collinearity and intersection if target reached or new waypoint added or its the start of code */
    if(FirstCheck == 1 and  NewWayPointAdded == 1)
    {
        
        
        RobotResourceSize = RobotResource.size();
        /* Stop all the robots before checking for collision*/
        StopAllRobots();

         /* Clear the vector data of all the robots before checking for collision */
        ClearRobotVectorData();

        /* Get the neighbors of the robots who have collected the resource*/
        GetNeighbors();
        
        
        /* Update the matrix for the robot regarding collinearity */
        CheckCollisionWithNeighbors(true);

        /* implement collision avoidance algorithm */
        Avoid_Collision();
        
        /* Check the collision and avoid collision for inersection and collinearity */
        CheckCollisionAndConsistency();

        /* set the flags to indicate that collision checks have been completed */
        CheckComplete();
        
//        BaseController::RobotData *stRobotDataCurrent = NULL;
//        BaseController::RobotData *stRobotDataNeighbor = NULL;
        
        //Stop all the robots
//        for(CSpace::TMapPerType::iterator it4 = m_cFootbots.begin();
//            it4 != m_cFootbots.end();
//            ++it4)
//        {
//            /* Get handle to foot-bot entity and controller of the current robot */
//            CFootBotEntity& cFootBot4 = *any_cast<CFootBotEntity*>(it4->second);
//            BaseController& cController4 = dynamic_cast<BaseController&>(cFootBot4.GetControllableEntity().GetController());
//
//            cController4.SetHardStopMovement();
////            cController4.SetStopMovement();
//
//        }
        
        //find neighbors of every robot and collinear and intersecting robots
//        for(CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin(); iterator1 != m_cFootbots.end(); ++iterator1)
//        {
//
//            /* Get handle to foot-bot entity and controller of the current robot */
//            CFootBotEntity& cFootBotCurrent = *any_cast<CFootBotEntity*>(iterator1->second);
//            BaseController& cControllerCurrent = dynamic_cast<BaseController&>(cFootBotCurrent.GetControllableEntity().GetController());
//            stRobotDataCurrent = &cControllerCurrent.GetRobotData();
//            stRobotDataCurrent->StartWaypoint = cControllerCurrent.GetPosition();
//
//            for(CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin(); iterator2 != m_cFootbots.end(); ++iterator2)
//            {
//
//                /* Get handle to foot-bot entity and controller of the current robot */
//                CFootBotEntity& cFootBotNeighbors = *any_cast<CFootBotEntity*>(iterator2->second);
//                BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbors.GetControllableEntity().GetController());
//                stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();
//
//                //avoid comparing the robot to itself
//                if(stRobotDataNeighbor->id_robot != stRobotDataCurrent->id_robot)
//                {
//                    stRobotDataNeighbor->StartWaypoint = cControllerNeighbor.GetPosition();
//
//                    DistanceBetweenRobots = CalculateDistance(stRobotDataCurrent->StartWaypoint, stRobotDataNeighbor->StartWaypoint);
//                    // if the distance between two robots is less than the user configured
//                    if(DistanceBetweenRobots < Neighbor_Radius)
//                    {
//                        stRobotDataCurrent->Neighbors.push(stRobotDataNeighbor->id_robot);
//                        stRobotDataNeighbor->Neighbors.push(stRobotDataCurrent->id_robot);
//                    }
//                }
//
//            }
//
//        }
        
//        for(CSpace::TMapPerType::iterator it4 = m_cFootbots.begin();
//            it4 != m_cFootbots.end();
//            ++it4)
//        {
//            /* Get handle to foot-bot entity and controller of the current robot */
//            CFootBotEntity& cFootBot4 = *any_cast<CFootBotEntity*>(it4->second);
//            BaseController& cController4 = dynamic_cast<BaseController&>(cFootBot4.GetControllableEntity().GetController());
//
//            cController4.SetHardStopMovement();
//            cController4.SetStopMovement();
//
//        }
        
        /* Get the hadndle to each robot */
//
//        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
//            it != std::prev(m_cFootbots.end());
//            ++it)
//
//        {
//
//            /* Get handle to foot-bot entity and controller of the current robot */
//            CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
//            BaseController& cController = dynamic_cast<BaseController&>(cFootBot.GetControllableEntity().GetController());
//
//
//            sRobotDataprevious = &cController.GetRobotData();
//
//            /* Get the hadndle to each next robot */
//            for(CSpace::TMapPerType::iterator it1 = std::next(it, 1);
//                it1 != m_cFootbots.end();
//                ++it1)
//
//            {
//
//                CFootBotEntity& cFootBot1 = *any_cast<CFootBotEntity*>(it1->second);
//                BaseController& cController1 = dynamic_cast<BaseController&>(cFootBot1.GetControllableEntity().GetController());
//
//
//                sRobotDatanext = &cController1.GetRobotData();
//                sIntersectionDataNextRobot = &cController1.GetIntersectionData();
//
//                //                cController.SetHardStopMovement();
//                //                cController1.SetHardStopMovement();
//
//                sRobotDataprevious->StartWaypoint = cController.GetPosition();
//                sRobotDatanext->StartWaypoint = cController1.GetPosition();
//
//                // Reset the collinearity flag
//                sRobotDataprevious->CollinearFlag = false;
//                sRobotDatanext->CollinearFlag = false;
//
//                DistanceRobots = CalculateDistance(sRobotDataprevious->StartWaypoint, sRobotDatanext->StartWaypoint);
//
//                // check if two robots are neighbors
//                if(DistanceRobots < Neighbor_Radius)
//                {
//                    if(sRobotDatanext->WaypointStackpopped == false)
//                    {
//                        /* check if robot's end waypoint is collinear in other robot's start and end waypoint */
//                        CheckCollinearity(sRobotDataprevious, sRobotDatanext);
//
//                        if(sRobotDataprevious->CollinearFlag == true and !sRobotDatanext->WaypointStack.empty())
//                        {
//                            sRobotDatanext->AddedPoint = sRobotDatanext->WaypointStack.top();
//
//                            cController1.SetTarget(sRobotDatanext->AddedPoint);
//                            sRobotDatanext->WaypointStackpopped = true;
//                            sRobotDatanext->TargetWaypoint = sRobotDatanext->AddedPoint;
//                            //                            cController.SetHardStopMovement();
//                            cController1.SetHardStopMovement();
//                            sRobotDatanext->WaypointStack.pop();
//
//
//                        }
//                    }
//
//                    if(sRobotDataprevious->CollinearFlag != true)
//                    {
//
//                        //                            cController.SetHardStopMovement();
//                        //                            cController1.SetHardStopMovement();
//                        //
//                        sRobotDataprevious->StartWaypoint = cController.GetPosition();
//                        sRobotDatanext->StartWaypoint = cController1.GetPosition();
//
//                        if((sRobotDataprevious->WaypointStackpopped == true or sRobotDataprevious->GoingToOrFromNest == true)
//                           and (sRobotDatanext->GoingToOrFromNest == false))
//                        {
//
//
//                            GetPointAtSafeDistance(sRobotDatanext);
//
//
//                            Find_Intersection(sRobotDataprevious->StartWaypoint, sRobotDataprevious->TargetWaypoint,
//                                              sRobotDatanext->StartWaypoint, sRobotDatanext->IntersectionPt1,sIntersectionDataNextRobot);
//
//                            if(sIntersectionDataNextRobot->Intersection_flag == true)
//                            {
//                                sIntersectionDataNextRobot->Robot_ID_Intersectingwith = sRobotDataprevious->id_robot;
//                                sRobotDatanext->Inter = 1;
//
//                                IntersectionCollisionCheck(sRobotDataprevious->StartWaypoint,
//                                                           sRobotDatanext->StartWaypoint,sRobotDataprevious,
//                                                           sRobotDatanext,sIntersectionDataNextRobot,1);
//
//                            }
//                            else
//                            {
//
//                                Find_Intersection(sRobotDataprevious->StartWaypoint, sRobotDataprevious->TargetWaypoint,
//                                                  sRobotDatanext->IntersectionPt1,
//                                                  sRobotDatanext->IntersectionPt2,sIntersectionDataNextRobot);
//
//                                if(sIntersectionDataNextRobot->Intersection_flag == true)
//                                {
//                                    sIntersectionDataNextRobot->Robot_ID_Intersectingwith = sRobotDataprevious->id_robot;
//                                    sRobotDatanext->Inter = 2;
//
//                                    IntersectionCollisionCheck(sRobotDataprevious->StartWaypoint, sRobotDatanext->IntersectionPt1,          sRobotDataprevious, sRobotDatanext,sIntersectionDataNextRobot,2);
//                                }
//                            }
//
//
//                        }
//                        else if((sRobotDatanext->WaypointStackpopped == true or sRobotDatanext->GoingToOrFromNest == true)
//                                and(sRobotDataprevious->GoingToOrFromNest == false))
//                        {
//
//
//                            GetPointAtSafeDistance(sRobotDataprevious);
//
//                            Find_Intersection(sRobotDatanext->StartWaypoint,sRobotDatanext->TargetWaypoint,
//                                              sRobotDataprevious->StartWaypoint, sRobotDataprevious->IntersectionPt1, sIntersectionDataNextRobot);
//
//                            if(sIntersectionDataNextRobot->Intersection_flag == true)
//                            {
//                                sIntersectionDataNextRobot->Robot_ID_Intersectingwith = sRobotDataprevious->id_robot;
//                                sRobotDatanext->Inter = 3;
//
//                                IntersectionCollisionCheck(sRobotDatanext->StartWaypoint, sRobotDataprevious->StartWaypoint,
//                                                           sRobotDatanext, sRobotDataprevious,sIntersectionDataNextRobot,1);
//
//                            }
//                            else
//                            {
//                                Find_Intersection(sRobotDatanext->StartWaypoint,sRobotDatanext->TargetWaypoint,
//                                                  sRobotDataprevious->IntersectionPt1, sRobotDataprevious->IntersectionPt2, sIntersectionDataNextRobot);
//
//                                if(sIntersectionDataNextRobot->Intersection_flag == true)
//                                {
//                                    sIntersectionDataNextRobot->Robot_ID_Intersectingwith = sRobotDataprevious->id_robot;
//                                    sRobotDatanext->Inter = 4;
//
//                                    IntersectionCollisionCheck(sRobotDatanext->StartWaypoint, sRobotDataprevious->IntersectionPt1,
//                                                               sRobotDatanext, sRobotDataprevious, sIntersectionDataNextRobot,2);
//
//                                }
//                            }
//
//                        }
//                        //                        both are coming, going or one is going towards nest and other is coming back from nest
////                            else if((sRobotDatanext->WaypointStackpopped == true and sRobotDataprevious->GoingToOrFromNest == true) or
////                                    (sRobotDataprevious->WaypointStackpopped == true and sRobotDatanext->GoingToOrFromNest == true))
////                            {
////                                Find_Intersection(sRobotDatanext->StartWaypoint,sRobotDatanext->TargetWaypoint,
////                                                  sRobotDataprevious->StartWaypoint, sRobotDataprevious->TargetWaypoint, sIntersectionDataNextRobot);
////
////                                if(sIntersectionDataNextRobot->Intersection_flag == true)
////                                {
////                                    sIntersectionDataNextRobot->Robot_ID_Intersectingwith = sRobotDataprevious->id_robot;
////                                    sRobotDatanext->Inter = 5;
////                                    IntersectionCollisionCheck(sRobotDatanext->StartWaypoint, sRobotDataprevious->StartWaypoint,
////                                                               sRobotDatanext, sRobotDataprevious, sIntersectionDataNextRobot,1);
////                                }
////
////
////                            }
////
//                        }// end of intersection check
//
//                    }//end of if distance check
//
//            } /* end of inner for loop */
////            sRobotDataprevious->WaypointStackpopped =  false;
////            sRobotDatanext->WaypointStackpopped =  false;
//        } /* end of outer for loop */
        
//        for(CSpace::TMapPerType::iterator it5 = m_cFootbots.begin();
//            it5 != m_cFootbots.end();
//            ++it5)
//        {
//            BaseController::RobotData* stRobotData_1 = NULL;
//            BaseController::IntersectionData *sIntersectionData_1 = NULL;
//            /* Get handle to foot-bot entity and controller of the current robot */
//            CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
//            BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
//
//            stRobotData_1 = &cController5.GetRobotData();
//            sIntersectionData_1 =&cController5.GetIntersectionData();
//
//            stRobotData_1->pathcheck = true;
//            stRobotData_1->WaypointStackpopped =  false;
//            stRobotData_1->CollinearFlag =  false;
//            sIntersectionData_1->Intersection_flag = false;
//
////            cController5.SetStopMovement();
//            cController5.SetMovement();
//        }

        
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
        // clar the intersection vector
        stIntersectionData->clear();
    }
        
}


/****************************************************************************************************************/
/* Function to reset all robot vector data */
/****************************************************************************************************************/
void DSA_loop_functions::TestFunction()
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
        
        stRobotData_1->StopTurningTime = 50;
//        //clear neighbor matrix
//        stRobotData_1->NeighborsMatrix.clear();
//        stRobotData_1->Neighbors.clear();
//        // clar the intersection vector
//        stIntersectionData->clear();
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
//        std::vector<BaseController::IntersectionData>* stIntersectionData = NULL;

        /* Get handle to foot-bot entity and controller of the current robot */
        CFootBotEntity& cFootBot5 = *any_cast<CFootBotEntity*>(it5->second);
        BaseController& cController5 = dynamic_cast<BaseController&>(cFootBot5.GetControllableEntity().GetController());
        
        stRobotData_1 = &cController5.GetRobotData();
//        stIntersectionData = &cController5.GetIntersectionData();

        
        stRobotData_1->pathcheck = true;
        stRobotData_1->WaypointStackpopped =  false;
        stRobotData_1->CollinearFlag =  false;
        
 
        
//        sIntersectionData_1->Intersection_flag = false;
 
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
        for(CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin(); iterator2 != m_cFootbots.end(); ++iterator2)
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
                if(DistanceBetweenRobots < Neighbor_Radius)
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
    //
//    else
//    {
//        IntersectionFlag_ret = Find_Intersection(ptr1->StartWaypoint, ptr1->TargetWaypoint,
//                                                 ptr2->StartWaypoint, ptr2->TargetWaypoint, ptr3, ptr4,
//                                                 ptr1->id_robot, ptr2->id_robot, ptr1, ptr2);
//        // if the robots are intersecting, update the matrix
//        if(IntersectionFlag_ret == true)
//        {
//            IntersectionLoopValue = 5;
//            InitializeMatrixElementAndTransformElement(ptr,ptr1_index,ptr2_index,INTERSECTION1);
//        }
//    }
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
//        stIntersectionDataThis = &cControllerThis.GetIntersectionData();
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
//                stIntersectionDataNeighbor = &cControllerNeighbor.GetIntersectionData();
                stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();

                stRobotDataThis->CollinearFlag = false;
                stRobotDataNeighbor->CollinearFlag = false;

                // check if the robots are collinear
                // ptr to the robot data of both the robots
                CheckCollinearity(stRobotDataThis, stRobotDataNeighbor);

                // update the matrix if the robots are collinear
                if(stRobotDataThis->CollinearFlag == true)
                {
//                    // add waypoint if the robot is collinear and going out of nest
//                    AddWayPoint(stRobotDataThis, stRobotDataNeighbor);
                    
//                    // add waypoint if the robot is collinear and going out of nest
//                    AddWayPoint(stRobotDataNeighbor, stRobotDataNeighbor);
                    
                    // ptr for matrix to update, matrix indices of the robots, value to update
                    InitializeMatrixElementAndTransformElement(stRobotDataThis,0,(robotneighborindex+1),COLLINEAR);

                    // Reset the collinearity flag
                    stRobotDataThis->CollinearFlag = false;
                    stRobotDataNeighbor->CollinearFlag = false;

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

                    // check if the robots are collinear
                    CheckCollinearity(stRobotDataNeighbor1st, stRobotDataNeighbor2nd);
                    
                    //update the matrix if robots are collinear
                    if(stRobotDataNeighbor1st->CollinearFlag == true)
                    {
                        InitializeMatrixElementAndTransformElement(stRobotDataThis,(neighbor+1),(neighbor+2),COLLINEAR);
                        stRobotDataNeighbor1st->CollinearFlag = false;
                        stRobotDataNeighbor2nd->CollinearFlag = false;
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
         
                //get the handle to next neighbor robot if the neighbor robot is not the last neighbor in the neihbor vector
                // to complete the neighbor matrix of the current matrix
//                if(robotneighborindex != (stRobotDataThis->Neighbors.size() - 1))
//                {
//                    iterator2 = m_cFootbots.begin();
//                    std::advance(iterator2, stRobotDataThis->Neighbors[robotneighborindex+1]);
//                    CFootBotEntity& cFootBotNeighborNext = *any_cast<CFootBotEntity*>(iterator2->second);
//                    BaseController& cControllerNeighborNext = dynamic_cast<BaseController&>(cFootBotNeighborNext.GetControllableEntity().GetController());
//                    stRobotDataNeighborNext = &cControllerNeighborNext.GetRobotData();
////                    stIntersectionDataNeighborNext = &cControllerNeighborNext.GetIntersectionData();
//                    stIntersectionDataNeighborNext = cControllerNeighborNext.GetIntersectionData();
//
//                    stRobotDataNeighborNext->CollinearFlag = false;
//                    stRobotDataNeighbor->CollinearFlag = false;
//
//                    // check if the robots are collinear
//                    CheckCollinearity(stRobotDataNeighbor, stRobotDataNeighborNext);
//
//                    // update the matrix if robots are collinear
//                    if(stRobotDataNeighbor->CollinearFlag == true)
//                    {
//                        InitializeMatrixElementAndTransformElement(stRobotDataThis,(robotneighborindex+1),(robotneighborindex+2),COLLINEAR);
//                        stRobotDataNeighborNext->CollinearFlag = false;
//                        stRobotDataNeighbor->CollinearFlag = false;
//                    }
//                    else
//                    {
//
//                        // Update the matrix for intersection of robots
//                        IntersectionCheckModule(stRobotDataThis,stRobotDataNeighbor, stRobotDataNeighborNext,
//                                                stIntersectionDataNeighbor, stIntersectionDataNeighborNext, (robotneighborindex+1),(robotneighborindex+2));
//                    }
//                }
//
               
    
    
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

/****************************************************************************************************************/
/* Function to check which anchor point is closest to the collinear robots */
/****************************************************************************************************************/
argos::CVector2 DSA_loop_functions::FindClosestAnchorPoint(BaseController::RobotData *ptr, BaseController::RobotData *Otherptr)
{
    argos::Real Distance;
    argos::Real DistancePoint1, DistancePoint2, DistancePoint3, DistancePoint4, MaxDistance;
    argos::Real SmallerDistance, CurrentDistance;
    argos::CVector2  ClosestAnchorPoint;
    bool CollinearFlag1, CollinearFlag2;
    argos::Real DistanceArray[8];
    argos::UInt8 CollinearArray[8];
    argos::UInt8 index, index1, index2, SmallestDistanceIndex;
    
    MaxDistance = 1000;
    
    // calculate the distance
    for(index = 0; index< 8; index++)
    {
        Distance = CalculateDistance(ptr->StartWaypoint, AnchorPoints[index]);
        DistanceArray[index] = Distance;
    }
    
    // Check if any of the anchor point is collinear
    for(index1 = 0; index1 < 8; index1++)
    {
        CollinearFlag1 = false;
        CollinearFlag2 = false;
        CollinearFlag1 = ThreePointsCollinear(AnchorPoints[index1], ptr->StartWaypoint, ptr->TargetWaypoint);
        CollinearFlag2 = ThreePointsCollinear(AnchorPoints[index1], Otherptr->StartWaypoint, Otherptr->TargetWaypoint);
        if(CollinearFlag1 & CollinearFlag2)
        {
            CollinearArray[index1] = 1;
        }
        else
        {
            CollinearArray[index1] = 0;
        }
    }
    
    SmallerDistance = 50;
    SmallestDistanceIndex = 0;
    
    // Check which is the smallest distance
    for(index2 = 0; index2 < 8; index2++)
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
    ClosestAnchorPoint = AnchorPoints[SmallestDistanceIndex];
    
    
//    DistancePoint1 = CalculateDistance(ptr->StartWaypoint, First_QuadrantAnchorPoint);
//    DistancePoint2 = CalculateDistance(ptr->StartWaypoint, Second_QuadrantAnchorPoint);
//    DistancePoint3 = CalculateDistance(ptr->StartWaypoint, Third_QuadrantAnchorPoint);
//    DistancePoint4 = CalculateDistance(ptr->StartWaypoint, Fourth_QuadrantAnchorPoint);
//
//    // find if the anchor points are collinear to the robots
//    CollinearFlag1 = false;
//    CollinearFlag2 = false;
//    CollinearFlag1 = ThreePointsCollinear(First_QuadrantAnchorPoint, ptr->StartWaypoint, ptr->TargetWaypoint);
//    CollinearFlag2 = ThreePointsCollinear(First_QuadrantAnchorPoint, Otherptr->StartWaypoint, Otherptr->TargetWaypoint);
//    if(CollinearFlag1 & CollinearFlag2)
//    {
//        DistancePoint1 = MaxDistance;
//    }
//
//    CollinearFlag1 = false;
//    CollinearFlag2 = false;
//    CollinearFlag1 = ThreePointsCollinear(Second_QuadrantAnchorPoint, ptr->StartWaypoint, ptr->TargetWaypoint);
//    CollinearFlag2 = ThreePointsCollinear(Second_QuadrantAnchorPoint, Otherptr->StartWaypoint, Otherptr->TargetWaypoint);
//    if(CollinearFlag1 & CollinearFlag2)
//    {
//        DistancePoint2 = MaxDistance;
//    }
//
//    CollinearFlag1 = false;
//    CollinearFlag2 = false;
//    CollinearFlag1 = ThreePointsCollinear(Third_QuadrantAnchorPoint, ptr->StartWaypoint, ptr->TargetWaypoint);
//    CollinearFlag2 = ThreePointsCollinear(Third_QuadrantAnchorPoint, Otherptr->StartWaypoint, Otherptr->TargetWaypoint);
//    if(CollinearFlag1 & CollinearFlag2)
//    {
//        DistancePoint3 = MaxDistance;
//    }
//
//    CollinearFlag1 = false;
//    CollinearFlag2 = false;
//    CollinearFlag1 = ThreePointsCollinear(Fourth_QuadrantAnchorPoint, ptr->StartWaypoint, ptr->TargetWaypoint);
//    CollinearFlag2 = ThreePointsCollinear(Fourth_QuadrantAnchorPoint, Otherptr->StartWaypoint, Otherptr->TargetWaypoint);
//    if(CollinearFlag1 & CollinearFlag2)
//    {
//        DistancePoint4 = MaxDistance;
//    }
//
//    // find the closest anchor point
//    if(DistancePoint2 < DistancePoint1)
//    {
//        ClosestAnchorPoint = Second_QuadrantAnchorPoint;
//        SmallerDistance = DistancePoint2;
//    }
//    else{
//        ClosestAnchorPoint = First_QuadrantAnchorPoint;
//        SmallerDistance = DistancePoint1;
//    }
//
//    if(DistancePoint3 < SmallerDistance)
//    {
//        ClosestAnchorPoint = Third_QuadrantAnchorPoint;
//        SmallerDistance = DistancePoint3;
//    }
//    else{
//        /* Keep the distance and point the same */
//    }
//    if(DistancePoint4 < SmallerDistance)
//    {
//        ClosestAnchorPoint = Fourth_QuadrantAnchorPoint;
//        SmallerDistance = DistancePoint4;
//    }
//    else{
//        /* Keep the distance and point the same */
//    }
    
    return ClosestAnchorPoint;
    
}

/****************************************************************************************************************/
/* Function to add waypoint while coming out of nest */
/****************************************************************************************************************/
void DSA_loop_functions::AddWayPoint(BaseController::RobotData *ptr, BaseController::RobotData *Otherptr)
{
    
    argos::CVector2 Waypoint;
  
    // add waypoint only when the robot is going out from nest
    if(ptr->GoingToNest == false and ptr->GoingToOrFromNest == true)
    {
        // calculate the waypoint if the robot is collinear and going out of nest
        Waypoint = FindClosestAnchorPoint(ptr, Otherptr);
        ptr->AddedPoint = Waypoint;
        
        if(ptr->WaypointStack.empty())
        {
            ptr->WaypointStack.push(ptr->TargetWaypoint);
        }
        
        /* add a way point before the final goal */
        ptr->WaypointStack.push(Waypoint);
        
        ptr->WaypointCounter++;
        
        // check for the intersection if the two robots are going in opposite direction and may interssect
//        if(Otherptr->GoingToNest == true)
//        {
//            IntersectionFlag = Find_Intersection(ptr->StartWaypoint, ptr->TargetWaypoint, Otherptr->StartWaypoint, Otherptr->TargetWaypoint,
//                                                 ptr1, ptr2);
//            if(IntersectionFlag == true)
//            {
//
//            }
//        }
    }
    
}

/************************************************************************************************************************************************/
/* Function to add waypoint while coming out of nest */
/************************************************************************************************************************************************/
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
//            else if(Value == INTERSECTION1 or Value == INTERSECTION2)
//            {
//                intersectionvalue = true;
//            }
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

/************************************************************************************************************************************************/
/* Function to handle intersection if a waypoint is added to te robot going out of nest in case of collinearity */
/************************************************************************************************************************************************/
void DSA_loop_functions::IntersectionHandlingForCollinearity(BaseController::RobotData *RobotDataptr, BaseController::RobotData *BaseRobotDataptr,
                                                             std::vector<BaseController::IntersectionData> *stRobotIntersectionData, argos::UInt8 CollinearRobotID)
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
    
//    if(RobotDataptr->Neighbors.size() > 0)
    if(BaseRobotDataptr->Neighbors.size() > 0)
    {
//        for(NeighborSize = 0; NeighborSize < RobotDataptr->Neighbors.size(); NeighborSize++)
        for(NeighborSize = 0; NeighborSize < BaseRobotDataptr->Neighbors.size(); NeighborSize++)
        
        {
            iterator1 = m_cFootbots.begin();
            // get the handle to particular robot
//            std::advance(iterator1, RobotDataptr->Neighbors[NeighborSize]);
            std::advance(iterator1, BaseRobotDataptr->Neighbors[NeighborSize]);
            CFootBotEntity& cFootBotNeighbor = *any_cast<CFootBotEntity*>(iterator1->second);
            BaseController& cControllerNeighbor = dynamic_cast<BaseController&>(cFootBotNeighbor.GetControllableEntity().GetController());
            
            // Get the robot data
            stRobotDataNeighbor = &cControllerNeighbor.GetRobotData();
            
            // Get Intersection Data
            stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();
            
            if(CollinearRobotID != stRobotDataNeighbor->id_robot)
            {
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
                        /* Add stop time to the robot going towards nest */
                        argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->AddedPoint).Angle();
                        argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
                        TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
                        
                        TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
                        
                        Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->AddedPoint);
                        TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
                        
                        stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
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

                            //                        if(NeighborRobotIntersectionData != NULL)
                            //                        {
                            ////                            stRobotIntersectionData->erase(stRobotIntersectionData->begin() + IntersectionStructIndex);
                            //                        }
                            IntersectionCollisionCheck(RobotDataptr, stRobotDataNeighbor, RobotIntersectionData, NeighborRobotIntersectionData,
                                                       INTERSECTION2, RobotToStop);
                        }
                        
                        /* neighbor robot is collinear to the current robots waypoint */
                        else if(IntersectionFlag == 2)
                        {
                            
                            /* Add stop time to the robot going towards nest */
                            argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->AddedPoint).Angle();
                            argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
                            TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
                            
                            TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
                            
                            Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->AddedPoint);
                            TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
                            
                            stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
                        }
                        
                    }
                    
                    
                    
                }
                /* if the neighbor is going in/ ut of nest*/
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
                        if(stRobotDataNeighbor->GoingToNest == true)
                        {
                            /* Add stop time to the robot going towards nest */
                            argos::CRadians Headingangle = (RobotDataptr->StartWaypoint - RobotDataptr->AddedPoint).Angle();
                            argos::CRadians TurningAngle = (RobotDataptr->Orientation - Headingangle).SignedNormalize();
                            TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), RobotDataptr->fBaseAngularWheelSpeed) + 20;
                            
                            TimeToTurn2 = GetTimeToTurn(90, RobotDataptr->fBaseAngularWheelSpeed) + 20;
                            
                            Distance = CalculateDistance(RobotDataptr->StartWaypoint, RobotDataptr->AddedPoint);
                            TimeToGoStraight = GetTicksToWait(Distance, RobotDataptr->fLinearWheelSpeed) + 20;
                            
                            stRobotDataNeighbor->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
                        }
                        else
                        {
                            /* Do nothing */
                        }
                    }
                    
                }
            
            }// end of if(CollinearRobotID != stRobotDataNeighbor->id_robot)
        }// end of for
    }// end of if(BaseRobotDataptr->Neighbors.size() > 0)
    
    
}
/**********************************************************************************************************************************************************************/
/* Function to add waypoint while coming out of nest */
/**********************************************************************************************************************************************************************/
void DSA_loop_functions::AvoidCollinearCollision(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, BaseController::RobotData *BaseRobotptr,
                                                 std::vector<BaseController::IntersectionData> *ptr3, std::vector<BaseController::IntersectionData> *ptr4)
{
    argos::CVector2 WaypointCalculated1, WaypointCalculated2;
    argos::Real dist1, dist2, Distance;
    argos::UInt8 TimeToTurn1, TimeToTurn2, TimeToGoStraight, StopTime;
    
    // both are collinear and going out of nest
    if((ptr1->GoingToNest == false and ptr1->GoingToOrFromNest == true) and (ptr2->GoingToNest == false and ptr2->GoingToOrFromNest == true))
    {
        WaypointCalculated1 = FindClosestAnchorPoint(ptr1, ptr2);
        WaypointCalculated2 = FindClosestAnchorPoint(ptr2, ptr1);
        
        ptr1->AddedPoint = WaypointCalculated1;
        ptr2->AddedPoint = WaypointCalculated2;
        
//        if(ptr1->AddedPoint == ptr2->AddedPoint)
//        {
//            dist1 = CalculateDistance(ptr1->StartWaypoint, ptr1->AddedPoint);
//            dist2 = CalculateDistance(ptr2->StartWaypoint, ptr2->AddedPoint);
//            if(dist1 < dist2)
//            {
//                /* Add stop time to the robot going towards nest */
//                argos::CRadians Headingangle = (ptr1->StartWaypoint - ptr1->AddedPoint).Angle();
//                argos::CRadians TurningAngle = (ptr1->Orientation - Headingangle).SignedNormalize();
//                TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), ptr1->fBaseAngularWheelSpeed) + 20;
//
//                TimeToTurn2 = GetTimeToTurn(90, ptr1->fBaseAngularWheelSpeed) + 20;
//                TimeToGoStraight = GetTicksToWait(Distance, ptr1->fLinearWheelSpeed) + 20;
//                ptr2->StopTurningTime = TimeToTurn1 + TimeToTurn2 + TimeToGoStraight + 20;
//            }
//            else
//            {
//                /* Add stop time to the robot going towards nest */
//                argos::CRadians Headingangle = (ptr2->StartWaypoint - ptr2->AddedPoint).Angle();
//                argos::CRadians TurningAngle = (ptr2->Orientation - Headingangle).SignedNormalize();
//                TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), ptr2->fBaseAngularWheelSpeed) + 20;
//
//                TimeToTurn2 = GetTimeToTurn(90, ptr2->fBaseAngularWheelSpeed) + 20;
//                TimeToGoStraight = GetTicksToWait(Distance, ptr2->fLinearWheelSpeed) + 20;
//                ptr1->StopTurningTime = TimeToTurn1 + TimeToTurn2 + TimeToGoStraight + 20;
//            }
//        }
        
        IntersectionHandlingForCollinearity(ptr1, BaseRobotptr, ptr3, ptr2->id_robot);

        IntersectionHandlingForCollinearity(ptr2, BaseRobotptr, ptr4, ptr1->id_robot);
       
    }
//    // Robot1 is going to nest and robot2 is going out of nest
    else if((ptr1->GoingToNest == true and ptr1->GoingToOrFromNest == false) and (ptr2->GoingToNest == false and ptr2->GoingToOrFromNest == true))
    {
        WaypointCalculated2 = FindClosestAnchorPoint(ptr2, ptr1);
        ptr2->AddedPoint = WaypointCalculated2;
        
        /* Add stop time to the robot going towards nest */
        argos::CRadians Headingangle = (ptr2->StartWaypoint - ptr2->AddedPoint).Angle();
        argos::CRadians TurningAngle = (ptr2->Orientation - Headingangle).SignedNormalize();
        TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), ptr2->fBaseAngularWheelSpeed) + 20;
        
        TimeToTurn2 = GetTimeToTurn(90, ptr2->fBaseAngularWheelSpeed) + 20;
        
        Distance = CalculateDistance(ptr2->StartWaypoint, ptr2->AddedPoint);
        TimeToGoStraight = GetTicksToWait(Distance, ptr2->fLinearWheelSpeed) + 20;
        
        ptr1->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
        
        IntersectionHandlingForCollinearity(ptr2, BaseRobotptr, ptr4, ptr1->id_robot);

    }
    //Robot1 is going out of nest and robot2 is going to the nest
    else if((ptr1->GoingToNest == false and ptr1->GoingToOrFromNest == true) and (ptr2->GoingToNest == true and ptr2->GoingToOrFromNest == false))
    {
        WaypointCalculated1 = FindClosestAnchorPoint(ptr1, ptr2);
        ptr1->AddedPoint = WaypointCalculated1;
        
        /* Add stop time to the robot going towards nest */
        argos::CRadians Headingangle = (ptr1->StartWaypoint - ptr1->AddedPoint).Angle();
        argos::CRadians TurningAngle = (ptr1->Orientation - Headingangle).SignedNormalize();
        TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), ptr1->fBaseAngularWheelSpeed) + 20;
        
        TimeToTurn2 = GetTimeToTurn(90, ptr1->fBaseAngularWheelSpeed) + 20;
        
        Distance = CalculateDistance(ptr1->StartWaypoint, ptr1->AddedPoint);
        TimeToGoStraight = GetTicksToWait(Distance, ptr1->fLinearWheelSpeed) + 20;
        
        ptr2->StopTurningTime = TimeToGoStraight + TimeToTurn1 + TimeToTurn2 + 20;
        
//        ptr2->StopTurningTime = 20;
        IntersectionHandlingForCollinearity(ptr1, BaseRobotptr, ptr3, ptr2->id_robot);
    }
    // both the robots are going to the nest
    else{
      /* Do nothing */
    }
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
//        if((RobotIntersectionData->Robot_ID_Intersectingwith == RobotId) and (RobotIntersectionData->TargetPoint == TargetPt))
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
    
    //    BaseController::IntersectionData *stIntersectionDataThis = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataThis = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighbor = NULL;
    std::vector<BaseController::IntersectionData>* stIntersectionDataNeighborNext = NULL;
    
    BaseController::IntersectionData* Robot1IntersectionData = NULL;
    BaseController::IntersectionData* Robot2IntersectionData = NULL;
    
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
//        stIntersectionDataThis = &cControllerThis.GetIntersectionData();
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
//                    IntersectionLoopValue = Matrix_Value;
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
//                        stIntersectionDataNeighbor = &cControllerNeighbor.GetIntersectionData();
                        stIntersectionDataNeighbor = cControllerNeighbor.GetIntersectionData();

                        iterator3 = m_cFootbots.begin();
                        // get the handle to neighbor robot indicated by column
                        std::advance(iterator3, robotnextneighborindex);
                        CFootBotEntity& cFootBotNextNeighbor = *any_cast<CFootBotEntity*>(iterator3->second);
                        BaseController& cControllerNextNeighbor = dynamic_cast<BaseController&>(cFootBotNextNeighbor.GetControllableEntity().GetController());

                        // Get the robot data
                        stRobotDataNeighborNext = &cControllerNextNeighbor.GetRobotData();
//                        stIntersectionDataNeighborNext = &cControllerNextNeighbor.GetIntersectionData();
                        stIntersectionDataNeighborNext = cControllerNextNeighbor.GetIntersectionData();

                        // implement collinear collision avoidance
                       if(Matrix_Value == COLLINEAR)
                       {
                           //Collinearity handling
                           AvoidCollinearCollision(stRobotDataNeighbor, stRobotDataNeighborNext, stRobotDataThis, stIntersectionDataNeighbor,
                                                   stIntersectionDataNeighborNext);

                           InitializeMatrixElementAndTransformElement(stRobotDataThis,row_index,
                                                                      column_index, CONSISTENT);

                       }
                        // implement intersection collision avoidance
                       else if(Matrix_Value == INTERSECTION1 or Matrix_Value == INTERSECTION2)
                       {

                           if(stRobotDataNeighbor->GoingToOrFromNest == false and stRobotDataNeighborNext->GoingToOrFromNest == true)
                           {
                               TestVariable = 1;
                               TestValue = stRobotDataNeighbor->id_robot;
                              InersectionDataRobot1 = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                    stRobotDataNeighborNext->id_robot, Matrix_Value);
//                              TestPoint = InersectionDataRobot1->IntersectionPoint;

                              InersectionDataRobot2 = GetIntersectionDataFromVector(stIntersectionDataNeighborNext,
                                                                                     stRobotDataNeighbor->id_robot, Matrix_Value);
                               
                              IntersectionCollisionCheck(stRobotDataNeighbor, stRobotDataNeighborNext, InersectionDataRobot1, InersectionDataRobot2,
                                                          Matrix_Value, 2);


                           }
                           else if(stRobotDataNeighborNext->GoingToOrFromNest == false and stRobotDataNeighbor->GoingToOrFromNest == true)
                           {

                               TestVariable = 2;
                               TestValue = stRobotDataNeighbor->id_robot;
                               InersectionDataRobot1 = GetIntersectionDataFromVector(stIntersectionDataNeighbor,
                                                                                     stRobotDataNeighborNext->id_robot, Matrix_Value);


                               InersectionDataRobot2 = GetIntersectionDataFromVector(stIntersectionDataNeighborNext,
                                                                                     stRobotDataNeighbor->id_robot, Matrix_Value);
                               
                               IntersectionCollisionCheck(stRobotDataNeighbor, stRobotDataNeighborNext, InersectionDataRobot1, InersectionDataRobot2,
                                                          Matrix_Value, 1);

                           }
                           else
                           {
                               TestVariable = 3;
                               TestValue = stRobotDataNeighbor->id_robot;


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

/******************************************************************************************************************************************/
/* Function to avoid intersection of robots */
/******************************************************************************************************************************************/
void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                    BaseController::IntersectionData* Robot1IntersectionData, BaseController::IntersectionData* Robot2IntersectionData,
                                                    argos::UInt8 IntersectionValue, argos::UInt8 index)
{
    
    argos::UInt16 TimeToIntersection1, TimeToIntersection2, TicksToWaitRobot1, TicksToWaitRobot2, TicksToWaitforSafedistance,
                  TimeToTurn1, TimeToTurn2, TimeFromStartPoint1, TimeFromStartPoint2, TimeToIntersectionRobot1, TimeToIntersectionRobot2,
                  TimeDiff, TimeToIntersection, i, j;
    argos::Real DistanceToIntersection1, DistanceToIntersection2, IntersectionDistance, AdjustedVelocity;
    
//    BaseController::IntersectionData* Robot1IntersectionData;
//    BaseController::IntersectionData* Robot2IntersectionData;
    
    // get the intersection data for robot1
//    for(i=0; i < ptr3->size(); i++)
//    {
//        Robot1IntersectionData = &(ptr3->at(i));
//        if(Robot1IntersectionData->Robot_ID_Intersectingwith == ptr2->id_robot)
//        {
//            break;
//        }
//
//    }
//
//    // get the intersection data for robot2
//    for(j=0; j < ptr4->size(); j++)
//    {
//        Robot2IntersectionData = &ptr3->at(i);
//        if(Robot2IntersectionData->Robot_ID_Intersectingwith == ptr1->id_robot)
//        {
//            break;
//        }
//
//    }
    
    if(Robot1IntersectionData != NULL and Robot2IntersectionData != NULL)
    {
        if(IntersectionValue == INTERSECTION1)
        {
            // caluclate for Robot 1
            
            // calculate the time required by robot 1 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle1 = (Robot1IntersectionData->IntersectionPoint - Robot1IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle1 = (ptr1->Orientation - Headingangle1).SignedNormalize();
            TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle1).GetValue(), ptr1->fBaseAngularWheelSpeed) + 50;
    //        TimeToTurn1 = 0;
            DistanceToIntersection1 = CalculateDistance(Robot1IntersectionData->StartPoint, Robot1IntersectionData->IntersectionPoint);
            TimeFromStartPoint1 = GetTicksToWait(DistanceToIntersection1, ptr1->fLinearWheelSpeed);
            TimeToIntersectionRobot1 = TimeToTurn1 + TimeFromStartPoint1 + ptr1->StopTurningTime + 50;

            // caluclate for Robot 2
    //        TimeToTurn2 = 0;
            argos::CRadians Headingangle2 = (Robot2IntersectionData->IntersectionPoint - Robot2IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle2 = (ptr2->Orientation - Headingangle2).SignedNormalize();
            TimeToTurn2 = GetTimeToTurn(ToDegrees(TurningAngle2).GetValue(), ptr2->fBaseAngularWheelSpeed) + 50;
            DistanceToIntersection2 = CalculateDistance(Robot2IntersectionData->StartPoint,Robot2IntersectionData->IntersectionPoint);
            TimeFromStartPoint2 = GetTicksToWait(DistanceToIntersection2, ptr2->fLinearWheelSpeed);
            TimeToIntersectionRobot2 = TimeToTurn2 + TimeFromStartPoint2 + ptr2->StopTurningTime + 50;

            TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + 20);
        }
        else
        {
            // calculate the time required by robot 1 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle1 = (Robot1IntersectionData->IntersectionPoint - Robot1IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle1 = (ptr1->Orientation - Headingangle1).SignedNormalize();
            TimeToTurn1 = GetTimeToTurn(ToDegrees(TurningAngle1).GetValue(), ptr1->fBaseAngularWheelSpeed) + 20;
            DistanceToIntersection1 = CalculateDistance(Robot1IntersectionData->StartPoint, Robot1IntersectionData->IntersectionPoint) +
                                      CalculateDistance(ptr1->StartWaypoint, Robot1IntersectionData->StartPoint);
            TimeFromStartPoint1 = GetTicksToWait(DistanceToIntersection1, ptr1->fLinearWheelSpeed);
            TimeToIntersectionRobot1 = TimeFromStartPoint1 + TimeToTurn1 + ptr1->StopTurningTime;


            // calculate the time required by robot 2 if the intersection value is INTERSECTION2
            argos::CRadians Headingangle2 = (Robot2IntersectionData->IntersectionPoint - Robot2IntersectionData->StartPoint).Angle();
            argos::CRadians TurningAngle2 = (ptr2->Orientation - Headingangle2).SignedNormalize();
            TimeToTurn2 = GetTimeToTurn(ToDegrees(TurningAngle2).GetValue(), ptr2->fBaseAngularWheelSpeed) + 20;
            DistanceToIntersection2 = CalculateDistance(Robot2IntersectionData->StartPoint, Robot2IntersectionData->IntersectionPoint) +
                                      CalculateDistance(ptr2->StartWaypoint, Robot2IntersectionData->StartPoint);
            TimeFromStartPoint2 = GetTicksToWait(DistanceToIntersection2, ptr2->fLinearWheelSpeed);
            TimeToIntersectionRobot2 = TimeFromStartPoint2 + TimeToTurn2 + ptr2->StopTurningTime;

            TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + (GetTimeToTurn(180, ptr1->fBaseAngularWheelSpeed)) + 20);
        }

        TimeDiff = abs(TimeToIntersectionRobot1 - TimeToIntersectionRobot2);

        if(TimeDiff <= TicksToWaitforSafedistance)
        {
            // add stop time to robot1
            if(index == 1)
            {
                IntersectionDistance = DistanceToIntersection2;
                TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot2;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MaxLinearSpeed;
                    ptr1->StopTurningTime += ((TimeToIntersection + 20));

                }

                ptr1->fLinearWheelSpeed = AdjustedVelocity;
            }
            // add stop time to robot2
            else if(index == 2)
            {
                IntersectionDistance = DistanceToIntersection1;
                TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot1;
                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                if(AdjustedVelocity < MinLinearSpeed)
                {
                    AdjustedVelocity = MaxLinearSpeed;
                    ptr2->StopTurningTime += ((TimeToIntersection + 20));

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

                    if(AdjustedVelocity < MinLinearSpeed)
                    {
                        AdjustedVelocity = MaxLinearSpeed;
                        ptr2->StopTurningTime += ((TimeToIntersection + 20));

                    }

                    ptr2->fLinearWheelSpeed = AdjustedVelocity;
                }
                // time required for robot2 is less than that for robot1, add stop time to robot1
                else
                {
                    IntersectionDistance = DistanceToIntersection2;
                    TimeToIntersection = TicksToWaitforSafedistance + TimeToIntersectionRobot2;
                    AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;

                    if(AdjustedVelocity < MinLinearSpeed)
                    {
                        AdjustedVelocity = MaxLinearSpeed;
                        ptr1->StopTurningTime += ((TimeToIntersection + 20));

                    }

                    ptr1->fLinearWheelSpeed = AdjustedVelocity;
                } // end of else

            }// end of else: add stop time to either of robot by evaluating
            
        }// end of if(TimeDiff < TimeToWaitForSafeDistance)
    }
    




//    ptr3->Intersection_flag = 0;
    
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

///****************************************************************************************************************/
///* Function to reset all robot data */
///****************************************************************************************************************/
//void DSA_loop_functions::RobotIntersectionWithMultipleRobots()
//{
//    BaseController::RobotData *stRobotDataThis = NULL;
//    BaseController::RobotData *stRobotDataOther = NULL;
//    argos::UInt16 i, rows, columns, column_count;
//    bool IsMatrixZero;
//
//    argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
//    CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
//
//    /* get handle to the robot with resource */
//    for(i = 0; i < RobotResource.size(); i++)
//    {
//        // get the handle to particular robot
//        std::advance(iterator1, RobotResource[i]);
//        CFootBotEntity& cFootBotThis = *any_cast<CFootBotEntity*>(iterator1->second);
//        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());
//        stRobotDataThis = &cControllerThis.GetRobotData();
//
//        IsMatrixZero = IsMatrixConsistent(stRobotDataThis);
//
//        // If matrix is non-zero, check the intersection
//        if(IsMatrixZero == true)
//        {
//
//            // get number of rows
//            rows = stRobotDataThis->NeighborsMatrix.size();
//            // get number of columns
//            if(rows > 0)
//            {
//                columns = stRobotDataThis->NeighborsMatrix[0].size();
//            }
//
//            // Case 1: The robot is intersecting with multiple robots
//            for(column_count = 1; column_count < columns; column_count++)
//            {
//                if((stRobotDataThis->NeighborsMatrix[0][column_count] == INTERSECTION1) or
//                   (stRobotDataThis->NeighborsMatrix[0][column_count] == INTERSECTION2)
//                {
//                    Inet
//                }
//
//            }
//               }
//}


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
    argos::CVector2 vec1, vec2;
    
    argos::UInt16 timetowait;
    
    argos::UInt16 CollinearityStopTime = 10;
    
    vec1.Set((ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX()), (ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()));
    vec2.Set((ptr2->TargetWaypoint.GetX() - ptr2->StartWaypoint.GetX()), (ptr2->TargetWaypoint.GetY() - ptr2->StartWaypoint.GetY()));
   
    vec1length = vec1.Length();
    vec2length = vec2.Length();
    
//    ptr1.vect1 = vec1;
//    ptr2.vect2 = vec2;
    
    dot_product = vec1.DotProduct(vec2);
    
    CosTheta = (dot_product/(vec1length * vec2length));
    
    // get the shortest distance between two vectors
    shortest_dist = ShortestDistTwoVectors(ptr1, ptr2);
    
//    ptr1->Priority = CosTheta;
//    ptr2->Priority = CosTheta;
    // angle = 0 to 21 deg celsius or 180 to 158
//    if(abs(CosTheta) <= 1 and abs(CosTheta) >= 0.93)
    
    //angle ->180 to 152.87 and 0 to 27.12
    if(abs(CosTheta) <= 1 and abs(CosTheta) >= 0.89)
    {
        if(ptr1->GoingToOrFromNest == true and ptr2->GoingToOrFromNest == true)
        {
            if((shortest_dist <= Safedistance))
            {
                ptr1->CollinearFlag = true;
                ptr2->CollinearFlag = true;
                
                // add stop time to the robot going to nest if the robots are in opposite direction
//                if(ptr1->GoingToNest == true and ptr2->GoingToNest == false)
//                {
//                    ptr1->StopTurningTime = CollinearityStopTime;
//                }
//                else if(ptr2->GoingToNest == true and ptr1->GoingToNest == false)
//                {
//                    ptr2->StopTurningTime = CollinearityStopTime;
//                }
//                else if(ptr1->GoingToNest == false and ptr2->GoingToNest == false and ptr1->GoingToOrFromNest == true and ptr2->GoingToOrFromNest == true)
//                {
//                    if(CalculateDistance(ptr1->StartWaypoint, ptr1->TargetWaypoint) < CalculateDistance(ptr2->StartWaypoint, ptr2->TargetWaypoint))
//                    {
//                        ptr2->StopTurningTime = CollinearityStopTime;
//                    }
//                    else{
//                        ptr1->StopTurningTime = CollinearityStopTime;
//                    }
//                }
//                if(vec1length > 0.9 or vec2length > 0.9)
//                {
//                    AddNewWayPoint(ptr1, ptr2);
//                }
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
/****************/
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
//void DSA_loop_functions::CheckRobotHeadingCourse(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                                 BaseController::IntersectionData &ptr3)
//{
//
//    argos::Real RobotCourseAngle;
//    argos::CVector2 WaypointAdd, PredictedTargetPt1, PredictedTargetPt2;
//    argos:: Real shortest_dist;
//    argos:: Real distance[4] = {0, 0, 0, 0};
//    argos:: Real min;
//    argos::UInt8 i;
//    min = 100;
//    i = 0;
//
//    /* find the angle between lines */
//    RobotCourseAngle = CalculateAngleBetweenRobotCourse(ptr1, ptr2);
//    ptr1.InitialOrientation = RobotCourseAngle;
//    ptr2.InitialOrientation = RobotCourseAngle;
//
//    /* In 2d the shortest of the distance between point A and line segment CD, B and CD, C and AB or D and AB.
//     So it's a fairly simple "distance between point and line" calculation (if the distances are all the same,
//     then the lines are parallel. */
//
//    // dist between robot 1 start, goal and robot2 start
////    distance[0] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.StartWaypoint);
//    distance[0] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, ptr2.StartWaypoint);
//    // dist between robot 1 start, goal and robot2 goal
////    distance[1] = DistancePointSegment(ptr1.StartWaypoint, ptr1.TargetWaypoint, ptr2.TargetWaypoint);
//    distance[1] = DistancePointSegment(ptr1.StartWaypoint, TargetPoint1, TargetPoint2);
//
//    // dist between robot 2 start, goal and robot1 start
////    distance[2] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.StartWaypoint);
//    distance[2] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, ptr1.StartWaypoint);
//
//    // dist between robot 2 start, goal and robot2 goal
////    distance[3] = DistancePointSegment(ptr2.StartWaypoint, ptr2.TargetWaypoint, ptr1.TargetWaypoint);
//    distance[3] = DistancePointSegment(ptr2.StartWaypoint, TargetPoint2, TargetPoint1);
//
//
//    for(i = 0; i< 4; i++)
//    {
//        if(distance[i] < min)
//        {
//            min = distance[i];
//        }
//        shortest_dist = min;
//    }
//
//    if(ptr1.id_robot == 0 and ptr2.id_robot == 2)
//    {
//        ptr1.Priority = shortest_dist;
//        ptr2.Priority = shortest_dist;
//    }
////    argos:: Real dist1 = (ptr1.StartWaypoint - NestPosition).SquareLength();
//    argos:: Real dist1 = (ptr1.StartWaypoint - TargetPoint1).SquareLength();
//    //        argos:: Real dist2 = (ptr1.TargetWaypoint - NestPosition).SquareLength();
////    argos:: Real dist3 = (ptr2.StartWaypoint - NestPosition).SquareLength();
//    argos:: Real dist3 = (ptr2.StartWaypoint - TargetPoint2).SquareLength();
//    //        argos:: Real dist4 = (ptr2.TargetWaypoint - NestPosition).SquareLength();
//
//    argos:: Real distance_fromnest1 = CalculateDistance(ptr1.StartWaypoint, NestPosition);
//    argos:: Real distance_fromnest2 = CalculateDistance(ptr2.StartWaypoint, NestPosition);
//    // if the lines are almost parallel
//    if(0 <= RobotCourseAngle and RobotCourseAngle <= OverlappingCourseAngle)
//    {
//        // if lines are very close to each other
//        if(shortest_dist <= Safedistance)
//        {
//
//            if((ptr1.TargetWaypoint == NestPosition and ptr2.TargetWaypoint == NestPosition) or
//               (ptr1.TargetWaypoint == NestPosition and distance_fromnest2 <= (NestRadiusSquared+0.02) and ptr2.GoingToOrFromNest == true) or
//               (ptr2.TargetWaypoint == NestPosition and distance_fromnest1 <= (NestRadiusSquared+0.02) and ptr1.GoingToOrFromNest == true))
////            if((ptr1.TargetWaypoint == NestPosition and ptr2.TargetWaypoint == NestPosition) or
////               (ptr1.TargetWaypoint == NestPosition and ptr2.GoingToOrFromNest == true) or
////               (ptr2.TargetWaypoint == NestPosition and ptr1.GoingToOrFromNest == true))
//
//            {
////                    ptr1.CollinearFlag = 1;
//                    AddNewWayPoint(ptr1, ptr2);
//
//            }
//            // if 2nd robot is in spiral inside 1st robot
//            else if(ptr2.TargetWaypoint != NestPosition and ptr2.StartWaypoint.GetX() > ptr1.StartWaypoint.GetX())
//            {
//                ptr1.StopTurningTime += 10;
//            }
//
//
//        }
//    }
//
//
//}

/*****************************************************************************************************************/
/* Function to find the angle between two lines */
/*****************************************************************************************************************/
//Real DSA_loop_functions::CalculateAngleBetweenRobotCourse(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2)
//{
//
////    argos::CDegrees headingToTargetR1 = ToDegrees((ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle());
////
////    ptr1.HeadingAngle = headingToTargetR1;
////    argos::CDegrees headingToTargetR2 = ToDegrees((ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle());
////
////    ptr2.HeadingAngle = headingToTargetR2;
//    /* get the current heading angle of the robot */
////    argos::Real AngleRobotCourse = abs(180 - (abs(headingToTargetR1.GetValue()) + abs(headingToTargetR2.GetValue())));
////    argos::Real AngleRobotCourse = abs((abs(headingToTargetR1.GetValue()) - abs(headingToTargetR2.GetValue())));
//
//    argos::Real angle;
//    CVector2 v1, v2;
//    Real v1_length, v2_length;
//
////    acos((v1 DOT v2)/(|v1|*|v2|))
//
//    v1 = (ptr1.TargetWaypoint - ptr1.StartWaypoint);
//    v2 = (ptr2.TargetWaypoint - ptr2.StartWaypoint);
//    v1_length = v1.Length();
//    v2_length = v2.Length();
//    angle = acos((v1.DotProduct(v2))/(v1_length * v2_length));
//
//
////    argos::CRadians headingToTargetR1 = (ptr1.TargetWaypoint - ptr1.StartWaypoint).Angle();
////    argos::CRadians headingToTargetR2 = (ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle();
////
////
////
////    CDegrees angle1 = ToDegrees(headingToTargetR1);
////    CDegrees angle2 = ToDegrees(headingToTargetR2);
////
////    ptr1.HeadingAngle = ToDegrees(headingToTargetR1);
////
////    ptr2.HeadingAngle = ToDegrees(headingToTargetR2);
////
////    CRadians res_angle_radians = (headingToTargetR1 - headingToTargetR2).SignedNormalize();
////
////    argos::Real AngleRobotCourse = ToDegrees(res_angle_radians).GetValue();
//
//    argos::CRadians angle_readians = CRadians(angle);
//    argos::Real AngleRobotCourse = ToDegrees(angle_readians).GetValue();
//
//    AngleRobotCourse  = abs(AngleRobotCourse);
//
//    return AngleRobotCourse;
//
//}


/*****************************************************************************************************************/
/* Function to find the target point on the circle */
/*****************************************************************************************************************/
//argos::CVector2 DSA_loop_functions::CalculateTargePoint(BaseController::RobotData& ptr)
//{
//    // Center of nest circle is (0,0)
//    argos::Real x, y;
//    argos::CVector2 PointOnCircle;
//    argos::Real theta = atan2((ptr.StartWaypoint.GetY() - 0),(ptr.StartWaypoint.GetX()-0));
//    x = 0 + NestRadius * cos(theta);
//    y = 0 + NestRadius * sin(theta);
//    PointOnCircle.Set(x, y);
//
//    return PointOnCircle;
//}

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
    
    x=0;
    y=0;
    i=1;
    PointChangeDirection.Set(0,0);
    PointAtSafeDistance.Set(0,0);
    
    x = ptr->TargetWaypoint.GetX();
    y = ptr->TargetWaypoint.GetY();

    
    PointChangeDirection.Set(x, y);
    
    
    if(ptr->pattern.size() > 0)
    {
        length = ptr->pattern.size();
        previous_direction = ptr->pattern[length - 1];
        
        for(i=1;i<5;i++)
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

/*****************************************************************************************************************/
/* Function to add new way point */
/*****************************************************************************************************************/
void DSA_loop_functions::AddNewWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
    argos::CVector2 AddedWaypoint;
    argos::UInt16 TicksToWait_Robot1;
    argos::Real dist, x, y, radius, t1, t2;
    argos::Real  DistanceToTarget_Robot1, DistRobot2NeedToTravel, dir,angle_value;
    argos::CVector3 a, b, perp, up;
    argos::CRadians headingAngleTowardsTarget;
    argos::Real dist1, dist2;
    argos::CRadians theta;
    argos::CVector2 robot2_vector;
    argos::CRadians angle;
    argos::Real AngleValue1,AngleValue;
    argos::CVector3 Up, Forward, VectorA, VectorB, GlobalUp, Perpendicular;
    argos::Real Direction;
    argos::Real AngleDifferenceSet = 60.0f;
    
//
//     // add some stop time to allow robot 2 turn and move to waypoint
////    ptr1.StopTurningTime += 10;
//
////    if(ptr1.StartWaypoint.GetX() < ptr2.StartWaypoint.GetX())
////    if((ptr1.StartWaypoint.GetX() > ptr2.StartWaypoint.GetX()) and
////       (ptr1.StartWaypoint.GetY() > ptr2.StartWaypoint.GetY()))
////    {
////        theta = ToRadians(CDegrees(45.0f)).GetValue();
////
////    }
////    else
////    {
////        theta = ToRadians(CDegrees(135.0f)).GetValue();
////    }
//
////    dist = CalculateDistance(ptr1.StartWaypoint, ptr2.TargetWaypoint);
//
////    if(ptr2.TargetWaypoint == NestPosition)
////    {
////        headingAngleTowardsTarget = (ptr2.TargetWaypoint - ptr2.StartWaypoint).Angle();
////    }
////    else
////    {
////        headingAngleTowardsTarget = ToRadians(CDegrees(0.0f));
////    }
//
////  // vector a = ptr2.TargetPoint - ptr1.Startwaypoint
////    a.SetX((ptr2.TargetWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
////    a.SetY((ptr2.TargetWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
////    a.SetZ(0);
////    // vector b = ptr1.StartWaypoint - ptr2.Startwaypoint
////    b.SetX((ptr1.StartWaypoint.GetX() - ptr2.StartWaypoint.GetX()));
////    b.SetY((ptr1.StartWaypoint.GetY() - ptr2.StartWaypoint.GetY()));
////    b.SetZ(0);
//
//      // vector a = ptr2.TargetPoint - ptr2.Startwaypoint
////        a.SetX((ptr2->TargetWaypoint.GetX() - ptr2->StartWaypoint.GetX()));
////        a.SetY((ptr2->TargetWaypoint.GetY() - ptr2->StartWaypoint.GetY()));
////        a.SetZ(0);
////        // vector b = ptr1.TargetWaypoint - ptr1.Startwaypoint
////        b.SetX((ptr1->TargetWaypoint.GetX() - ptr1->StartWaypoint.GetX()));
////        b.SetY((ptr1->TargetWaypoint.GetY() - ptr1->StartWaypoint.GetY()));
////        b.SetZ(0);
////
////    // vector c = (b x a)
////    up.Set(0,1,0);
////    perp = b.CrossProduct(a);
////    dir = perp.DotProduct(up);
//
////    angle = ((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle()).UnsignedNormalize();
//
//
//
////    check if angle is negative
////    if(AngleValue < 0)
////    {
////
////
////        // turn the robot to left
////        if(abs(AngleValue) < 90)
////        {
////            theta = ToRadians(CDegrees(360.0f + AngleValue + AngleDifferenceSet));
////
////        }
////        // turn to right
////        else
////        {
////            theta = ToRadians(CDegrees(360.0f + AngleValue - AngleDifferenceSet));
////
////        }
////    }
////    // angle is positive
////    else
////    {
////
////
////        // turn
////        if(abs(AngleValue) < 90)
////        {
////            theta = ToRadians(CDegrees(abs(AngleValue) - AngleDifferenceSet));
////
////        }
////        else
////        {
////            theta = ToRadians(CDegrees(abs(AngleValue) + AngleDifferenceSet));
////
////        }
////
////    }
//
//
//
//    // check if angle is negative -> -180 to 0 range
////    if(AngleHeading < 0)
////    {
//////        RotationValue = (90.0f + AngleHeading) - 45.0f;
////        if(abs(AngleHeading) > 90)
////        {
////            RotationValue = -45.0f;
////        }
////        else
////        {
////            RotationValue = -135.0f;
////        }
////
////    }
////    //angle is positive -> 0 to 180 range
////    else
////    {
////        if(AngleHeading > 90)
////        {
////            RotationValue = 45.0f;
////        }
////        else
////        {
////            RotationValue = 135.0f;
////        }
////    }
////
////    theta = ToRadians(CDegrees(AngleHeading - RotationValue));
//
////    if((ToDegrees((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle()).GetValue()) < 0)
////    {
////        angle_value = -1.0f;
////    }
////    else{
////        angle_value = 1.0f;
////    }
////
////    argos:: Real angle = (ToDegrees((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle()).GetValue());
////    argos:: Real angle_counterclockwise = 360 - abs(angle);
//
////    theta = ToRadians(CDegrees(-180.0f));
//    //target is to the left side
////    if(dir == -1)
////    {
////
//////        theta = ToRadians(CDegrees(angle_value * 135.0f));
////        theta = ToRadians(CDegrees(angle_counterclockwise - 135.0f));
////    }
////    //target is to the right side
////    else if(dir == 1)
////    {
//////        theta = ToRadians(CDegrees(angle_value * 45.0f));
////        theta = ToRadians(CDegrees(angle_counterclockwise - 45.0f));
////    }
////    //front or behind
////    else
////    {
////        argos::Real angle_Robot1 = ToDegrees((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle()).GetValue();
//
////        theta = ToRadians(CDegrees(angle_value * angle_Robot1));
//
////        if(abs(angle) > 90)
////        {
//////            theta = ToRadians(CDegrees(-angle_value * (45.0f)));
////            argos::Real angle_45 = (360 - (abs(angle_counterclockwise) - 45.0f));
////            theta = ToRadians(CDegrees(angle_45));
////        }
////        else
////        {
////            argos::Real angle_135 = ((abs(angle_counterclockwise) + 45.0f) - 360);
////            theta = ToRadians(CDegrees(angle_135));
////
////
////        }
////        theta = ToRadians(CDegrees(angle_value * ToDegrees((ptr1->TargetWaypoint - ptr1->StartWaypoint).Angle())));
////    }
//
//
//    //if z coordinate is +ve, then ptr1 is to the left of ptr2
////    if(c.GetZ() > 0)
////    {
////        theta = ToRadians(CDegrees(135.0f));
////    }
////    else
////    {
////        theta = ToRadians(CDegrees(45.0f));
////    }
//
//
//
////    radius = (dist * sqrt(2))/2;
////
////    x = ptr2.StartWaypoint.GetX() + cos(theta) * radius;
////    y = ptr2.StartWaypoint.GetY() + sin(theta) * radius;
//
////    AddedWaypoint.Set(x,y);
////    argos::CVector3 robot2vector3d = GetVectorFromTwoPoints(ptr2.StartWaypoint, TargetPoint2);
////    robot2_vector.Set(robot2vector3d.GetX(), robot2vector3d.GetY());
////    robot2_vector.Set(a.GetX(), a.GetY());
//
//
//
//    // if robot is closer to nest
////    if(dist1 < 0.3)
////    {
////        ptr2.StopTurningTime += 2*((GetTicksToWait(dist1 , ptr1.fLinearWheelSpeed) + GetTimeToTurn(180, ptr1.fBaseAngularWheelSpeed)));
////    }
////    if(dist2 < 0.5)
////    {
////        AddedWaypoint = (robot2_vector.Rotate(theta));
////
////    }
////     if robot is farther from nest
////    else
////    {
////        AddedWaypoint = 0.5*(robot2_vector.Rotate(theta));
////    }
//
//
//
//    ptr1->HeadingAngle = ToDegrees((ptr1->TargetWaypoint - ptr1->StartWaypoint).Angle());
//    ptr2->HeadingAngle = ToDegrees((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle());
//
//    AngleValue = (ToDegrees((ptr2->TargetWaypoint - ptr2->StartWaypoint).Angle()).GetValue());
//    AngleValue1 = (ToDegrees((ptr1->TargetWaypoint - ptr1->StartWaypoint).Angle()).GetValue());
//
//
//    if(abs(AngleValue) > 90)
//    {
//        if(abs(AngleValue1) > abs(AngleValue))
//        {
//            theta = ToRadians(CDegrees(AngleValue + AngleDifferenceSet));
//        }
//        else
//        {
//            theta = ToRadians(CDegrees(AngleValue - AngleDifferenceSet));
//        }
//    }
//    else
//    {
//        if(abs(AngleValue1) > abs(AngleValue))
//        {
//            theta = ToRadians(CDegrees(AngleValue - AngleDifferenceSet));
//        }
//        else
//        {
//            theta = ToRadians(CDegrees(AngleValue + AngleDifferenceSet));
//        }
//
//    }
//
//    dist1 = CalculateDistance(ptr1->StartWaypoint, ptr1->TargetWaypoint);
//    dist2 = CalculateDistance(ptr2->StartWaypoint, ptr2->TargetWaypoint);
//
//
//
//    if(dist2 < 1)
//    {
//        radius = ((ptr2->TargetWaypoint - ptr2->StartWaypoint).Length());
//    }
//    else
//    {
//        radius = ((ptr2->TargetWaypoint - ptr2->StartWaypoint).Length()) * 0.3;
//
//    }
//
//
//    x = ptr2->StartWaypoint.GetX() + (radius * Cos(theta));
//    y = ptr2->StartWaypoint.GetY() + (radius * Sin(theta));
//
//    AddedWaypoint.Set(x,y);
//
//
//    ptr1->Theta = theta;
//    ptr2->Theta = theta;
//
//    ptr2->AddedPoint = AddedWaypoint;

        AddedWaypoint = CalculateWayPoint(ptr1, ptr2);
        if(ptr2->WaypointCounter <=5)
        {
            ptr1->CollinearFlag = true;
            if(ptr2->WaypointStack.empty())
            {
                ptr2->WaypointStack.push(ptr2->TargetWaypoint);
            }
           
            /* add a way point before the final goal */
            ptr2->WaypointStack.push(AddedWaypoint);

            ptr2->WaypointCounter++;


            // check if robot2 is in inner spiral than that of robot1
            if(dist1 > dist2)
            {
                // check if robot 2 is stopped
                if(ptr2->StopTurningTime > 0)
                {
                    ptr1->StopTurningTime += ptr2->StopTurningTime + 10;
                }
                else
                {
    //                ptr1->StopTurningTime += 10;
                }
            }
            // robot 2 is in the outer spiral than that of robot 1
            else
            {
    //            ptr2->StopTurningTime += 10;
            }


        }
   

}

/****************************************************************************************************************/
/* Function to check if robot paths intersect */
/****************************************************************************************************************/
//void DSA_loop_functions::Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
//                                           BaseController::IntersectionData &ptr3){
//void DSA_loop_functions::Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
//                                           BaseController::IntersectionData *ptr3){
//    argos::Real x_inter;
//    argos::Real y_inter;
//    argos::Real A1, A2, B1, B2, C1, C2, det;
//    bool Intersection_flag = false;
//
//    /* get the start and target position of robot 1 */
////    argos::CVector2 StartPosition_Robot1 = ptr1.StartWaypoint;
////    argos::CVector2 TargetPosition_Robot1 = ptr1.TargetWaypoint;
//
//    argos::CVector2 Point1 = pt1;
//    argos::CVector2 Point2 = pt2;
//
////    /* get the start and target position of robot 2 */
////    argos::CVector2 StartPosition_Robot2 = ptr2.StartWaypoint;
////    argos::CVector2 TargetPosition_Robot2 = ptr2.TargetWaypoint;
//
//    argos::CVector2 Point3 = pt3;
//    argos::CVector2 Point4 = pt4;
//
//    /*A1 = Robot1_goal_y - Robot1_start_y*/
//    A1 = Point2.GetY() - Point1.GetY();
//
//
//
//    /*B1 = Robot1_start_x - Robot1_goal_x*/
//    B1 = Point1.GetX() - Point2.GetX();
//
//
//
//    /* C1 = A1 * Robot1_start_x + B1 * Robot1_start_y */
//    C1 = A1 * Point1.GetX() + B1 * Point1.GetY();
//
//
//    /*A2 = Robot2_goal_y - Robot2_start_y*/
//    A2 = Point4.GetY() - Point3.GetY();
//
//    /*B2 = Robot2_start_x - Robot2_goal_x*/
//    B2 = Point3.GetX() - Point4.GetX();
//
//    /* C2 = A2 * Robot2_start_x + B2 * Robot2_start_y */
//    C2 = A2 * Point3.GetX() + B2 * Point3.GetY();
//
//    det = A1*B2 - A2*B1;
//
//    if(det == 0)
//    {
//        /* Lines are parallel */
//        ptr3->Intersection_flag= 0;
//    }
//
//    /* Lines intersect and find the intersection point */
//    else
//    {
//        x_inter = (B2*C1 - B1*C2)/det;
//
//        y_inter = (A1*C2 - A2*C1)/det;
//
//        /* Check if intersection point is out of bound for the line segment */
//        if((x_inter < std::max(std::min(Point1.GetX(), Point2.GetX()),
//                               std::min(Point3.GetX(), Point4.GetX()))) or
//           (x_inter > std::min(std::max(Point1.GetX(), Point2.GetX()),
//                               std::max(Point3.GetX(), Point4.GetX()))))
//
//        {
////            ptr3->Intersection_flag = false;
//            Intersection_flag = false;
//        }
//        else if((y_inter < std::max(std::min(Point1.GetY(), Point2.GetY()),
//                                    std::min(Point3.GetY(), Point4.GetY()))) or
//                (y_inter > std::min(std::max(Point1.GetY(), Point2.GetY()),
//                                    std::max(Point3.GetY(), Point4.GetY()))))
//        {
////            ptr3->Intersection_flag = false;
//            Intersection_flag = false;
//        }
//        else
//        {
//            Intersection_flag = true;
////            ptr3->Intersection_flag = true;
////            ptr3->IntersectionPoint.Set(x_inter, y_inter);
//
//        }
//    }
//
//    ptr3->Intersection_flag = Intersection_flag;
//    ptr3->IntersectionPoint.Set(x_inter, y_inter);
//}

/****************************************************************************************************************/
/* Function to check if robot paths intersect */
/* CVector2 pt1, CVector2 pt2: Start point and end point of robot1
 * CVector2 pt3, CVector2 pt4: Start point and end point of robot2
 * std::vector<BaseController::IntersectionData> *ptr3 : Intersection data for robot1
 * std::vector<BaseController::IntersectionData> *ptr4 : Intersection data for robot2
 * argos::UInt16 RobotId1, argos::UInt16 RobotId2: Robot Ids of robot 1 and robot2 */
/****************************************************************************************************************/
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
            Intersection_flag = 0;
       
            
        }
    }
    IntersectionPoint.Set(x_inter, y_inter);
    
//    ptr3->Intersection_flag = Intersection_flag;
//    ptr3->IntersectionPoint.Set(x_inter, y_inter);
//    ptr3->Robot_ID_Intersectingwith
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


/*****************************************************************************************************************/
/* Check the direction of vector 1 with respect to direction of vector 2 */
/*****************************************************************************************************************/
argos::Real DSA_loop_functions::CheckDirection(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
    argos::Real direction;
    
    direction = (ptr1->StartWaypoint.GetX() - ptr2->StartWaypoint.GetX())*(ptr2->TargetWaypoint.GetY() - ptr2->StartWaypoint.GetY()) -
    (ptr1->StartWaypoint.GetY() - ptr2->StartWaypoint.GetY())*(ptr2->TargetWaypoint.GetX() - ptr2->StartWaypoint.GetX());
    
    return direction;
}


/*****************************************************************************************************************/
/* Function To Calculate Waypoint */
/*****************************************************************************************************************/
argos::CVector2 DSA_loop_functions::CalculateWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2)
{
    argos::Real ret_direction;
    argos::CVector2 ResultantVector, RotatedVector, Waypoint;
    CRadians theta;
    
    argos::Real AngleRotationValue = 45.0f;
    argos::CRadians VectorAngle;
    argos::Real TurnAngle;
    argos::Real VectorLength;
    
    ResultantVector = (ptr2->TargetWaypoint - ptr2->StartWaypoint);
    VectorAngle = (ResultantVector).Angle();
    VectorLength = (ResultantVector).Length();
    
    //
    //    PolarCoordinates.FromPolarCoordinates(VectorLength, VectorAngle);
    
    ret_direction = CheckDirection(ptr1, ptr2);
    
    
    // if ret_direction is negative, the point is to the left
    
    if(ret_direction < 0.0)
    {
        //turn the robot to right
        TurnAngle = ToDegrees(VectorAngle).GetValue() + AngleRotationValue;
        theta = ToRadians(CDegrees(TurnAngle));
    }
    // if ret_direction is positive, the point is to the right
    else
    {
        //turn the robot to left
        TurnAngle = ToDegrees(VectorAngle).GetValue() - AngleRotationValue;
        theta = ToRadians(CDegrees(TurnAngle));
    }
    
    ptr2->Theta =theta;
    ptr1->Theta = theta;
    
    
    RotatedVector = ResultantVector.Rotate(theta);
    Waypoint = CalculatePointAtDistanceAlongVectorDirection(ptr2->StartWaypoint, RotatedVector, 0.4);
//    if(VectorLength > 0.7 or VectorLength < 0.7)
//    if(VectorLength > 0.9 or VectorLength < 0.9)
//    {
//        Waypoint = CalculatePointAtDistanceAlongVectorDirection(ptr2->StartWaypoint, RotatedVector, 0.4);
//    }
//    else{
////        Waypoint = RotatedVector;
////        Waypoint.Set(0,0);
//    }
//
    return Waypoint;
}



/**************************************************************************************************************************/
/* Function to calculate point a point a distance away on a vector direction */
/**************************************************************************************************************************/
argos::CVector2 DSA_loop_functions::CalculatePointAtDistanceAlongVectorDirection(argos::CVector2 Point1, argos::CVector2 Point2, argos::Real Distance)
{
    
    
    argos::CVector2 ResultantVector = (Point2 - Point1);
    argos::Real vectorlength = (ResultantVector).Length();
    
    argos::CVector2 NormalizedVector = ResultantVector.Normalize();
    argos::CVector2 du = NormalizedVector * Distance;
    
    argos::CVector2 VectorAtDistance = du + Point1;
    return VectorAtDistance;
}

/**************************************************************************************************************************/
/* Function to calculate time required by robot to reach intersection point */
/**************************************************************************************************************************/
//void DSA_loop_functions::IntersectionCollisionCheck(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                                    BaseController::IntersectionData &ptr3){
//void DSA_loop_functions::IntersectionCollisionCheck(CVector2 pt1, CVector2 pt2,
//                                                    BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
//                                                    BaseController::IntersectionData *ptr3, argos::UInt8 index)
//{
//    argos::UInt16 TicksToWait_Robot1, TicksToWait_Robot2, TicksToWaitforSafedistance, TimeToIntersection, TimeDiff, time_turn_safe_dist,            TimeToTurn_2;
//    argos::Real DistanceToIntersection_Robot1, DistanceToIntersection_Robot2, IntersectionDistance, DistanceFromStartPoint,DistFromNest1,    DistFromNest2;
//    argos::Real AdjustedVelocity;
//    bool Intersection_flag;
//
//
//    TicksToWait_Robot1 = 0;
//    TicksToWait_Robot2 = 0;
////    ptr1->Intial_TurningWaitTime = 0;
////    ptr2->Intial_TurningWaitTime = 0;
//
//    /* Get the distance between start point and intersection point */
//    if(index == 2)
//    {
//        DistanceFromStartPoint = CalculateDistance(ptr2->StartWaypoint, pt2);
//        if((ptr2->StartWaypoint - ptr2->TargetWaypoint).Angle() != (pt2 - ptr3->IntersectionPoint).Angle())
//        {
//            argos::CRadians Headingangle = (ptr3->IntersectionPoint - pt2).Angle();
//            argos::CRadians TurningAngle = (ptr2->Orientation - Headingangle).SignedNormalize();
//            TimeToTurn_2 = GetTimeToTurn(ToDegrees(TurningAngle).GetValue(), ptr2->fBaseAngularWheelSpeed) + 20;
//
////            TimeToTurn_2 = GetTimeToTurn(90, ptr2->fBaseAngularWheelSpeed) + 20;
//
//        }
//        else
//        {
//
//            TimeToTurn_2 = 0;
//        }
//    }
//    else
//    {
//        DistanceFromStartPoint = 0;
//        TimeToTurn_2 = 0;
//    }
//
//    DistanceToIntersection_Robot1 = CalculateDistance(pt1, ptr3->IntersectionPoint);
//    DistanceToIntersection_Robot2 = CalculateDistance(pt2, ptr3->IntersectionPoint);
//
//
//    /* calculate the time required to reach the intersection point */
//    argos::CRadians AngleToHead = (ptr1->TargetWaypoint - ptr1->StartWaypoint).Angle();
//    argos::CRadians AngleToTurn = (ptr1->Orientation - AngleToHead).SignedNormalize();
//
////    ptr1->AngleTurn = AngleToTurn;
////    ptr2->AngleTurn = ToRadians(argos::CDegrees(0.0f));
//
//    argos::UInt16 TimeToTurn_1 = GetTimeToTurn(ToDegrees(AngleToTurn).GetValue(), ptr1->fBaseAngularWheelSpeed) + 20;
//
//    TicksToWait_Robot1 = GetTicksToWait(DistanceToIntersection_Robot1, ptr1->fLinearWheelSpeed) + ptr1->StopTurningTime + TimeToTurn_1;
//
//
//    TicksToWait_Robot2 = GetTicksToWait(DistanceToIntersection_Robot2, ptr2->fLinearWheelSpeed) + ptr2->StopTurningTime + TimeToTurn_2;
//
//    TimeDiff = abs(TicksToWait_Robot1 - TicksToWait_Robot2);
//
//    TicksToWaitforSafedistance = (GetTicksToWait(Safedistance , MaxLinearSpeed) + (GetTimeToTurn(180, ptr1->fBaseAngularWheelSpeed)) + 20);
//
////    ptr1->Intial_TurningWaitTime = TicksToWait_Robot1;
////    ptr2->Intial_TurningWaitTime = TicksToWait_Robot2;
//
////    if(ptr1->GoingToOrFromNest == true)
////    {
////        ptr2->StopTurningTime = 200;
////    }
////    else if(ptr2->GoingToOrFromNest == true)
////    {
////        ptr1->StopTurningTime = 200;
////    }
////    else
////    {
////        ptr2->StopTurningTime = 200;
////    }
//
//    //if the difference between the times is equal to safe distance time between two robots
//    if((TimeDiff <= (TicksToWaitforSafedistance)))
//    {
//
////         there is a chance of collision
////         slow down the velocity of robot 2 as its priority is lower
//
//        if((ptr1->GoingToOrFromNest == true or ptr1->WaypointStackpopped == true) and
//           (DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2))
//        {
//            IntersectionDistance = DistanceToIntersection_Robot1;
//            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = MaxLinearSpeed;
//                ptr2->StopTurningTime += ((TimeToIntersection + 20));
//
//            }
//
//            ptr2->fLinearWheelSpeed = AdjustedVelocity;
//
//        }
//
//        else if((ptr2->GoingToOrFromNest == true or ptr2->WaypointStackpopped == true) and
//                (DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1))
//        {
//            IntersectionDistance = DistanceToIntersection_Robot1;
//            TimeToIntersection = TicksToWait_Robot1 + TicksToWaitforSafedistance;
//            AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//            if(AdjustedVelocity < MinLinearSpeed)
//            {
//                AdjustedVelocity = MaxLinearSpeed;
//                ptr1->StopTurningTime += ((TimeToIntersection + 20));
//            }
//            ptr1->fLinearWheelSpeed = AdjustedVelocity;
//        }
//        else
//        {
//            if((DistanceToIntersection_Robot1 < DistanceToIntersection_Robot2) and
//               (ptr1->GoingToOrFromNest == false) and (TicksToWait_Robot1 < TicksToWait_Robot2))
//            {
//                IntersectionDistance = DistanceToIntersection_Robot1;
//                TimeToIntersection = TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = MaxLinearSpeed;
//                    // adding extr time as spirals are close enough
//                    ptr2->StopTurningTime += ((TimeToIntersection + 20));
//                }
//                ptr2->fLinearWheelSpeed = AdjustedVelocity;
//
//
//            }
//
//            else if((DistanceToIntersection_Robot2 < DistanceToIntersection_Robot1) and
//                    (ptr2->GoingToOrFromNest == false) and (TicksToWait_Robot2 < TicksToWait_Robot1))
//            {
//                IntersectionDistance = DistanceToIntersection_Robot2;
//                TimeToIntersection = TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = MaxLinearSpeed;
//                    ptr1->StopTurningTime += ((TimeToIntersection + 20));
//                }
//                ptr1->fLinearWheelSpeed = AdjustedVelocity;
//
//            }
//            else
//            {
//
//                IntersectionDistance = DistanceToIntersection_Robot1;
//                TimeToIntersection = TicksToWaitforSafedistance;
//                AdjustedVelocity = (IntersectionDistance/TimeToIntersection) * SimulatorTicksperSec;
//                if(AdjustedVelocity < MinLinearSpeed)
//                {
//                    AdjustedVelocity = MaxLinearSpeed;
//                    ptr2->StopTurningTime += ((TimeToIntersection+ 20));
//                }
//                ptr2->fLinearWheelSpeed = AdjustedVelocity;
//
//            }
//        }
//
//
//    }
//
//    /* Reset the flag */
//    ptr3->Intersection_flag = 0;
//
//}

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
//    TicksToWaitforSafedistance = GetTicksToWait(Safedistance, Velocity) + GetTimeToTurn(180, Velocity) + 10;
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
//    if(abs(IntersectionTime1 - IntersectionTime2) <= (TicksToWaitforSafedistance))
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
////                ptr2.StopTurningTime += (2*TimeToIntersection);
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
////                ptr1.StopTurningTime += (2*TimeToIntersection);
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
////                    ptr1.StopTurningTime += (2*TimeToIntersection);
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
////                    ptr2.StopTurningTime += (2*TimeToIntersection);
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




REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
