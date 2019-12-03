#include "BaseController.h"
#include <random>

using namespace std;

/**
 * Constructor for the BaseController. Several important variables are defined here.
 * <p>
 * TODO: update xml configuration file to allow these to be adjusted from configuration without recompiling.
 */
BaseController::BaseController() :
    LF(argos::CSimulator::GetInstance().GetLoopFunctions()),
    WaitTime(0),
    NestDistanceTolerance(0.01),
    NestAngleTolerance(0.05),
    TargetDistanceTolerance(0.01),
    TargetAngleTolerance(0.04),
    SearchStepSize(0.18),
    RobotForwardSpeed(10.0),
    RobotRotationSpeed(10.0),
    TicksToWaitWhileMoving(0.0),
    CurrentMovementState(STOP),
    heading_to_nest(false),
    DestinationNoiseStdev(0),
    PositionNoiseStdev(0),
    Initial_State(true),
//    StopMovement(false),
    RNG(argos::CRandom::CreateRNG("argos"))
{
    // calculate the forage range and compensate for the robot's radius of 0.085m
    argos::CVector3 ArenaSize = LF.GetSpace().GetArenaSize();
    argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);
    GoStraightAngleRangeInDegrees.Set(-37.5, 37.5);
    
    stRobotData.WaypointCounter = 0;

    stRobotData.StopTurningTime = 0;
    collision_counter = 0;
    stRobotData.CollisionCounter = 0;
    stRobotData.pathcheck = false;
    stRobotData.Orientation = ToRadians(argos::CDegrees(0.0f));

    stRobotData.CollinearFlag = false;
    stRobotData.Waypoint_Added = false;
    stRobotData.StopMovement = false;

    stRobotData.Checked = false;
    stRobotData.GoingToNest = false;
    stRobotData.GoingToOrFromNest = false;
    stRobotData.AddWaypoint = false;
    stRobotData.WaypointStackpopped = false;

    stRobotData.IntersectionPt1.Set(0,0);
    stRobotData.IntersectionPt2.Set(0,0);

    stRobotData.Theta = ToRadians(argos::CDegrees(0.0f));
//    stRobotData.IntersectionTime = 0;
    stRobotData.HeadingAngle = argos::CDegrees(0.0f);
    stRobotData.CosTheta = 0;
    stRobotData.CollinearAvoidance = false;
    stRobotData.QuadrantGroup = 0;


}





/*****************************************************************************************************************/
argos::CRadians BaseController::GetHeading() {
    /* in ARGoS, the robot's orientation is represented by a quaternion */
    const argos::CCI_PositioningSensor::SReading& readings = compassSensor->GetReading();
    argos::CQuaternion orientation = readings.Orientation;

    /* convert the quaternion to euler angles */
    argos::CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* the angle to the z-axis represents the compass heading */

    return z_angle;
}
/*****************************************************************************************************************/
argos::CVector2 BaseController::GetPosition() {
    /* the robot's compass sensor gives us a 3D position */
    argos::CVector3 position3D = compassSensor->GetReading().Position;
    /* return the 2D position components of the compass sensor reading */

    float x = position3D.GetX();
    float y = position3D.GetY();

// Add noise to the current position unless travelling to the nest
  // Make the noise proportional to the distance to the target
    /*
  if (!heading_to_nest)
    {
      argos::Real noise_x = RNG->Gaussian(PositionNoiseStdev);
      argos::Real noise_y = RNG->Gaussian(PositionNoiseStdev);

      x += noise_x;
      y += noise_y;
    }
    */
    return argos::CVector2(x, y);
}
/*****************************************************************************************************************/
argos::CVector2 BaseController::GetTarget() {
//    return TargetPosition;
    return stRobotData.TargetPosition;
//    return stRobotData.TargetWaypoint;
}

void BaseController::SetTarget(argos::CVector2 t) {

  argos::Real x(t.GetX()), y(t.GetY());

  //if(x > ForageRangeX.GetMax()) x = ForageRangeX.GetMax();
  //else if(x < ForageRangeX.GetMin()) x = ForageRangeX.GetMin();
  
  //if(y > ForageRangeY.GetMax()) y = ForageRangeY.GetMax();
  //else if(y < ForageRangeY.GetMin()) y = ForageRangeY.GetMin();
  
  //argos::LOG << "<<Updating Target Position>>" << std::endl;

  if (!heading_to_nest)
    {
      argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
      argos::Real noise_x = RNG->Gaussian(DestinationNoiseStdev*distanceToTarget);
      argos::Real noise_y = RNG->Gaussian(DestinationNoiseStdev*distanceToTarget);
      
      x += noise_x;
      y += noise_y;

      //argos::LOG << "Not Heading to Nest " << std::endl;
      //argos::LOG << "Noise x: "<< noise_x << std::endl;
      //argos::LOG << "Noise y:" << noise_y << std::endl;
    }
  else
    {
      //argos::LOG << "Heading to Nest " << std::endl;
    }

  

  if( y > ForageRangeY.GetMax() 
      || y < ForageRangeY.GetMin()
      || x > ForageRangeX.GetMax()
      || x < ForageRangeX.GetMin() )
    {
      x = GetPosition().GetX();
      y = GetPosition().GetY();
      SetRightTurn(37.5);
    }

  // argos::LOG << "New Target x: "<< x << std::endl;
  //  argos::LOG << "New Target y:" << y << std::endl;
  TargetPosition = argos::CVector2(x, y);
  if(stRobotData.WaypointStack.empty())
  {
      stRobotData.TargetPosition = TargetPosition;
      stRobotData.fLinearWheelSpeed = RobotForwardSpeed;
  }
  stRobotData.StartWaypoint = GetPosition();
  stRobotData.TargetWaypoint = TargetPosition;

  argos::Real distanceToTarget = (stRobotData.TargetWaypoint - GetPosition()).Length();
    
}

/*****************************************************************************************************************/
void BaseController::SetStartPosition(argos::CVector3 sp) {
    argos::CVector2 startpos;

    startpos.Set(sp.GetX(), sp.GetY());
    stRobotData.StartPosition = startpos;
    stRobotData.StartWaypoint = startpos;
    StartPosition = sp;
}

/*****************************************************************************************************************/
argos::CVector3 BaseController::GetStartPosition() {
    
    return StartPosition;
}

size_t BaseController::GetMovementState() {
    return CurrentMovementState;
}

/*****************************************************************************************************************/
void BaseController::SetIsHeadingToNest(bool n)
{
  heading_to_nest = n;
}

/*****************************************************************************************************************/
void BaseController::SetNextMovement() 
{

  argos::CRadians AngleTol;
  float DistTol;
  
  // Allow the searcher to treat movement to the nest
  // differently than other movement
  if (heading_to_nest)
    {
      DistTol = NestDistanceTolerance;
      AngleTol = NestAngleTolerance;
    }
  else
    {
      DistTol = TargetDistanceTolerance;
      AngleTol = TargetAngleTolerance;
    }
    
    if(MovementStack.size() == 0 && CurrentMovementState == STOP)
    {
        argos::Real distanceToTarget = (stRobotData.TargetWaypoint - GetPosition()).Length();
        argos::CRadians headingToTarget = (stRobotData.TargetWaypoint - GetPosition()).Angle();
        stRobotData.Orientation = GetHeading();

        argos::CRadians headingToTargetError = (GetHeading() - headingToTarget).SignedNormalize();
       
        if(!IsAtTarget())
        {
            if(headingToTargetError > AngleTol)
            {
        
            
                PushMovement(LEFT, -ToDegrees(headingToTargetError).GetValue());

            }
            else if(headingToTargetError < -AngleTol)
            {
          
            
              PushMovement(RIGHT, ToDegrees(headingToTargetError).GetValue());
            
            }
            else
            {
              //cout << "Move Forward " << endl;
              PushMovement(FORWARD, distanceToTarget);
            }
        }
            /* if stack is empty, robot has reached the target */
        else{

                SetMovement();
                PushMovement(STOP, 0.0);

                if(!stRobotData.WaypointStack.empty())
                {
                    SetTarget(stRobotData.WaypointStack.top());
                    stRobotData.AddedPoint.Set(0,0);
                    stRobotData.CollectiveCollinearChecked = false;
                    stRobotData.WaypointStack.pop();
                    
                }
                else if(stRobotData.GoingToOrFromNest == true and stRobotData.WaypointStack.empty())
                {
                   stRobotData.CollectiveCollinearChecked = false;
                }
            
            
                stRobotData.Waypoint_Added = true;
                stRobotData.pathcheck = false;
                stRobotData.fLinearWheelSpeed = RobotForwardSpeed;
                stRobotData.fBaseAngularWheelSpeed = RobotRotationSpeed;
            
                if(stRobotData.CollinearRobotGoingToNestList.size() > 0)
                {
                    stRobotData.CollinearRobotGoingToNestList.clear();
                    
                }
                if(stRobotData.CollinearRobotGoingAwayNestList.size() > 0)
                {
                    stRobotData.CollinearRobotGoingAwayNestList.clear();
                }
        }
    }
    else
    {
        PopMovement();
    }
}

/*****************************************************************************************************************/
argos::Real BaseController::SetTargetAngleDistance(argos::Real newAngleToTurnInDegrees) {
    // s = arc_length = robot_radius * turning_angle
    // NOTE: the footbot robot has a radius of 0.085 m... or 8.5 cm...
    // adjusting with + 0.02 m, or + 2 cm, increases accuracy...
  argos::Real TicksToWait;
  argos::Real s = 0.105 * newAngleToTurnInDegrees;
  TicksToWait = std::ceil((SimulationTicksPerSecond() * s) / stRobotData.fBaseAngularWheelSpeed);
  return TicksToWait;
}

/*****************************************************************************************************************/
argos::Real BaseController::SetTargetTravelDistance(argos::Real newTargetDistance) {
    // convert meters into cm
    argos::Real TicksToWait;
    argos::Real d = newTargetDistance * 100.0;
    TicksToWait = std::ceil((SimulationTicksPerSecond() * d) / stRobotData.fLinearWheelSpeed);
//    TicksToWaitWhileMoving = std::ceil((SimulationTicksPerSecond() * d) / RobotForwardSpeed);
    return TicksToWait;
}

/*****************************************************************************************************************/
void BaseController::SetLeftTurn(argos::Real newAngleToTurnInDegrees) {
    if(newAngleToTurnInDegrees > 0.0) {
        TicksToWaitWhileMoving = SetTargetAngleDistance(newAngleToTurnInDegrees);
        CurrentMovementState = LEFT;
    } else if(newAngleToTurnInDegrees < 0.0) {
        TicksToWaitWhileMoving = SetTargetAngleDistance(-newAngleToTurnInDegrees);
        CurrentMovementState = RIGHT;  
    } else {
        Stop();
    }
}


/*****************************************************************************************************************/
void BaseController::SetRightTurn(argos::Real newAngleToTurnInDegrees) {
    if(newAngleToTurnInDegrees > 0.0) {
        TicksToWaitWhileMoving = SetTargetAngleDistance(newAngleToTurnInDegrees);
        CurrentMovementState = RIGHT;
    } else if(newAngleToTurnInDegrees < 0.0) {
        TicksToWaitWhileMoving = SetTargetAngleDistance(-newAngleToTurnInDegrees);
        CurrentMovementState = LEFT;
    } else {
        Stop();
    }
}

/*****************************************************************************************************************/
void BaseController::SetMoveForward(argos::Real newTargetDistance) {
    if(newTargetDistance > 0.0) {
        TicksToWaitWhileMoving = SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = FORWARD;
    } else if(newTargetDistance < 0.0) {
        TicksToWaitWhileMoving = SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = BACK;
    } else {
        Stop();
    }
}

/*****************************************************************************************************************/
void BaseController::SetMoveBack(argos::Real newTargetDistance) {
    if(newTargetDistance > 0.0) {
        TicksToWaitWhileMoving = SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = BACK;
    } else if(newTargetDistance < 0.0) {
        TicksToWaitWhileMoving = SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = FORWARD;
    } else {
        Stop();
    }
}

/*****************************************************************************************************************/
void BaseController::PushMovement(size_t moveType, argos::Real moveSize) {
    Movement newMove = { moveType, moveSize };
    MovementStack.push(newMove);
}


/*****************************************************************************************************************/
void BaseController::PopMovement() {
    Movement nextMove = MovementStack.top();

    previous_movement = nextMove;

    MovementStack.pop();

    switch(nextMove.type) {

        case STOP: {
            Stop();
            break;
        }

        case LEFT: {
            SetLeftTurn(nextMove.magnitude);
            break;
        }

        case RIGHT: {
            SetRightTurn(nextMove.magnitude);
            break;
        }

        case FORWARD: {
            SetMoveForward(nextMove.magnitude);
            break;
        }

        case BACK: {
            SetMoveBack(nextMove.magnitude);
            break;
        }

    }

}



/*****************************************************************************************************************/
argos::UInt8 BaseController::GetDirectionOfPoint(argos::CVector2 Point1, argos::CVector2 Point2, argos::CVector2 Point)
{
    argos::Real Cross_Product;
    Direction ret_dir;
    Point2.SetX(Point2.GetX() - Point1.GetX());
    Point2.SetY(Point2.GetY() - Point1.GetY());
    Point.SetX(Point.GetX() - Point1.GetX());
    Point.SetY(Point.GetY() - Point1.GetY());
    
    Cross_Product = Point2.GetX() * Point.GetY() - Point2.GetY() * Point.GetX();
    
    if(Cross_Product > 0)
    {
        ret_dir =  RIGHT_DIR;
    }
    else if(Cross_Product < 0)
    {
        ret_dir =  LEFT_DIR;
    }
    else{
        ret_dir =  ZERO;
    }
    
    return ret_dir;
}

/**************************************************************************************************************************/
void BaseController::SortRobotInAscOrder(std::vector<argos::UInt8>* ptr, argos::UInt8 size)
{
    int i, j, temp;
    argos::Real dist1, dist2;
    argos::CVector2 NestPosition;

    BaseController::RobotData *stRobotDataThis = NULL;
    BaseController::RobotData *stRobotDataNext = NULL;

    
    argos::CSpace::TMapPerType& m_cFootbots = LF.GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator iterator1 = m_cFootbots.begin();
    argos::CSpace::TMapPerType::iterator iterator2 = m_cFootbots.begin();
    
    // dereference the pased vector pointer to easily access the vector elements
    std::vector<argos::UInt8>& vecRef = *ptr;
    
    NestPosition.Set(0.0f, 0.0f);
    
    for(i=0; i < size; i++)
    {
        iterator1 = m_cFootbots.begin();
        // get the handle to particular robot
        std::advance(iterator1, vecRef[i]);
        argos::CFootBotEntity& cFootBotThis = *argos::any_cast<argos::CFootBotEntity*>(iterator1->second);
        BaseController& cControllerThis = dynamic_cast<BaseController&>(cFootBotThis.GetControllableEntity().GetController());

        // Get the robot data
        stRobotDataThis = &cControllerThis.GetRobotData();
        
        dist1 = (NestPosition - stRobotDataThis->StartWaypoint).Length();
        
        
        for(j = i+1; j <= size-1; j++)
        {
            iterator2 = m_cFootbots.begin();
            // get the handle to particular robot
            std::advance(iterator2, vecRef[j]);
            argos::CFootBotEntity& cFootBotNext = *argos::any_cast<argos::CFootBotEntity*>(iterator2->second);
            BaseController& cControllerNext = dynamic_cast<BaseController&>(cFootBotNext.GetControllableEntity().GetController());

            // Get the robot data
            stRobotDataNext = &cControllerNext.GetRobotData();

            dist2 = (NestPosition - stRobotDataNext->StartWaypoint).Length();

            // arrange the robot in ascending order of distance from nest
            if(dist2 < dist1)
            {
                temp = vecRef[i];
                vecRef[i] = vecRef[j];
                vecRef[j] = temp;
            }
        }
    }
    
}



/*****************************************************************************************************************/
/* Function to handle collision detection */
bool BaseController::CollisionDetection()
{
    
//    argos::CVector2 Perpendicular, CurrentVector;

    argos::CVector2 CurrentVector_Copy, collisionvec_copy;
    argos:: Real Nest_Radius_Squared = 0.0625;

    argos::Real dist, dist_waypt;
    argos::CRadians angle, angle_error;
    argos::CRadians orientation;
    argos::UInt8 CollinearType;
    bool NormalCollisionHandling = true;
    
    argos::CSpace::TMapPerType& m_cFootbots = LF.GetSpace().GetEntitiesByType("foot-bot");



    collisionVector = GetCollisionVector();

    
    collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
    bool isCollisionDetected = false;
    
    dist = 0.0;
    dist_waypt = 0.0f;
    
    
    // check if the collision vector is to the left or right of current vector

    if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
       && collisionVector.Length() > 0.0)
    {

        Stop();
        isCollisionDetected = true;
        orientation = GetHeading();
        while(MovementStack.size() > 0) MovementStack.pop();
        
        CurrentVector = (stRobotData.TargetWaypoint - stRobotData.StartWaypoint);
        CurrentVectorAngle = ToDegrees(CurrentVector.Angle()).GetValue();
        CurrentVector_Copy = CurrentVector;
        collisionvec_copy = collisionVector;
        Perpendicular = CurrentVector_Copy.Perpendicularize();
        collisionVector_adj = collisionvec_copy.Rotate(orientation);

        /* sort the robots in ascending order of distance from nest */
        if(stRobotData.CollinearRobotGoingToNestList.size() > 0)
        {
            SortRobotInAscOrder(&stRobotData.CollinearRobotGoingToNestList, stRobotData.CollinearRobotGoingToNestList.size());

        }
        if(stRobotData.CollinearRobotGoingAwayNestList.size() > 0)
        {
            SortRobotInAscOrder(&stRobotData.CollinearRobotGoingAwayNestList, stRobotData.CollinearRobotGoingAwayNestList.size());

        }


        /* Robot Going to nest */
        if(stRobotData.GoingToNest == true and stRobotData.CollinearRobotGoingToNestList.size() > 0)
        {
            CollinearType = 1;
        }
         /* Robot Going away from nest */
        else if((stRobotData.GoingToNest == false and stRobotData.GoingToOrFromNest == true) and
                stRobotData.CollinearRobotGoingAwayNestList.size() > 0)
        {
            CollinearType = 2;
        }
        /* robot in the loop */
        else
        {
            CollinearType = 0;
        }

        directioncollision = GetDirectionOfPoint(stRobotData.StartWaypoint, Perpendicular,  collisionVector_adj);

        /* if the dot product is  +ve-> 2 vectors in same direction
         * else if dot product is -ve-> 2 vectors in opposite direction */
        DotProductVectors = CurrentVector.DotProduct(collisionVector_adj);

        if(AlgorithmActivated == true and CollinearType != 0)
        {
            
            // if the dot product is +ve then the collision is in the same direction
            if(DotProductVectors > 0.0f)
            {
                // collineartype  = 2 => collinear robots going away from nest
                if(CollinearType == 2)
                {

                    /* when robot going away from nest
                    if the collision vector is to the left of perpendicular, it is to the back of the current robot
                    then add stop time */
                    if(directioncollision == LEFT_DIR)
                    {
                        /* don't do anything */
                    }
                    // this robot is to the back, then add stop time
                    else if((directioncollision == RIGHT_DIR) or (directioncollision == ZERO))
                    {
                        NormalCollisionHandling = false;
                        stRobotData.StopTurningTime += (SimulationTicksPerSecond() * 3) ;
                    }
                    else
                    {
                        NormalCollisionHandling = true;
                    }

                } // end of if(CollinearType == 2)

                // collineartype  = 1 => collinear robots going to nest
                else if(CollinearType == 1)
                {
                    /* when robot going to nest
                    if the collision vector is to the right of perpendicular, it is to the back of the current robot
                    then add stop time to other robot */
                    if(directioncollision == RIGHT_DIR)
                    {
                        /* don't do anything */
                    }
                    // this robot is to the back, then add stop time
                    else if((directioncollision == LEFT_DIR) or (directioncollision == ZERO))
                    {
                        NormalCollisionHandling = false;
                        stRobotData.StopTurningTime += (SimulationTicksPerSecond() * 3);
                    }
                    // same location
                    else
                    {
                        NormalCollisionHandling = true;
                    }
                }// end of else if(CollinearType == 1)
            }// end of if(DotProductVectors > 0.0f)
        } // end of if(AlgorithmActivated == true and CollinearType != 0)

        if(NormalCollisionHandling == true)
        {
            collision_counter++;
            stRobotData.CollisionCounter++;
            PushMovement(FORWARD, SearchStepSize);
            
            if(collisionAngle <= 0.0)
            {
                SetLeftTurn(37.5 - collisionAngle);
            }
            else
            {
                SetRightTurn(37.5 + collisionAngle);
            }
        }

 
    }
        return isCollisionDetected;
   
}

/*****************************************************************************************************************/
argos::CVector2 BaseController::GetCollisionVector() {
    prox_size = 0;
    /* Get readings from proximity sensor */
    const argos::CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();

    /* Sum them together */
    argos::CVector2 collisionVector;
    
    

    for(size_t i = 0; i < proximityReadings.size(); ++i) {
        if(proximityReadings[i].Value != 0)
        {
            prox_size++;
        }
        collisionVector += argos::CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }

    collisionVector /= proximityReadings.size();


    return collisionVector;
}

/*****************************************************************************************************************/
void BaseController::Stop() {
    SetTargetTravelDistance(0.0);
    SetTargetAngleDistance(0.0);
    TicksToWaitWhileMoving = 0.0;
    CurrentMovementState = STOP;
}

/*****************************************************************************************************************/
bool BaseController::CheckStopTime()
{
    bool RobotStopped = false;
    if((stRobotData.StopTurningTime > 0.0))
    {
        Stop();
        wheelActuator->SetLinearVelocity(0.0, 0.0);
        stRobotData.StopTurningTime--;
        RobotStopped = true;
    }
    return RobotStopped;
}


/*****************************************************************************************************************/
void BaseController::Move()
{

    
    if(CheckStopTime() == true) return;
    
    CollisionDetection();

    /* move based on the movement state flag */
    switch(CurrentMovementState)
    {

    /* stop movement */
    case STOP:
    {
        wheelActuator->SetLinearVelocity(0.0, 0.0);
        SetNextMovement();

        break;
    }
            

    /* turn left until the robot is facing an angle inside of the TargetAngleTolerance */
    case LEFT:
    {
        if((TicksToWaitWhileMoving--) <= 0.0)
        {
            Stop();
        }
        else
        {
   
            wheelActuator->SetLinearVelocity(-stRobotData.fBaseAngularWheelSpeed, stRobotData.fBaseAngularWheelSpeed);
        }
        break;
    }

        /* turn right until the robot is facing an angle inside of the TargetAngleTolerance */
        case RIGHT: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {
                wheelActuator->SetLinearVelocity(stRobotData.fBaseAngularWheelSpeed, -stRobotData.fBaseAngularWheelSpeed);
            }
            break;
        }

        /* move forward until the robot has traveled the specified distance */
        case FORWARD:
        {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {

                 wheelActuator->SetLinearVelocity(stRobotData.fLinearWheelSpeed,
                                                  stRobotData.fLinearWheelSpeed);
            }
            break;
        }

        /* move backward until the robot has traveled the specified distance */
        case BACK: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {

                wheelActuator->SetLinearVelocity(-stRobotData.fLinearWheelSpeed,
                                                 -stRobotData.fLinearWheelSpeed);
            }
            break;
        }
            
       
        
    }
}

/*****************************************************************************************************************/
bool BaseController::Wait() {

    bool wait = false;

    if(WaitTime > 0) {
        WaitTime--;
        wait = true;
    }

    return wait;
}

/*****************************************************************************************************************/
void BaseController::Wait(size_t wait_time_in_seconds) {

    WaitTime += (wait_time_in_seconds * SimulationTicksPerSecond());

}

/*****************************************************************************************************************/
size_t BaseController::SimulationTick() {
    return LF.GetSpace().GetSimulationClock();
}

/*****************************************************************************************************************/
size_t BaseController::SimulationTicksPerSecond() {
    
    return LF.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();
}

/*****************************************************************************************************************/
argos::Real BaseController::SimulationSecondsPerTick() {
    return LF.GetSimulator().GetPhysicsEngine("default").GetSimulationClockTick();
}

/*****************************************************************************************************************/
argos::Real BaseController::SimulationTimeInSeconds() {
    return (argos::Real)(SimulationTick()) * SimulationSecondsPerTick();
}

/*****************************************************************************************************************/
void BaseController::SetHardStopMovement()
{
    Stop();
    while(!MovementStack.empty())
    {
        MovementStack.pop();
    }
    
//    PushMovement(STOP, 0.0);
    stRobotData.StopMovement = true;
}



/***************************************************************************************************/
/* Function to stop movement */
/***************************************************************************************************/
void BaseController::SetStopMovement()
{
    PushMovement(STOP, 0.0);
    Stop();
    stRobotData.StopMovement = true;
}

/***************************************************************************************************/
/* Function to resume movemnet after checking */
/***************************************************************************************************/
void BaseController::SetMovement()
{
    stRobotData.StopMovement = false;
    while(!MovementStack.empty())
    {
        MovementStack.pop();
    }
    Stop();
    
}

/*****************************************************************************************************************/
bool BaseController::IsAtTarget() 
{
  argos::CRadians AngleTol;
  float DistTol;
  
  // Allow the searcher to treat movement to the nest
  // differently than other movement
  if (heading_to_nest)
    {
      DistTol = NestDistanceTolerance;
      AngleTol = NestAngleTolerance;
    }
  else
    {
      DistTol = TargetDistanceTolerance;
      AngleTol = TargetAngleTolerance;
    }
    argos::Real distanceToTarget = (stRobotData.TargetWaypoint - GetPosition()).Length();


    //argos::LOG << "IsAtTarget: Distance to Target: " << distanceToTarget << endl;
    //argos::LOG << "IsAtTarget: TargetDistanceTolerance: " << DistTol << endl;

    return (distanceToTarget < DistTol) ? (true) : (false);
}

//REGISTER_CONTROLLER(BaseController, "BaseController")
