#ifndef BASECONTROLLER_H
#define BASECONTROLLER_H

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/simulator/loop_functions.h>
#include <cmath>
#include <stack>

/**
 * BaseController
 * @author Antonio Griego
 */
class BaseController : public argos::CCI_Controller {

    public:

        BaseController();

        /*  navigation functions */
        argos::CRadians GetHeading();
        argos::CVector2 GetPosition();
        argos::CVector2 GetTarget();
        void SetTarget(argos::CVector2 t);
        void SetStartPosition(argos::CVector3 sp);
        argos::CVector3 GetStartPosition();
        size_t GetMovementState();

        void Stop();
        void Move();
        bool Wait();
        void Wait(size_t wait_time_in_seconds);
    
        void PushMovement(size_t moveType, argos::Real moveSize);
    
        void SetMovement();
        bool CheckStopTime();
        void SetStopMovement();
        void SetHardStopMovement();
        void ResetIntersectionData();
        /*  time calculation functions */
        size_t SimulationTick();
        size_t SimulationTicksPerSecond();
        argos::Real SimulationSecondsPerTick();
        argos::Real SimulationTimeInSeconds();
    
    public:
//        bool StopMovement;;
        // controller state variables
        enum MovementState {
            STOP    = 0,
            LEFT    = 1,
            RIGHT   = 2,
            FORWARD = 3,
            BACK    = 4
        } CurrentMovementState;
    
        struct RobotData{
            argos::CVector2 TargetPosition;
            argos::CVector2 StartPosition;
            argos::CVector2 TargetWaypoint;
            argos::CVector2 StartWaypoint;
            argos::CVector2 AddedPoint;
            argos::UInt16 id_robot;
//            argos::UInt16 Total_robots;
//            argos::Real Priority;
            argos::UInt16 WaypointCounter;
            argos::Real fBaseAngularWheelSpeed;
            argos::Real fLinearWheelSpeed;
            bool GoingToNest;
            bool GoingToOrFromNest;
//            bool MovementStarted;
            bool StopMovement;
//            argos::CVector2 NormailzedVector;
//            argos::UInt16 Intial_TurningWaitTime;
            argos::UInt16 StopTurningTime;
            bool Checked;
            bool Waypoint_Added;
//            bool Intersection_Adjustment;
            bool CollinearFlag;
            bool AddWaypoint;
            bool pathcheck;
            bool WaypointStackpopped;
            argos::CRadians Orientation;
            argos::CDegrees HeadingAngle;
            std::stack<argos::CVector2>WaypointStack;
            std::vector<char> pattern;
            argos::CVector2 IntersectionPt1;
            argos::CVector2 IntersectionPt2;
//            argos::CVector3 CrossProduct;
//            argos::UInt16 IntersectionTime;
//            argos::CVector3 vect1;
//            argos::CVector3 vect2;
            argos::CRadians Theta;
//            argos::UInt8 Inter;
//            argos::CVector2 PointChange;
//            argos::CVector2 PointSafe;
            char direction;
            char prevdirection;
//            argos::CRadians AngleTurn;
            std::vector<argos::UInt8>Neighbors;
            std::vector < std::vector<argos::UInt8> > NeighborsMatrix;
//            std::stack<argos::UInt16>CollinearRobots;
//            std::stack<argos::UInt16>IntersectingRobots;
//            argos::CMatrix <>NeighborMatrix;

        };
    
//    struct IntersectionData{
//        argos::UInt16 Robot_ID_Intersectingwith;
//        bool Intersection_flag;
//        argos::Real IntersectionDistance;
//        argos::CVector2 IntersectionPoint;
//    };
        struct IntersectionData{
            argos::UInt16 Robot_ID_Intersectingwith;
            argos::UInt8 Intersection_flag;
            argos::UInt8 Intersection_Type;
            argos::UInt16 StopTimeCalculated;
            argos::Real IntersectionDistance;
            argos::CVector2 StartPoint;
            argos::CVector2 TargetPoint;
            argos::CVector2 IntersectionPoint;
        };
    
        std::vector<IntersectionData> IntersectionDataVector;
 
        /*
         * Returns the robot data
         */
        inline RobotData& GetRobotData() {
            return stRobotData;
        }
    
        /*
         * Returns the intersection data of the robot
         */
//        inline IntersectionData& GetIntersectionData() {
//            return st_IntersectionData;
//        }

//        inline  std::vector<IntersectionData>& GetIntersectionData() {
//            return IntersectionDataVector;
//        }
    
            inline  std::vector<IntersectionData>* GetIntersectionData() {
                return &IntersectionDataVector;
            }

        void SetIsHeadingToNest(bool n);

        bool IsAtTarget();
    
//        inline void ActivatePathPlanning(){
//            path_planning_activated = true;
//        }
    void SetMovementState(size_t state);
    
public:
    RobotData stRobotData;
    
     //movement definition variables
    
    struct Movement {
        size_t type;
        argos::Real magnitude;
    };

    Movement previous_movement;
    argos::CVector2 previous_pattern_position;

    std::stack<Movement> MovementStack;
    
//    IntersectionData st_IntersectionData;
//    argos::Real RobotForwardSpeed;
    size_t WaitTime;

    
    protected:

	argos::CRandom::CRNG* RNG;

	unsigned int collision_counter;
	float DestinationNoiseStdev; // for introducing error in destination positions
	float PositionNoiseStdev; // for introducing error in current position

    

    argos::CRadians TargetAngleTolerance;
    argos::Real NestDistanceTolerance;
    argos::CRadians NestAngleTolerance;
    argos::Real TargetDistanceTolerance;
    argos::Real SearchStepSize;

    argos::CRange<argos::Real> ForageRangeX;
    argos::CRange<argos::Real> ForageRangeY;
    argos::CRange<argos::Real> GoStraightAngleRangeInDegrees;

    //  base controller movement parameters
    argos::Real RobotForwardSpeed;
    argos::Real RobotRotationSpeed;
    argos::Real TicksToWaitWhileMoving;

    // foot-bot components: sensors and actuators
    argos::CCI_PositioningSensor* compassSensor;
    argos::CCI_DifferentialSteeringActuator* wheelActuator;
    argos::CCI_FootBotProximitySensor* proximitySensor;

    

/* movement definition variables */
//    struct Movement {
//        size_t type;
//        argos::Real magnitude;
//    };
//
//    Movement previous_movement;
//    argos::CVector2 previous_pattern_position;
	
//    std::stack<Movement> MovementStack;

    private:

        argos::CLoopFunctions& LF;
        argos::CVector3 StartPosition;
        argos::CVector2 TargetPosition;

                //MovementState CurrentMovementState;
        

        /* private navigation helper functions */
        void SetNextMovement();
        argos::Real SetTargetAngleDistance(argos::Real newAngleToTurnInDegrees);
        argos::Real SetTargetTravelDistance(argos::Real newTargetDistance);
        void SetLeftTurn(argos::Real newTargetAngle);
        void SetRightTurn(argos::Real newTargetAngle);
        void SetMoveForward(argos::Real newTargetDistance);
        void SetMoveBack(argos::Real newTargetDistance);
//        void PushMovement(size_t moveType, argos::Real moveSize);
        void PopMovement();
    

        /* collision detection functions */
        bool CollisionDetection();
        argos::CVector2 GetCollisionVector();

        argos::UInt16 GetInitial_TurningWaitTime(BaseController::RobotData stRobotData);

	bool heading_to_nest;
    bool Initial_State;
};

#endif /* IANTBASECONTROLLER_H */
