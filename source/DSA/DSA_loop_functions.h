#ifndef DSA_LOOP_FUNCTIONS_H
#define DSA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/DSA/DSA_controller.h>
#include <source/Base/BaseController.h>


using namespace argos;
using namespace std;

class DSA_loop_functions : public argos::CLoopFunctions {

	friend class DSA_controller;
	friend class DSA_qt_user_functions;

	public:

		DSA_loop_functions();

		void Init(TConfigurationNode& node);
		void PreStep();

		void PostExperiment();

		/* Calculates the performance of the robot in a trial */
		Real Score();
    
        void SetAverageCollision();
	

        void SetFoodDistribution();

	argos::Real getSimTimeInSeconds();
    
    argos::Real CalculateDistance(argos::CVector2 cPosition1, argos::CVector2 cPosition2);
    argos::UInt16 GetTicksToWait(argos::Real dist, argos::Real speed);
    
//    void Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
//                                               BaseController::IntersectionData &ptr3);
    
//    void Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
//                                               BaseController::IntersectionData *ptr3);
    argos::UInt8 Find_Intersection(CVector2 pt1, CVector2 pt2, CVector2 pt3, CVector2 pt4,
                                               std::vector<BaseController::IntersectionData> *ptr3,
                                               std::vector<BaseController::IntersectionData> *ptr4,
                           BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, bool CollinearIntersection);
    
//    void IntersectionCollisionCheck(argos::CVector2 pt1, argos::CVector2 pt2,               BaseController::RobotData& ptr1, BaseController::RobotData &ptr2, BaseController::IntersectionData &ptr3,argos::UInt8 index);
    
    void IntersectionCollisionCheck(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                        std::vector<BaseController::IntersectionData> *ptr3, std::vector<BaseController::IntersectionData> *ptr4,
                                                        argos::UInt8 IntersectionValue, argos::UInt8 index);
    
//    void IntersectionCollisionCheck(argos::CVector2 pt1, argos::CVector2 pt2, BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, BaseController::IntersectionData *ptr3,argos::UInt8 index);
    
//    void Find_Intersection(BaseController::RobotData& ptr1, BaseController::RobotData& ptr2,
//                           BaseController::IntersectionData& ptr3);
//
//    void IntersectionCollisionCheck(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                    BaseController::IntersectionData &ptr3);
    
//    void CheckCollinearity(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2);
    void CheckCollinearity(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
    void AddWayPoint(BaseController::RobotData *ptr);
    
    void CollinearityCollisionCheck(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
//    void CheckRobotHeadingCourse(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                                 BaseController::IntersectionData &ptr3);
    
//    argos::Real CalculateAngleBetweenRobotCourse(BaseController::RobotData& ptr1,
//                                                 BaseController::RobotData &ptr2);
    
//    void AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2,
//                        argos::UInt8 ptrIndex);
//    void AddNewWayPoint(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2);
    
    void AddNewWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
//    void CalculateWaitTime(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2);
    
    argos::Real DistancePointSegment(argos::CVector2 SegPoint1, CVector2 SegPoint2, CVector2 Point);
    
    argos::UInt16 GetTimeToTurn(argos:: Real newAngleToTurnInDegrees, argos::Real speed);
    
//    argos::UInt16 CalculateTotalTime(BaseController::RobotData& ptr, argos::CVector2 TargetPoint, argos::CVector2 StartPoint);
    
//    void AdjustTimeToReachNest(BaseController::RobotData& ptr1, BaseController::RobotData& ptr2);
    
//    argos::CVector2 CalculateTargePoint(BaseController::RobotData& ptr);
    
//    void GetPointAtSafeDistance(BaseController::RobotData& ptr);
    void GetPointAtSafeDistance(BaseController::RobotData *ptr);
    
//    argos::Real CalculateAngleBetweenVectors(CVector3 v1, CVector3 v2);
//    argos::CVector3 GetVectorFromTwoPoints(argos::CVector2 StartPoint, argos::CVector2 EndPoint);
//    argos:: Real ShortestDistTwoVectors(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2);
    argos:: Real ShortestDistTwoVectors(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
    argos::CVector2 CalculatePointAtDistanceAlongVectorDirection(argos::CVector2 Point1, argos::CVector2 Point2, argos::Real Distance);
    
    argos::Real CheckDirection(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
//    argos::CVector2 CalculateWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
    argos::CVector2 CalculateWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, UInt8 turntype);
    
    void IntersectionHandling_ForRobotsGoingOutOfNest_Collinear(BaseController::RobotData *Robot1, BaseController:: RobotData *Robot2,
                                                                                    std::vector<BaseController::IntersectionData> *stRobotIntersectionData1,
                                                                std::vector<BaseController::IntersectionData> *stRobotIntersectionData2);
    void QuadrantGroupType(std::vector<argos::UInt8>* ptr, argos::UInt8 size);
//    argos::UInt8 QuadrantGroupType(std::vector<argos::UInt8>* ptr, argos::UInt8 size);
    
    void TestFunction();
    void GetNeighbors();
    void StopAllRobots();
    void ClearRobotVectorData();
    void CheckComplete();
    void CheckCollisionWithNeighbors(bool CheckOnlyCollinearity);
    void InitializeMatrix(BaseController::RobotData *ptr1, argos::UInt8 dimension);
    void InitializeMatrixElementAndTransformElement(BaseController::RobotData                                                                   *ptr,argos::UInt8 row, argos::UInt8 column,                                                  argos::UInt8 value);
    
    void IntersectionCollisionCheck(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                        BaseController::IntersectionData* Robot1IntersectionData, BaseController::IntersectionData* Robot2IntersectionData,
                                    argos::UInt8 IntersectionValue, argos::UInt8 index);
    
    bool IsMatrixConsistent(BaseController::RobotData *ptr);
    void Avoid_Collision();
    bool ThreePointsCollinear(CVector2 Point1, CVector2 Point2, CVector2 Point3);
    
    argos::CVector2 FindClosestAnchorPoint(BaseController::RobotData *ptr, BaseController::RobotData *Otherptr, bool AtNest);
    
    bool MatrixCollinearityIntersectionEquation(BaseController::RobotData *ptr, argos::UInt8 row_size, argos::UInt8 column_size);
    
    
//    void IntersectionHandlingForCollinearity(BaseController::RobotData *RobotDataptr, BaseController::RobotData *BaseRobotDataptr,
//                                             std::vector<BaseController::IntersectionData> *stRobotIntersectionData, argos::UInt8 CollinearRobotID);
    
    void IntersectionHandlingForCollinearity(BaseController::RobotData *RobotDataptr,
                                                                 std::vector<BaseController::IntersectionData> *stRobotIntersectionData);
    
    void AvoidCollinearCollision(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, BaseController::RobotData *BaseRobotptr,
                                                     std::vector<BaseController::IntersectionData> *ptr3, std::vector<BaseController::IntersectionData> *ptr4);
    
    
    BaseController::IntersectionData* GetIntersectionDataFromVector(std::vector<BaseController::IntersectionData> *ptr,
                                                                                        argos::UInt16 RobotId, argos::UInt8 IntersectionType);
    
    void IntersectionCheckModule(BaseController::RobotData *ptr, BaseController::RobotData *ptr1, BaseController::RobotData *ptr2,
                                                     std::vector<BaseController::IntersectionData> *ptr3,
                                                     std::vector<BaseController::IntersectionData> *ptr4,
                                 argos::UInt8 ptr1_index,argos::UInt8 ptr2_index);
    
    void CheckCollisionAndConsistency();
    
    
    void IntersectionCheck_ForRobots_GoingOutOfNest(BaseController::RobotData *Robot1, BaseController:: RobotData *Robot2,
                                                                        std::vector<BaseController::IntersectionData> *stRobotIntersectionData1,
                                                                        std::vector<BaseController::IntersectionData> *stRobotIntersectionData2,
                                                    argos::UInt8 index_1, argos::UInt8 index_2, BaseController::RobotData *Baseptr);
    
    bool WayPointCollinearityCheck(argos::CVector2 Pt1,argos::CVector2 Pt2 , argos::CVector2 Pt3, argos::CVector2 Pt4,
                                                       BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
    argos::Real ShortestDistTwoSegments(argos::CVector2 Pt1, argos::CVector2 Pt2 , argos::CVector2 Pt3, argos::CVector2 Pt4);
    argos::UInt8 DecideLeftOrRight(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2, argos::Real direction);
    
    void CalculatePointsAlongCircle(argos::CVector2 *Array, BaseController::RobotData *ptr, argos::Real radius, bool CentreNest);
    void CollectiveCollinearCheck();
    void SortforLeftMostRobot(std::vector<argos::UInt8>* ptr, argos::UInt8 size);
    void GetNeighbor_ThisRobot(BaseController::RobotData *ptr);
    
    void CollinearPathPlanning(BaseController::RobotData *stRobotDataThis);
    argos::UInt8 FindRobotGoingToWayPt(std::vector<argos::UInt8>* ptr, argos::UInt8 size);
    bool Check_CollinearVectors(argos::CVector2 Vec1, argos::CVector2 Vec2, argos::CVector2 Vec3, argos::CVector2 Vec4);
    
    void SortRobotForward(std::vector<argos::UInt8>* ptr, argos::UInt8 size, argos::UInt8 ToNestDirection);
    void GetRectangleCoordinates(argos::CVector2 StartPoint, argos::CVector2 EndPoint);
    void ClusterModeOperation();
//    argos::Real CheckDirection(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
//    argos::CVector2 CalculateWayPoint(BaseController::RobotData *ptr1, BaseController::RobotData *ptr2);
    
    
//    argos::CVector2 CalculatePointAtDistanceAlongVectorDirection(argos::CVector2 Point1, argos::CVector2 Point2, argos::Real Distance);
    argos::CVector2 GetWayPointClusterMode(BaseController::RobotData *ptr1);
    public:
    
        std::vector<argos::UInt8> CollinearRobots_GoingAwayFromNest;
        std::vector<argos::UInt8> CollinearRobots_GoingTowardsNest;
        argos::Real MaxArenaDistance;
        argos::UInt8 RoboId1;
        argos::UInt8 RoboId2;
        argos::CVector2 IntersectionPtCopy1;
        argos::CVector2 IntersectionPtCopy2;
        argos::UInt16 Time_1Int;
        argos::UInt16 Time_2Int;
        argos::UInt8 IntValue;
        argos::UInt8 RoboNo;
        argos::UInt16 Timesafe;
        bool RobotWithWaypt;
        argos::UInt8 GoingAwayNestLeft, WaypointType;
        bool FirstCheck;
        argos::UInt8 RobotId;
        argos::UInt8 LeftMostRobotID;
        argos::Real Average_Collision_Avoidance;
        argos::CRandom::CRNG* m_pcRNG;
        argos::UInt16 SimulatorTicksperSec;
        argos::Real distance;
//        argos::UInt16 stoptime1;
//        argos::UInt16 stoptime2;
//        argos::UInt16 TimeTaken;
        argos::UInt16 RobotIDTrial;
        argos:: Real DistanceBetweenRobots;
        std::vector<argos::UInt16>RobotResource;
    
        /* Robot collision data */
        enum COLLISIONDATA {CONSISTENT = 0, COLLINEAR = 1, INTERSECTION1 = 2,
                            INTERSECTION2 = 3, COLLINEAR_INTERSECTION = 4} COLLISION_TYPE;
    
        argos::CVector2 First_QuadrantAnchorPoint;
        argos::CVector2 Second_QuadrantAnchorPoint;
        argos::CVector2 Third_QuadrantAnchorPoint;
        argos::CVector2 Fourth_QuadrantAnchorPoint;
    
        argos::Real AnchorDistance;
        argos::UInt16 RobotResourceSize;
        argos::CVector2 IntersectionPointLF;
        argos::UInt16 IntersectionLoopValue;
        argos::UInt8 IntersectionStructIndex;
    
        argos::UInt8 TestVariable;
        argos::UInt8 TestValue;
        argos::CVector2 TestPoint;
        argos::Real AnchorRadius = 0.54;
        argos::Real DiagonalAnchorPointX = AnchorRadius * argos::Cos(ToRadians(argos::CDegrees(45.0f)));
        argos::Real DiagonalAnchorPointY = AnchorRadius * argos::Sin(ToRadians(argos::CDegrees(45.0f)));
    
    //    argos::CVector2 AnchorPoints[8] = {{0, AnchorRadius}, {AnchorRadius, 0}, {-AnchorRadius, 0}, {0, -AnchorRadius},
    //                                       {DiagonalAnchorPointX, DiagonalAnchorPointY}, {-DiagonalAnchorPointX, -DiagonalAnchorPointY},
    //                                       {-DiagonalAnchorPointX, DiagonalAnchorPointY}, {DiagonalAnchorPointX, -DiagonalAnchorPointY}};
    
        argos::CVector2 AnchorPointsHalfPos[5] =  {{AnchorRadius, 0}, {DiagonalAnchorPointX, DiagonalAnchorPointY}, {0, AnchorRadius},
                                                    {-DiagonalAnchorPointX, DiagonalAnchorPointY}, {0, -AnchorRadius}};

        argos::CVector2 AnchorPointsHalfNeg[5] =  {{AnchorRadius, 0}, {DiagonalAnchorPointX, -DiagonalAnchorPointY}, {0, -AnchorRadius},
                                                    {-DiagonalAnchorPointX, -DiagonalAnchorPointY}, {0, -AnchorRadius}};
    
    //    argos::CVector2 AnchorPoints[8] = {{0, 0.54}, {0.54, 0}, {-0.54, 0}, {0, -0.54}, {0.3818, 0.3818}, {-0.3818, -0.3818}, {-0.3818, 0.3818},
    //                                    {0.3818, -0.3818}};
    
	protected:

	void setScore(double s);

        argos::CRandom::CRNG* RNG;
        bool algoactivated;
        bool speedcheck_flag;
        size_t sim_time;
        size_t ticks_per_second;
        size_t MaxSimTime;
        size_t ResourceDensityDelay;
        size_t RandomSeed;
        size_t SimCounter;
        size_t MaxSimCounter;
        size_t VariableFoodPlacement;
        size_t OutputData;
        size_t DrawDensityRate;
        size_t DrawIDs;
        size_t DrawTrails;
        size_t DrawTargetRays;
        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

        argos::Real WaypointDistance;
        argos::Real Rotation_Angle;
        CVector2 ClusterPos;
		CVector2 NestPosition;
        CVector2 TargetPoint1;
        CVector2 TargetPoint2;
        /* physical robot & world variables */
        argos::Real FoodRadius;
        argos::Real FoodRadiusSquared;
        argos::Real NestRadius;
        argos::Real NestRadiusSquared;
        argos::Real NestElevation;
        argos::Real SearchRadiusSquared;
        argos::Real Neighbor_Radius;
        argos::Real DistanceRobots;
        argos::Real FoodBoundsWidth;
        argos::Real FoodBoundsHeight;
        argos::UInt32 Random_Seed;
        argos::UInt8 TotalRobots;
        /* list variables for food & pheromones */
        std::vector<argos::CVector2> FoodList;

        std::vector<argos::CColor>   FoodColoringList;
        std::vector<argos::CVector2> FidelityList;

        std::vector<argos::CRay3>    TargetRayList;
        std::vector<argos::CColor>   TargetRayColorList;

        argos::CRange<argos::Real>   ForageRangeX;
        argos::CRange<argos::Real>   ForageRangeY;
    
    private:

        /* private helper functions */
        void RandomFoodDistribution();
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();
        void FindClusterLengthWidth();
        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
        bool IsCollidingWithNest(argos::CVector2 p);
        bool IsCollidingWithFood(argos::CVector2 p);
//        void Check_GoingToNest(BaseController::RobotData& ptr1, BaseController::RobotData &ptr2);
    
        BaseController::IntersectionData *sIntersectionDataNextRobot = NULL;
        BaseController::RobotData *sRobotDataprevious = NULL;
        BaseController::RobotData *sRobotDatanext = NULL;
        double score;
        int PrintFinalScore;
        argos::UInt16 Result_Checked;
        argos::UInt16 RobotNumber;
        std::vector <BaseController::RobotData> stRobotDataStruct;
        bool update_movement_state;
        bool RobotReachedWayPoint;
        bool NewWayPointAdded;
        char direction_last;
//        argos::Real MaxLinearSpeed;
        string file_path;
        string file_name;
        string full_path;
        argos::Real MaxLinearSpeed;
        const argos::UInt16 WayPointMaxCount = 1;
        const argos::Real CollinearGap = 0.2;
        const argos::Real Safedistance = 0.5;
//        const argos::Real CollinearDistanceGap = 0.32;
        const argos::Real CollinearDistanceGap = 0.18;

        const argos::Real Searcher_Gap = 0.18;
//        const argos::Real MaxLinearSpeed = 6.0f;
        const argos::Real MinLinearSpeed = 4.0f;
        const argos::Real Robot_Gap_Distance = 0.2f;
        const argos::UInt16 MaximumWaypoint = 5;
        const argos::Real OverlappingCourseAngle = 20.0f;
        argos::CRange<argos::Real>   ForageRangeX_1;
        argos::CRange<argos::Real>   ForageRangeY_1;
        argos::Real FOOTBOT_RADIUS   = 0.085036758f;
        const bool CLUSTERCONFIGONLY = true;
        argos::UInt16 count_start;
        argos::UInt16 count_end;
        argos::UInt16 next_id;
    
    

};

#endif /* DSA_LOOP_FUNCTIONS_H */
