#DDSA-ARGoS

This is a fork of the iAnt-ARGoS code. It contains the DDSA (Distributed Deterministic Spider Algorithm).

To install ARGoS follow the instructions here: http://www.argos-sim.info

Compile the DDSA simulation by running the build.sh script in the root directory of the repository.

A video of the DDSA running can be found at: https://www.youtube.com/watch?v=lloXmwfT4aE

hanges in the DDSA code for comparing its comparison to PPCA-CCPFA approach:

1. Nest radius changed to 4 cm
2. Robot rotation Speed changed to 10 cm/s
3. Ticks per second changed to 48

In DDSA.xml, Section params -> settings
1. The parameter CircuitToActivateAlgo is used to activate the PPCA-CCPFA algorithm. The value specifies the number of spirals each robot should complete before activating the PPCA-CCPFA algorithm
2. TargetStepsToActivateAlgo is -1 if PPCA-CCPFA algorithm needs to be activated after specific number of spirals.
The value is other than -1 if robots should complete specific number of steps - North, South, West, East directions before activating the PPCA-CCPFA algorithm. You ca specify specific distance using TargetStepsToActivateAlgo parameter. The distance travelled is calculated using the formula: 8cm *  TargetStepsToActivateAlgo as each robot travels 8cm per cycle.

In DDSA.xml files, Section DDSA:
1. Change the cluster position by changing the coordinates for parameter "ClusterCenter"
2. Change the waypoint angle through the parameter RotationAngle 
3. Change the waypoint distance through the parameter WaypointDist
4. Change the radius for finding the neighbors of the robot through the parameter NeighborRadius
5. Change the path for storing the log files using ResultsPath 

The configuration parameters can be changed in the experiment file : DDSA.xml

Compile the DDSA simulation by running the build.sh script in the root directory of the repository.
Run the code using ./run.sh
