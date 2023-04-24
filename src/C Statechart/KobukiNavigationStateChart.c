/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>


// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								// Drive straight
	TURN_LEFT,
	TURN_RIGHT,
	TURN_HILL,
	U_TURN,
	TURN_OBSTACLE,
	TURN_TO_HILL,
	STOP

} robotState_t;

#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree

void KobukiNavigationStatechart(
	const int16_t 				maxWheelSpeed,
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed,
	const bool					isSimulator
	){

	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg
	static int32_t				zAccelAtStart = 1;
	static int32_t				hillHeading = 0;
	static bool					cliffDetected = false;
	static bool					obstacleDetected = false;
	static bool					onHill = false;
	static bool					completeHillHead = false;
	static double_t				ONHILL_CONST = 0.05;
	static double_t				HILL_DIR_CONST = 0.01;

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************

	cliffDetected = sensors.cliffCenter || sensors.cliffLeft || sensors.cliffRight;
	obstacleDetected = sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpRight || sensors.bumps_wheelDrops.bumpCenter;
	onHill = sqrt((accelAxes.x)*(accelAxes.x) + (accelAxes.y)*(accelAxes.y)) >= ONHILL_CONST;

	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		){
		switch (state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************
	//else if (state = ON_HILL sensors.cliffCenter || sensors.cliffLeft || sensors.cliffRight) {

	else if (state == DRIVE && cliffDetected){
		state = TURN_HILL;
	}

	else if (state == TURN_HILL) {
		if (sensors.cliffLeft) {
			state = TURN_RIGHT;
		}
		else if (sensors.cliffRight) {
			state = TURN_LEFT;
		}
		else if (sensors.cliffCenter){
			state = U_TURN;
		}
	}

	else if (state == DRIVE && obstacleDetected) {
		state = TURN_OBSTACLE;
	}

	else if (state == TURN_OBSTACLE) {
		if (sensors.bumps_wheelDrops.bumpLeft) {
			state = TURN_RIGHT;
		}
		else if (sensors.bumps_wheelDrops.bumpRight) {
			state = TURN_LEFT;
		}
		else if (sensors.bumps_wheelDrops.bumpCenter) {
			state = TURN_RIGHT;
		}
	}

	else if (state == TURN_RIGHT && abs(netAngle - angleAtManeuverStart) >= 35) {
		state = DRIVE;
	}

	else if (state == TURN_LEFT && abs(netAngle - angleAtManeuverStart) >= 35) {
		state = DRIVE;
	}

	else if (state == U_TURN && abs(netAngle - angleAtManeuverStart) >= 180) {
		state = DRIVE;
	}

	/*else if (state == DRIVE_HILL && abs(zAccelAtStart - accelAxes.z) < 0.2) {
		state = U_TURN;
	}

	else if (state == U_TURN && abs(netAngle - angleAtManeuverStart) >= 180) {
		state = DRIVE;
	}*/

	// Check if on hill
	else if (state == DRIVE && onHill && !completeHillHead) {
		state = TURN_TO_HILL;
	}

	// Turn towards hill
	else if (state == TURN_TO_HILL && (accelAxes.x > (-1.0) * (HILL_DIR_CONST)) && (accelAxes.x < HILL_DIR_CONST) && (accelAxes.y > -0.03)) {
		//hillHeading = sensors.Angle;
		completeHillHead = true;
		state = DRIVE;
	}

	else if (state == DRIVE && (abs(netDistance - distanceAtManeuverStart) > 200) && completeHillHead) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN_TO_HILL;
		//if (abs(netDistance - distanceAtManeuverStart) > 5) {
		//	state = STOP;
		//}

		//if (completeHillHead && (netDistance - distanceAtManeuverStart) > 5) {
		//	state = TURN_TO_HILL;
		//}
	}

	//else if (state == TURN && abs(netAngle - angleAtManeuverStart) >= 90){
	//	angleAtManeuverStart = netAngle;
	//	distanceAtManeuverStart = netDistance;
	//	state = DRIVE;
	//}
	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch (state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 20;
		break;

	case TURN_RIGHT:
		leftWheelSpeed = 15;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN_LEFT:
		rightWheelSpeed = 15;
		leftWheelSpeed = -rightWheelSpeed;
		break;

	case U_TURN:
		leftWheelSpeed = 15;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN_TO_HILL:
		leftWheelSpeed = 15;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case STOP:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}


	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
