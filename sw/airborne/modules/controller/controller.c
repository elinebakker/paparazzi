/*
 * Copyright (C) Group2
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/controller/controller.c"
 * @author Group2
 * This module will try to fly as far as possible within a certain obstacle zone.
 */
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "generated/flight_plan.h"
//#include "modules/computer_vision/colorfilter.h"
//#include "modules/controller/controller.h"

/* Include Vision Modules*/
//#include "modules/orange_avoider/orange_avoider.c"

// Define verbose mode which will write messages to the telemetry when activated.
#define CONTROLLER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[controller->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if CONTROLLER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Define initial variables
uint8_t safeToGoForwards        = false;
float incrementForAvoidance     = 5.0;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2;

void controller_init() {
    VERBOSE_PRINT("Controller initialized.\n");
}
uint8_t looping_counter = 0;

/*
 * Function that calls a vision module to check if it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void controller_periodic() {

    looping_counter+=1;
    VERBOSE_PRINT("--------------------------------------------------\n");
    float bestDirection;
    safeToGoForwards = checkIfSafeToGoForwards();
    float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
    if(safeToGoForwards){
            bestDirection = findBestDirection();
            VERBOSE_PRINT("The best direction is: %f\n", bestDirection);
        if(abs(bestDirection)>1.0 && looping_counter % 10 == 0){
            VERBOSE_PRINT("Found a better direction, turning %f deg\n",bestDirection);
            int bestdir = (int) bestDirection;
            increase_nav_heading(&nav_heading, bestdir);
            moveWaypointForward(WP_GOAL, moveDistance, bestdir);
            moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance, bestdir);
            nav_set_heading_towards_waypoint(WP_GOAL);
        } else {
            VERBOSE_PRINT("Forward\n");
            moveWaypointForward(WP_GOAL, moveDistance, 0.0);
            moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance, 0.0);
            nav_set_heading_towards_waypoint(WP_GOAL);
            trajectoryConfidence += 1;
        }
        VERBOSE_PRINT("\n");
    } else{
        VERBOSE_PRINT("Pause for a little\n");
        waypoint_set_here_2d(WP_GOAL);
        waypoint_set_here_2d(WP_TRAJECTORY);
        increase_nav_heading(&nav_heading, incrementForAvoidance);
        if(trajectoryConfidence > 5){
            trajectoryConfidence -= 4;
        }
        else{
            trajectoryConfidence = 1;
        }
    }
    return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
    struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
    int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
    // Check if your turn made it go out of bounds...
    INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
    *heading = newHeading;
    //VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
    return false;
}

/*
 * Perform a 360 degree spin. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t perform_scan(int degree)
{
    increase_nav_heading(&nav_heading,degree);
/*    for(int angle;angle<=180;angle +=5) {
        increase_nav_heading(&nav_heading, angle);
    }*/
    return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, float angle)
{
    struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
    struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
    // Calculate the sine and cosine of the heading the drone is keeping
    float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)+angle);
    float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)+angle);
    // Now determine where to place the waypoint you want to go to
    new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
    new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
    //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
    return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
    VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
    waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
    return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters, float angle)
{
    struct EnuCoor_i new_coor;
    calculateForwards(&new_coor, distanceMeters, angle);
    moveWaypoint(waypoint, &new_coor);
    return false;
}

