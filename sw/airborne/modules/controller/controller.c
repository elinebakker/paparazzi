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
#include "subsystems/datalink/telemetry.h"

// Define verbose mode which will write messages to the telemetry when activated.
#define CONTROLLER_VERBOSE true

#define PRINT(string,...) fprintf(stderr, "[controller->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if CONTROLLER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Define initial variables
uint8_t safeToGoForwards        = false;
float incrementForAvoidance     = 10.0; // Amount of degrees the drone should turn when stuck
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2; // Max distance the GOAL waypoint is placed ahead (in meters)
uint8_t looping_counter         = 0;
static bool NormalFrameWidth    = true;

void controller_init() {
    VERBOSE_PRINT("Controller initialized.\n");
}

/*
 * Function that calls a vision module to check if it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void controller_periodic() {

    // The looping counter counts the number of calls to controller_periodic.
    looping_counter+=1;
    VERBOSE_PRINT("--------------------------------------------------\n");
    float bestDirection;
    float moveDistance;
    static float total_incrementForAvoidance;


    // Check the vision module whether it is safe to go forwards
    safeToGoForwards = checkIfSafeToGoForwards(NormalFrameWidth);

    if(safeToGoForwards){
        // Find the best direction
        bestDirection = findBestDirection(true);
        VERBOSE_PRINT("The best direction is: %f\n", bestDirection);
        total_incrementForAvoidance= 0.0;
        moveDistance = 0.5;

        // Limits on the max yaw
        if(bestDirection>15.0){
            bestDirection = 15.0;
            moveDistance = 0.0;
        } else if(bestDirection<-15.0){
            bestDirection = -15.0;
            moveDistance = 0.0;
        }

        // Only consider changing direction every 50 iterations (prefent jitter)
        // Exception when the drone is stuck
        if(abs(bestDirection)>1.0 && (looping_counter % 50 == 0 || NormalFrameWidth == false)){
            // Change heading
            VERBOSE_PRINT("Found a better direction, turning %f deg\n",bestDirection);
            int bestdir = (int) bestDirection;

            // Immediately start turning the drone
            increase_nav_heading(&nav_heading, bestdir);

            // Move the waypoints GOAL and TRAJECTORY in this new direction
            moveWaypointForward(WP_GOAL, moveDistance, bestdir);
            moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance, bestdir);

            // Set heading towards the GOAL waypoint
            nav_set_heading_towards_waypoint(WP_GOAL);
        } else {
            // If we are not turning, just move the waypoint forward

            // Estimate how fast we can move forward based on the DetermineTrajectoryConfidence() function of the vision
            // This calculation returns an value between 0.1 and 2.
            moveDistance = fmax(fmin(maxDistance, 10 * DetermineTrajectoryConfidence() - 8),0.1);
            VERBOSE_PRINT("Forward\n");

            // Move the waypoints GOAL and TRAJECTORY
            moveWaypointForward(WP_GOAL, moveDistance, 0.0);
            moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance, 0.0);

            // Set the heading towards the GOAL waypoint
            nav_set_heading_towards_waypoint(WP_GOAL);
        }
        VERBOSE_PRINT("\n");
        NormalFrameWidth=true;
    } else{
        // When it is not safe to move forward
        VERBOSE_PRINT("Pause for a little\n");

        // set waypoints to current horizontal poisiton
        waypoint_set_here_2d(WP_GOAL);
        waypoint_set_here_2d(WP_TRAJECTORY);

        // increase the heading
        increase_nav_heading(&nav_heading, incrementForAvoidance);

        // the total heading change
        total_incrementForAvoidance += incrementForAvoidance;
        VERBOSE_PRINT("TOTAL INCREMENT FOR AVOIDANde %f",total_incrementForAvoidance);

        if (total_incrementForAvoidance>20){
            // Apparently the drone is stuck (it has turned more than 20 degrees without finding a good direction)
            // Therefore we start looking at a smaller region to consider whether it is safe to move forwards.
            NormalFrameWidth = false;
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