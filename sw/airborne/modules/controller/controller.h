/*
 * Copyright (C) Group2
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/controller/controller.h"
 * @author Group2
 * This module will try to fly as far as possible within a certain obstacle zone.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <inttypes.h>
#include "state.h"

/* The documentation can be found in the source file*/

// Variables
extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern float maxDistance;
extern uint16_t trajectoryConfidence;

// Functions
extern void controller_init(void);
extern void controller_periodic(void);
extern uint8_t moveWaypointForward(uint8_t, float, float);
extern uint8_t calculateForwards(struct EnuCoor_i *, float, float);
extern uint8_t moveWaypoint(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading(int32_t *, float);

#endif

