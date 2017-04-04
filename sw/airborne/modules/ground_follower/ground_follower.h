/*
 * Copyright (C) Geart van Dam
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ground_follower/ground_follower.h"
 * @author Group02
 * Identify the ground and follow it
 */

#ifndef GROUND_FOLLOWER_H
#define GROUND_FOLLOWER_H
#include <inttypes.h>
#include "state.h"
#include "modules/computer_vision/lib/vision/image.h"

/* The documentation can be found in the source file*/

// Variables
extern uint8_t color_lum_min;
extern uint8_t color_lum_max;
extern uint8_t color_cb_min;
extern uint8_t color_cb_max;
extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern float conf_vision_init_y;
extern float conf_vision_init_u;
extern float conf_vision_init_v;

extern int conf_steps;
extern int conf_baseThreshold;
extern float conf_thresholdSlope;
extern float conf_vision_directbox_height;

extern int conf_vision_fuzzy_ramp_y;
extern int conf_vision_fuzzy_ramp_u;
extern int conf_vision_fuzzy_ramp_v;
extern float conf_vision_safeToGoForwards_threshold;

extern uint8_t performGroundScan;
extern uint8_t orange_avoider_safeToGoForwards;


// Functions
extern bool checkIfSafeToGoForwards(bool);
void drawRectangle(struct image_t*, int, int, int, int);
extern void ground_follower_init(void);
extern struct image_t *calculateOptionMatrix(struct image_t *input_img);
extern float getFuzzyValue(int Y, int U, int V);
extern struct image_t *createHistogram(struct image_t *input_img);
float findPercentageGround(int x_min, int x_max, int y_min, int y_max);
float findBestDirection(bool);
void updateGroundFilterSettings(void);
int find_max(int a[], int n);
int * find_limits(int a[], int n, float margin);
float DetermineTrajectoryConfidence(void);

#endif
