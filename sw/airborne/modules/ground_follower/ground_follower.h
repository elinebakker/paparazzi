/*
 * Copyright (C) Geart van Dam
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ground_follower/ground_follower.h"
 * @author Geart van Dam
 * Identify the ground and follow it
 */

#ifndef GROUND_FOLLOWER_H
#define GROUND_FOLLOWER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;
extern uint8_t color_cb_min;
extern uint8_t color_cb_max;
extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint8_t orange_avoider_safeToGoForwards;

extern void ground_follower_init(void);

extern bool checkIfSafeToGoForwards(void);

extern struct image_t *calculateOptionMatrix(struct image_t *input_img);

extern struct image_t *createHistogram(struct image_t *input_img);

float findPercentageGround(int x_min, int x_max, int y_min, int y_max);

float findBestDirection(void);

void updateGroundFilterSettings(void);

int find_max(int a[], int n);

int * find_limits(int a[], int n, float margin);

#endif

