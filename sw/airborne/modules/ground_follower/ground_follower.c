/*
 * Copyright (C) Geart van Dam
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ground_follower/ground_follower.c"
 * @author Geart van Dam
 * Identify the ground and follow it
 */

#include "modules/ground_follower/ground_follower.h"

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "modules/computer_vision/lib/vision/image.h"

#define GROUND_FOLLOWER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ground_follower->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_FOLLOWER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct video_listener *listener = NULL;

// Variables related to the recognition of the ground floor
uint8_t color_lum_min = 0;
uint8_t color_lum_max = 0;
uint8_t color_cb_min = 0;
uint8_t color_cb_max = 0;
uint8_t color_cr_min = 0;
uint8_t color_cr_max = 0;

// Variables required for control
uint8_t orange_avoider_safeToGoForwards        = false;
int bestDirection                              = 0;
float tresholdColorCount                       = 0.90; // As a percentage
int optionMatrix[520][240]; // The values in this matrix should be the sum of the number of pixels which are found to be 'ground' in the rectangle cornered by the x,y position and the top left corner.
int optionMatrix2[520][240];

bool checkIfSafeToGoForwards(){
    orange_avoider_safeToGoForwards = findPercentageGround(130, 390, 160 , 240) > tresholdColorCount;
    return orange_avoider_safeToGoForwards;
}

float findBestDirection(){
    // Return a direction between -25 and + 25 degree (from current heading) that should provide a better track. Return 0.0 otherwise (keep current heading).
    return 0.0;
}

struct image_t *calculateOptionMatrix(struct image_t *input_img)
{
    uint8_t *source = input_img->  buf;  // Go trough all the pixels

    int pixel_sum=0;

    for (uint16_t y = 0; y < input_img->h; y++) {
        for (uint16_t x = 0; x < input_img->w; x += 2) {


            // Calculate the running sum of pixels from the top left corner till this pixel
            if(y>0 && x>0){// This is true for all except the top and left border
                optionMatrix[x][y] = optionMatrix[x-2][y]+optionMatrix[x][y-1]-optionMatrix[x-2][y-1];
            }else if(x>0) { // Top border
                optionMatrix[x][y] = optionMatrix[x-2][y];
            } else if(y>0){ // Left border
                optionMatrix[x][y] = optionMatrix[x][y-1];
            } else { // Top left pixel
                optionMatrix[x][y] = 0;
            }
            // Check if the color is inside the specified values
            if (
                (source[1] >= color_lum_min)
                && (source[1] <= color_lum_max)
                && (source[0] >= color_cb_min)
                && (source[0] <= color_cb_max)
                && (source[2] >= color_cr_min)
                && (source[2] <= color_cr_max)
                ) {
                // If this pixel is found to be 'ground-like', using the check above a value of 1 is added to the sum at this point
                //pixel_sum=optionMatrix[x-1][y]+optionMatrix[x][y-1]-optionMatrix[x-1][y-1]+1

                optionMatrix[x][y] +=1;

            }
            // Go to the next 2 pixels
            source += 2;
        }
    }

    //per rij hier dus anders
    VERBOSE_PRINT("values bblabalbalbalablablabalbal1 %d\n",optionMatrix[0][52]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal2 %d\n",optionMatrix[0][53]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal3 %d\n",optionMatrix[0][54]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal4 %d\n",optionMatrix[0][55]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal5 %d\n",optionMatrix[1][52]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal6 %d\n",optionMatrix[1][53]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal7 %d\n",optionMatrix[1][54]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal8 %d\n",optionMatrix[1][55]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal9 %d\n",optionMatrix[2][52]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal10 %d\n",optionMatrix[2][53]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal11 %d\n",optionMatrix[2][54]);
    VERBOSE_PRINT("values bblabalbalbalablablabalbal12 %d\n",optionMatrix[2][55]);



    return input_img;
}

float findPercentageGround(int x_min, int x_max, int y_min, int y_max){
    int sum = 0;
    float percentage = 0.0;
    //VERBOSE_PRINT("%i",optionMatrix[1][1]);
    sum =  optionMatrix[x_max][y_max]-optionMatrix[x_min][y_max]-optionMatrix[x_max][y_min]+optionMatrix[x_min][y_min];
    percentage = sum/((x_max-x_min)*(y_max-y_min));
    //VERBOSE_PRINT("Ground percentage %f\n",percentage);
    return percentage;
}

void updateGroundFilterSettings(){
    // This function should determine the color of the ground and save this in the following variables.
    // Called right before start of the obstacle course
    color_lum_min = 44;
    color_lum_max = 71;
    color_cb_min  = 87;
    color_cb_max  = 116;
    color_cr_min  = 126;
    color_cr_max  = 144;

}

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void ground_follower_init()
{
    // Initialise the settings of the ground filter
    color_lum_min = 44;
    color_lum_max = 71;
    color_cb_min  = 87;
    color_cb_max  = 116;
    color_cr_min  = 126;
    color_cr_max  = 144;

    // Apply the calculateOptionMatrix function to the images generated by the video camera
    listener = cv_add_to_device(&GROUND_FOLLOWER_CAMERA, calculateOptionMatrix);
}