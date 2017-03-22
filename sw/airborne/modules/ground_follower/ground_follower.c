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
float tresholdColorCount                       = 0.75; // As a percentage
uint16_t optionMatrix[240][520]; // The values in this matrix should be the sum of the number of pixels which are found to be 'ground' in the rectangle cornered by the x,y position and the top left corner.

bool checkIfSafeToGoForwards(){
    orange_avoider_safeToGoForwards = findPercentageGround(0, 60, 174 , 346) > tresholdColorCount;
    return orange_avoider_safeToGoForwards;
}

float findBestDirection(void){

    int width_y=520;
    int height_x=240;
    int y_offset_min;
    int y_offset_max;

    int x_offset_min=0;
    int x_offset_max=height_x/2;


    int steps=5;
    int g_pixels[steps-1];
    int pixel_step=width_y/steps;
    if ( pixel_step % 2 != 0 ){
        pixel_step=pixel_step+1;
    }
    for (int y = 0; y < steps; y++) {

        //first set frame
        y_offset_min=pixel_step*y;
        y_offset_max=pixel_step*(y+1);

        if (y_offset_max>518){
            y_offset_max=518;
        }


        g_pixels[y]=optionMatrix[x_offset_max][y_offset_max]-optionMatrix[x_offset_min][y_offset_max]-optionMatrix[x_offset_max][y_offset_min]+optionMatrix[x_offset_min][y_offset_min];


    }

    int threshold=-250;

    int pixels_center=g_pixels[steps/2];
    float direction_deg=0.0;
    int max_value=0;
    for (int x=0; x<steps; x++){
        if (x!=steps/2 && g_pixels[x]>max_value && (g_pixels[x]-threshold)>pixels_center){
            max_value=g_pixels[x];
            direction_deg=(float) -30.0+((60.0/steps)*x);

        }


    }




    return direction_deg;
}



/*
float findBestDirection(void){
    // bottom left
    int x_offset_min_l;
    int x_offset_max_l;
    int y_offset_min_l;
    int y_offset_max_l;

    x_offset_min_l=0;
    x_offset_max_l=120;
    y_offset_min_l=0;
    y_offset_max_l=173;

    //bottom middle
    int x_offset_min_m;
    int x_offset_max_m;
    int y_offset_min_m;
    int y_offset_max_m;

    x_offset_min_m=0;
    x_offset_max_m=120;
    y_offset_min_m=174;
    y_offset_max_m=346;

    //bottom right
    int x_offset_min_r;
    int x_offset_max_r;
    int y_offset_min_r;
    int y_offset_max_r;

    x_offset_min_r=0;
    x_offset_max_r=120;
    y_offset_min_r=347;
    y_offset_max_r=519;

    //calculating number of green pixels
    int g_pixels_left;
    int g_pixels_middle;
    int g_pixels_right;
    float direction_deg;

    g_pixels_left=optionMatrix[x_offset_max_l][y_offset_max_l]-optionMatrix[x_offset_min_l][y_offset_max_l]-optionMatrix[x_offset_max_l][y_offset_min_l]+optionMatrix[x_offset_min_l][y_offset_min_l];
    g_pixels_middle=optionMatrix[x_offset_max_m][y_offset_max_m]-optionMatrix[x_offset_min_m][y_offset_max_m]-optionMatrix[x_offset_max_m][y_offset_min_m]+optionMatrix[x_offset_min_m][y_offset_min_m];
    g_pixels_right=optionMatrix[x_offset_max_r][y_offset_max_r]-optionMatrix[x_offset_min_r][y_offset_max_r]-optionMatrix[x_offset_max_r][y_offset_min_r]+optionMatrix[x_offset_min_r][y_offset_min_r];


    printf("pixels left %i ",g_pixels_left);
    printf("pixels middel%i ",g_pixels_middle);
    printf("pixels right %i \n",g_pixels_right);

    int threshold=500;

    if(g_pixels_left>g_pixels_right && g_pixels_left-g_pixels_middle>threshold) {
        direction_deg = -10.0;

    }else if (g_pixels_left<g_pixels_right && g_pixels_right-g_pixels_middle>threshold){
        direction_deg = 10.0;

    } else {
        direction_deg = 0.0;
    }

    return direction_deg;

}
*/

struct image_t *calculateOptionMatrix(struct image_t *input_img)
{
    uint8_t *source = input_img->  buf;  // Go trough all the pixels
    // VERBOSE_PRINT("h:%d , w:%d",input_img->h, input_img->w);
    for (uint16_t y = 0; y < (input_img->h); y++) {
        for (uint16_t x = 0; x < (input_img->w); x += 1) {
            int Y,U,V;
            int R,G,B;
            //optionMatrix[x][y] = 0;
            // Calculate the running sum of pixels from the top left corner till this pixel
            if(y>0 && x>0){// This is true for all except the top and left border
                optionMatrix[x][y] = optionMatrix[x-1][y]+optionMatrix[x][y-1]-optionMatrix[x-1][y-1];
            }else if(x>0) { // Top border
                optionMatrix[x][y] = optionMatrix[x-1][y];
            } else if(y>0){ // Left border
                optionMatrix[x][y] = optionMatrix[x][y-1];
            } else { // Top left pixel
                optionMatrix[x][y] = 0;
            }
            V = source[2];
            U = source[0];
            if(x%2 == 0){
                // Even pixels
                Y = source[1];
            } else {
                // Odd pixels
                Y = source[3];
            }

            // Format is UYVY
            //R = 1.164(Y - 16) + 1.596(V - 128);
            //G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128);
            B = 1.164(Y - 16)                   + 2.018(U - 128);
            // VERBOSE_PRINT("%d >= %d",source[1],color_lum_min);
            // Check if the color is inside the specified values
            if (
                (Y >= color_lum_min)
                && (Y <= color_lum_max)
                && (U >= color_cb_min)
                && (U <= color_cb_max)
                && (V >= color_cr_min)
                && (V <= color_cr_max)
                ) {
                // If this pixel is found to be 'ground-like', using the check above a value of 1 is added to the sum at this point
                //pixel_sum=optionMatrix[x-1][y]+optionMatrix[x][y-1]-optionMatrix[x-1][y-1]+1
                //VERBOSE_PRINT("(%d,%d)\n",x,y);
                optionMatrix[x][y] +=1;
                // Change color of the ground pixels
                source[0] = 88;        // U
                source[2] = 255;        // V

            }
            if(x%2 == 0) {
                // Go to the next pixel (2 bytes)
                source += 4;
            }
        }
    }


    return input_img;
}

float findPercentageGround(int x_min, int x_max, int y_min, int y_max){
    int sum = 0;
    float percentage = 0.0;

/*    for (int col=510; col<=519; col++)
    {
        for(int row=230; row<=239; row++)
            printf("%d  ", optionMatrix[row][col]);
        printf("\n");
    }*/

    //VERBOSE_PRINT("%i",optionMatrix[1][1]);
    sum =  optionMatrix[x_max][y_max]-optionMatrix[x_min][y_max]-optionMatrix[x_max][y_min]+optionMatrix[x_min][y_min];
    VERBOSE_PRINT("%d-%d-%d+%d=%d",optionMatrix[x_max][y_max],optionMatrix[x_min][y_max],optionMatrix[x_max][y_min],optionMatrix[x_min][y_min],sum);
    percentage = sum/((x_max-x_min)*(y_max-y_min));
    VERBOSE_PRINT("Ground percentage %f\n",percentage);
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
    color_lum_min = 30;
    color_lum_max = 208;
    color_cb_min  = 47;
    color_cb_max  = 156;
    color_cr_min  = 114;
    color_cr_max  = 150;

    // Apply the calculateOptionMatrix function to the images generated by the video camera
    listener = cv_add_to_device(&GROUND_FOLLOWER_CAMERA, calculateOptionMatrix);
}




