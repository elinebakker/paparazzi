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

uint8_t performGroundScan = 0;

// Variables required for control
uint8_t orange_avoider_safeToGoForwards        = false;
float tresholdColorCount                       = 0.80; // As a percentage
float optionMatrix[240][520]; // The values in this matrix should be the sum of the number of pixels which are found to be 'ground' in the rectangle cornered by the x,y position and the top left corner.
float F; // The fuzzy value that determines the groundness per pixel

bool checkIfSafeToGoForwards() {
    orange_avoider_safeToGoForwards = findPercentageGround(0, 90, 210, 310) > tresholdColorCount;
    return orange_avoider_safeToGoForwards;
}

float findBestDirection(void){

    int width_y=520;
    int height_x=240;
    int y_offset_min;
    int y_offset_max;

    int x_offset_min=0;
    int x_offset_max=(height_x/4)*2;


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

    int threshold=150;

    int pixels_center=g_pixels[steps/2];
    float direction_deg=0.0;
    int max_value=0;
    for (int x=0; x<steps; x++){
        if (x!=steps/2 && g_pixels[x]>max_value && (g_pixels[x]-threshold)>pixels_center){
            max_value=g_pixels[x];
            direction_deg=(float) -20.0+((40.0/(steps-1))*x);

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
    if(performGroundScan) {
        createHistogram(input_img);
        performGroundScan = 0;
    }
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
            //VERBOSE_PRINT("%d:%d,%d,%d\n",x,Y,U,V);
            // Format is UYVY
            //R = 1.164(Y - 16) + 1.596(V - 128);
            //G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128);
            //B = 1.164(Y - 16)                   + 2.018(U - 128);
            // VERBOSE_PRINT("%d >= %d",source[1],color_lum_min);
            // Check if the color is inside the specified values

                //VERBOSE_PRINT("x=%d",x);
                //pixel_sum=optionMatrix[x-1][y]+optionMatrix[x][y-1]-optionMatrix[x-1][y-1]+1
                //VERBOSE_PRINT("(%d,%d)\n",x,y);
            F=getFuzzyValue(Y,U,V);
            optionMatrix[x][y] +=F;
            float fuzzy_threshold = 0.8;

            if (F > fuzzy_threshold) {
                // Change color of the ground pixels
                if(x % 2 == 0) {
                    //source[1] = 0;
                } else {
                    source[0] = 88;        // U
                    source[2] = 255;        // V
                }
            }

            if(x % 2 == 1 && x < (input_img->w-1)) {
                // Go to the next pixel (2 bytes)
                source += 4;
            }
        }
    }


    return input_img;
}

float getFuzzyValue(int Y, int U, int V) {
    int rampLength = 20; // Set fuzzy rampLength
    // Determine Y bounds
    int color_lum_min_lower = color_lum_min - rampLength / 2; // Determine upper and lower bounds per value.
    int color_lum_min_upper = color_lum_min + rampLength / 2;
    int color_lum_max_lower = color_lum_max - rampLength / 2; // Determine upper and lower bounds per value.
    int color_lum_max_upper = color_lum_max + rampLength / 2;

    // Determine U bounds
    int color_cb_min_lower = color_cb_min - rampLength / 2; // Determine upper and lower bounds per value.
    int color_cb_min_upper = color_cb_min + rampLength / 2;
    int color_cb_max_lower = color_cb_max - rampLength / 2; // Determine upper and lower bounds per value.
    int color_cb_max_upper = color_cb_max + rampLength / 2;

    // Determine V bounds
    int color_cr_min_lower = color_cr_min - rampLength / 2; // Determine upper and lower bounds per value.
    int color_cr_min_upper = color_cr_min + rampLength / 2;
    int color_cr_max_lower = color_cr_max - rampLength / 2; // Determine upper and lower bounds per value.
    int color_cr_max_upper = color_cr_max + rampLength / 2;


    // Determine Fuzzy Y value
    Y = 1; //  When Y is larger than the min_upper bound or lower than the max_lower bound.
    if (Y > color_lum_min_lower && Y < color_lum_min_upper) { // If Y on fuzzy ramp around the min value
        Y = 1 / color_lum_min * (color_lum_min - color_lum_min_lower); // Assign a value from 0 to 1

    } else if (Y > color_lum_max_lower && Y < color_lum_max_upper) { // If Y on fuzzy ramp around the max value
        Y = 1 - (1 / color_lum_min * (color_lum_min - color_lum_min_lower)); // Assign a value from 1 to 0.
    }

    (Y <= color_lum_min_lower) ? 0 : Y; // If Y is smaller than the lowest bound, assign 0, otherwise remain old value.
    (Y >= color_lum_max_upper) ? 0 : Y; // If Y is larger than the highest bound, assign 0, otherwise remain old value.

    // Determine Fuzzy U value
    U = 1; //  When U is larger than the min_upper bound or lower than the max_lower bound.
    if (U > color_cb_min_lower && U < color_cb_min_upper) { // If U on fuzzy ramp around the min value
        U = 1 / color_cb_min * (color_cb_min - color_cb_min_lower); // Assign a value from 0 to 1

    } else if (U > color_cb_max_lower && U < color_cb_max_upper) { // If U on fuzzy ramp around the max value
        U = 1 - (1 / color_cb_min * (color_cb_min - color_cb_min_lower)); // Assign a value from 1 to 0.
    }

    (U <= color_cb_min_lower) ? 0 : U; // If U is smaller than the lowest bound, assign 0, otherwise remain old value.
    (U >= color_cb_max_upper) ? 0 : U; // If U is larger than the highest bound, assign 0, otherwise remain old value.

    // Determine Fuzzy V value
    V = 1; //  When V is larger than the min_upper bound or lower than the max_lower bound.
    if (V > color_cr_min_lower && V < color_cr_min_upper) { // If V on fuzzy ramp around the min value
        V = 1 / color_cr_min * (color_cr_min - color_cr_min_lower); // Assign a value from 0 to 1

    } else if (V > color_cr_max_lower && V < color_cr_max_upper) { // If V on fuzzy ramp around the max value
        V = 1 - (1 / color_cr_min * (color_cr_min - color_cr_min_lower)); // Assign a value from 1 to 0.
    }

    (V <= color_cr_min_lower) ? 0 : V; // If V is smaller than the lowest bound, assign 0, otherwise remain old value.
    (V >= color_cr_max_upper) ? 0 : V; // If V is larger than the highest bound, assign 0, otherwise remain old value.

    F = Y*U*V;
    return F;
}

float findPercentageGround(int x_min, int x_max, int y_min, int y_max){
    int sum = 0;
    float percentage = 0.0;

    /*for (int col=510; col<=519; col++)
    {
        for(int row=230; row<=239; row++)
            printf("%d  ", optionMatrix[row][col]);
        printf("\n");
    }*/

    //VERBOSE_PRINT("%i",optionMatrix[1][1]);
    sum =  optionMatrix[x_max][y_max]-optionMatrix[x_min][y_max]-optionMatrix[x_max][y_min]+optionMatrix[x_min][y_min];
    //VERBOSE_PRINT("%d-%d-%d+%d=%d",optionMatrix[x_max][y_max],optionMatrix[x_min][y_max],optionMatrix[x_max][y_min],optionMatrix[x_min][y_min],sum);
    percentage = sum/((x_max-x_min)*(y_max-y_min)*1.0);
    VERBOSE_PRINT("Ground percentage %f\n",percentage);
    return percentage;
}

void updateGroundFilterSettings(){
    // This function should determine the color of the ground and save this in the following variables.
    // Called right before start of the obstacle course
    performGroundScan = 1;

}

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
// # Function to find the maximum element of a given array; in our case the element number is the corresponding intensity for the channel #
    int find_max(int a[], int n) {
      int c, max, index;
      // int n;
      //n = sizeof(a)/sizeof(a[0]);
      //VERBOSE_PRINT("%d\n",n);
      max = a[0];
      index = 0;
     
      for (c = 1; c < n; c++) {
        if (a[c] > max) {
           index = c;
           max = a[c];
        }
      }
      return index;
    }
    

//Limit calculation
int * find_limits(int a[], int n, float margin){
    int i, id, range = 0, sum = 0;
    // float margin = 0.9;                     //The percentage of values desired to be within the limits
    
    id = find_max(a,n);
    int lowerlim = id;
    int upperlim = id;

    for(i=0;i<n;i++){
        sum += a[i];
    }
    float ratio = 0.0;
    while(ratio<margin){                    //Runs until the ratio is achieved
        
        if(a[lowerlim-1]<a[upperlim+1]){
            if(upperlim ==n){upperlim=n-1;} //So that the upper limit remains in bounds
            upperlim += 1;
        }else if(a[lowerlim-1]>a[upperlim+1]){
            if(lowerlim ==0){lowerlim=1;}   //So that lower limit remains in bounds
            lowerlim -= 1;
        }else{upperlim +=1;}
        

        range = 0;
        for(i=lowerlim; i<=(upperlim);i++){
            range += a[i];
        }
        //VERBOSE_PRINT("l:%d u:%d r:%d s:%d n:%d %f\n",lowerlim, upperlim, range, sum, n, ratio);
    ratio = range/(float)sum;       
    }

    static int results[2];
    results[0] = lowerlim;
    results[1] = upperlim;
    return results;
    // Use int *results;  results =find_limits(a,n); To get the limits in an another function
}

// #    This section generates histogram for YUV channels # 
// # This function generates the Histogram in form of an array #
    struct image_t *createHistogram(struct image_t *input_img) //change output struct to whatever we will have
    {
        int histo_y[256] = {0};
        int histo_u[256] = {0};
        int histo_v[256] = {0};
        int ymax, ymin, vmax, vmin, umax, umin;
        // float margin = 0; // margin tolerances for the intensity

        uint8_t *source = input_img->buf;  // Go trough all the pixels
        // VERBOSE_PRINT("h:%d , w:%d",input_img->h, input_img->w);
        for (uint16_t y = 0; y < (input_img->h); y++) {
            for (uint16_t x = 0; x < (input_img->w); x += 2) {

                histo_y[source[1]] += 1; //Store Y value in histogram
                histo_y[source[4]] += 1; //Store Y value in histogram
                histo_u[source[0]] += 1; //Store U value in histogram
                histo_v[source[2]] += 1; //Store V value in histogram

                // Go to the next pixel (2 bytes)
                source += 4;
            }
        }
        // Ranges for 3 channels: Y,U,V
        int *results;
        int n = 256;
        float margin = 0.9;
        results = find_limits(histo_y, n, 0.99);
        ymax = results[1];
        ymin = results[0];

        results = find_limits(histo_v, n, 0.95);
        vmax = results[1];
        vmin = results[0];

        results = find_limits(histo_u, n, 0.95);
        umax = results[1];
        umin = results[0];
        VERBOSE_PRINT("\n Y range %d to %d\n U range %d to %d\n V range %d to %d\n", ymin, ymax, umin, umax, vmin,
                      vmax);

        color_lum_min = ymin;
        color_lum_max = ymax;
        color_cb_min  = umin;
        color_cb_max  = umax;
        color_cr_min  = vmin;
        color_cr_max  = vmax;

        performGroundScan = 0;
        return input_img;
        // return ymax, ymin, umin, umax, vmin, vmax; //change output
    }

void ground_follower_init()
{
    // Initialise the settings of the ground filter
    color_lum_min = 37;
    color_lum_max = 209;
    color_cb_min  = 47;
    color_cb_max  = 110;
    color_cr_min  = 123;
    color_cr_max  = 173;

    // Apply the calculateOptionMatrix function to the images generated by the video camera
    listener = cv_add_to_device(&GROUND_FOLLOWER_CAMERA, calculateOptionMatrix);
}