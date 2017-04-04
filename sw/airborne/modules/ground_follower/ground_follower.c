/*
 * Copyright (C) Geart van Dam
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/ground_follower/ground_follower.c"
 * @author Group 02
 * Identify the ground and follow it
 */

#include "modules/ground_follower/ground_follower.h"

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "state.h"
#include "boards/bebop.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/computer_vision/lib/isp/libisp.h"

#define GROUND_FOLLOWER_VERBOSE true

#define PRINT(string,...) fprintf(stderr, "[ground_follower->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_FOLLOWER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct video_listener *listener = NULL;

// Minimum and maximum values used for the recognition of the ground floor
uint8_t color_lum_min = 0;
uint8_t color_lum_max = 0;
uint8_t color_cb_min = 0;
uint8_t color_cb_max = 0;
uint8_t color_cr_min = 0;
uint8_t color_cr_max = 0;

// Configuration vars, what percentage of pixels is considered 'ground' at initialisation
float conf_vision_init_y = 0.96;
float conf_vision_init_u = 0.98;
float conf_vision_init_v = 0.98;

// Configuration vars for FindBestDirection function
int conf_steps = 15; // Amount of boxes the image is divided into.
int conf_baseThreshold = 50; // Threshold of going forward in pixels
float conf_thresholdSlope = 10; // slope of threshold line in pixels per degree
float conf_vision_directbox_height = 0.95;

// Configuration vars GetFuzzyValues function
int conf_vision_fuzzy_ramp_y = 15; // 5 -> Conservative settings.
int conf_vision_fuzzy_ramp_u = 20; // 10
int conf_vision_fuzzy_ramp_v = 20; // 10
float conf_vision_safeToGoForwards_threshold = 0.80; // As a percentage

// Variables used for communication with the control module
uint8_t performGroundScan = 0;
uint8_t orange_avoider_safeToGoForwards        = false;

// The optionMatrix, every pixel is a cell in the matrix
float optionMatrix[240][520]; // The values in this matrix should be the sum of the number of pixels which are found to be 'ground' in the rectangle cornered by the x,y position and the top left corner.

/*This function checks whether it's safe to go forwards, it is being called from the controller module. It determines this by comparing the percentage of ground in a small rectangle at the bottom of the screen.
The boolean NormalFrameWidth is set to false by the controller module when the drone is stuck with no easy path out. It then decreases the width of the rectangle with the hope to provide a path out.*/
bool checkIfSafeToGoForwards(bool NormalFrameWidth) {
    if(NormalFrameWidth){
        // normal operation
        orange_avoider_safeToGoForwards = findPercentageGround(0, 130, 130, 390) > conf_vision_safeToGoForwards_threshold;
    } else{
        // when the drone is 'stuck'
        orange_avoider_safeToGoForwards = findPercentageGround(0, 130, 190, 330) > conf_vision_safeToGoForwards_threshold;
    }
    return orange_avoider_safeToGoForwards;
}

/* This function determines the best direction to fly
 * The boolean flying determines whether there should be a threshold on returning a direction other than forward. This is true during flying, because it is not desirable to keep changing direction while flying (can result in jitter). It should only be done when another direction is significantly better. When the drone is however not moving forward, but instead scanning the horizon there should be no threshold, but the best direction should be selected.
 * */
float findBestDirection(bool flying) {

    int width_y = 520;
    int height_x = 240;
    int y_offset_min;
    int y_offset_max;

    int x_offset_min = 0;
    int x_offset_max = height_x * conf_vision_directbox_height; // Take 50% of image (bottom half)

    int g_pixels[conf_steps - 1];
    int g_pixels_updated[conf_steps - 1];
    float directionMatrix[conf_steps - 1];
    int pixel_step = width_y / conf_steps;
    int imageWidth = 90; // Width of camera image is 90 degrees. Determined empirically
    float threshold;
    float direction_deg = 0.0;

    // Loop through the different boxes (each box represents a possible direction)
    // Store the number of ground pixels in each direction-box in g_pixels[]
    // Store the direction in degrees in directionMatrix[]
    for (int y = 0; y < conf_steps; y++) {

        //first set frame
        y_offset_min = pixel_step * y;
        y_offset_max = pixel_step * (y + 1);

        // Make sure the frame is within the camera image
        if (y_offset_max > 518) {
            y_offset_max = 518;
        }

        // Calculate & store the number of ground pixels within each frame
        g_pixels[y] = findPercentageGround(x_offset_min, x_offset_max, y_offset_min, y_offset_max)*(x_offset_max-x_offset_min)*(y_offset_max-y_offset_min);

        // Calculate direction (in degrees) corresponding to the box under consideration
        float stepWidth = (float) imageWidth / conf_steps;
        directionMatrix[y] = (-imageWidth / 2  + stepWidth / 2) +
                             stepWidth * y; // Initial point (half a step from the left boundary plus stepsize)

        // Linear threshold, a higher threshold is given to larger direction changes since these will require higher yaw angle changes. (wbad for stability & speed)
        threshold = (float) abs(conf_thresholdSlope * directionMatrix[y]) + conf_baseThreshold;
        if (flying) {
            // While flying apply the threshold
            g_pixels_updated[y] = g_pixels[y] - threshold;
        } else {
            // If not in flight no threshold is needed.
            g_pixels_updated[y] = g_pixels[y];
        }

    }

    // Loop through the frames again to find the best one
    int g_pixels_updated_max = 0;
    for (int w = 0; w < conf_steps; w++) {
        if (g_pixels_updated[w] > g_pixels_updated_max) {
            g_pixels_updated_max = g_pixels_updated[w];
            direction_deg = directionMatrix[w];
        }
    }
    // The best direction is send to the ground station
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 1, &direction_deg);

    // Return the best direction in degrees
    return direction_deg;
}


/* The calculateOptionMatrix() function is part of the video process, every frame provided by the camera passes through this function.
 * It has 2 main goals:
 *      1) Calculate the likelihood of each pixel being ground and store this in the optionMatrix matrix.
 *      2) Make the above process visible by changing the visual image to represent the likelihood of being ground
 * */
struct image_t *calculateOptionMatrix(struct image_t *input_img)
{
    // Adjust the part of the image that is being investigated based on the pitch angle.
    // If the drone moves forward it pitches down, so the camera would be more directed to the ground. This is compensated by taking an higher part of the pixel for processing.
    // It must be noted that the effect of this is not direct, but has a few frames delay.

    // Find pitch angle
    struct FloatEulers* current_eulerAngles = stateGetNedToBodyEulers_f();

    // Calculate & apply resulting offset for the offset of the image taken (could be improved by not taking a linear relationship)
    mt9f002.offset_x = 1700 - current_eulerAngles->theta * 150;
    printf("Desired offset_x: %d\n", mt9f002.offset_x);
    mt9f002_update_resolution(&mt9f002);

    // The part below will only be executed when the ground initialization is to be performed
    if(performGroundScan) {
        // This part resets the window for performing a ground scan
        isp_request_statistics_yuv_window( MT9F002_INITIAL_OFFSET_X-500, MT9F002_INITIAL_OFFSET_X + MT9F002_SENSOR_WIDTH, MT9F002_INITIAL_OFFSET_Y, MT9F002_INITIAL_OFFSET_Y + MT9F002_SENSOR_HEIGHT, 0, 0);
        isp_set_statistics_yuv_window();

        // Perform initialisation on the color of the ground
        createHistogram(input_img);
        performGroundScan = 0;
        // Once the histogram is generated, this part pans it back to its original state.
        // isp_request_statistics_yuv_window( MT9F002_INITIAL_OFFSET_X, MT9F002_INITIAL_OFFSET_X + MT9F002_SENSOR_WIDTH, MT9F002_INITIAL_OFFSET_Y, MT9F002_INITIAL_OFFSET_Y + MT9F002_SENSOR_HEIGHT, 0, 0);
        // isp_set_statistics_yuv_window();
    }

//    This part will loop through all the pixels and calculate the likelihood of it being ground
    uint8_t *source = input_img->  buf;  // Go trough all the pixels
    for (uint16_t y = 0; y < (input_img->h); y++) {
        for (uint16_t x = 0; x < (input_img->w); x += 1) {
            int Y,U,V;
            //int R,G,B;
            float F; // The fuzzy value that determines the groundness per pixel
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

            // Calculate likelihood of the current pixel being ground and add this to the matrix
            F=getFuzzyValue(Y,U,V);
            optionMatrix[x][y] +=F;

            // To visualize the resulting image all pixels above a certain threshold can be colored
            float fuzzy_threshold = -0.8;
            if (F > fuzzy_threshold) {
                // Change color of the ground pixels
                if(x % 2 == 0) {
                    source[1] = (int)F*255;
                } else {
                    source[3] = (int)F*255;
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

    // Draw boxes
    drawRectangle(input_img,0, 130, 130, 390); // Safe to go forward
    drawRectangle(input_img,0, 130, 190, 330); // Sate to go forwards (smaller window, used if stuck)
    drawRectangle(input_img,0, 239, 150, 370); // Speed
    drawRectangle(input_img, 0, 192, 243, 277); // Middle step of directionFinder.

    return input_img;
}

/* The function that determines the likelihood of the pixel being ground. Returns a value between 0 and 1, based on the inputed Y,U&V*/
float getFuzzyValue(int Y, int U, int V) {
    float F;
    float Y_f,V_f,U_f;

    // Determine the boundaries of the fuzzy set
    // A possible improvement would be to move these to another function so they are not calculated every time (waste of processing time)

    // Determine Y bounds
    int color_lum_min_lower = color_lum_min - conf_vision_fuzzy_ramp_y; // Determine upper and lower bounds per value.
    int color_lum_min_upper = color_lum_min;
    int color_lum_max_lower = color_lum_max; // Determine upper and lower bounds per value.
    int color_lum_max_upper = color_lum_max + conf_vision_fuzzy_ramp_y;

    // Determine U bounds
    int color_cb_min_lower = color_cb_min - conf_vision_fuzzy_ramp_u; // Determine upper and lower bounds per value.
    int color_cb_min_upper = color_cb_min;
    int color_cb_max_lower = color_cb_max; // Determine upper and lower bounds per value.
    int color_cb_max_upper = color_cb_max + conf_vision_fuzzy_ramp_u;

    // Determine V bounds
    int color_cr_min_lower = color_cr_min - conf_vision_fuzzy_ramp_v; // Determine upper and lower bounds per value.
    int color_cr_min_upper = color_cr_min;
    int color_cr_max_lower = color_cr_max; // Determine upper and lower bounds per value.
    int color_cr_max_upper = color_cr_max + conf_vision_fuzzy_ramp_v;

    // Determine Fuzzy Y value
    Y_f = 1; //  When Y is larger than the min_upper bound or lower than the max_lower bound.
    if (Y > color_lum_min_lower && Y < color_lum_min_upper) { // If Y on fuzzy ramp around the min value
        Y_f = 1 / color_lum_min * (color_lum_min - color_lum_min_lower); // Assign a value from 0 to 1

    } else if (Y > color_lum_max_lower && Y < color_lum_max_upper) { // If Y on fuzzy ramp around the max value
        Y_f = 1 - (1 / color_lum_min * (color_lum_min - color_lum_min_lower)); // Assign a value from 1 to 0.
    }

    Y_f = (Y <= color_lum_min_lower) ? 0.0 : Y_f; // If Y is smaller than the lowest bound, assign 0, otherwise remain old value.
    Y_f = (Y >= color_lum_max_upper) ? 0.0 : Y_f;// If Y is larger than the highest bound, assign 0, otherwise remain old value.

    // Determine Fuzzy U value
    U_f = 1; //  When U is larger than the min_upper bound or lower than the max_lower bound.
    if (U > color_cb_min_lower && U < color_cb_min_upper) { // If U on fuzzy ramp around the min value
        U_f = 1 / color_cb_min * (color_cb_min - color_cb_min_lower); // Assign a value from 0 to 1

    } else if (U > color_cb_max_lower && U < color_cb_max_upper) { // If U on fuzzy ramp around the max value
        U_f = 1 - (1 / color_cb_min * (color_cb_min - color_cb_min_lower)); // Assign a value from 1 to 0.
    }

    U_f=(U <= color_cb_min_lower) ? 0.0 : U_f; // If U is smaller than the lowest bound, assign 0, otherwise remain old value.
    U_f=(U >= color_cb_max_upper) ? 0.0 : U_f; // If U is larger than the highest bound, assign 0, otherwise remain old value.

    // Determine Fuzzy V value
    V_f = 1; //  When V is larger than the min_upper bound or lower than the max_lower bound.
    if (V > color_cr_min_lower && V < color_cr_min_upper) { // If V on fuzzy ramp around the min value
        V_f = 1 / color_cr_min * (color_cr_min - color_cr_min_lower); // Assign a value from 0 to 1

    } else if (V > color_cr_max_lower && V < color_cr_max_upper) { // If V on fuzzy ramp around the max value
        V_f = 1 - (1 / color_cr_min * (color_cr_min - color_cr_min_lower)); // Assign a value from 1 to 0.
    }

    V_f = (V <= color_cr_min_lower) ? 0.0 : V_f; // If V is smaller than the lowest bound, assign 0, otherwise remain old value.
    V_f = (V >= color_cr_max_upper) ? 0.0 : V_f; // If V is larger than the highest bound, assign 0, otherwise remain old value.

    // The fuzzy values are combined using multiplication.
    F = Y_f*U_f*V_f;
    return F;
}

/* This function returns the percentage of ground pixels in any given rectangle based on the global optionMatrix */
float findPercentageGround(int x_min, int x_max, int y_min, int y_max){
    int sum = 0;
    float percentage = 0.0;

    // An option to print part of the matrix (for debugging only)
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
    //VERBOSE_PRINT("Ground percentage x:%d-%d, y%d-%d=%f\n",x_min,x_max,y_min,y_max,percentage);
    return percentage;
}


/* This function is called when the ground initialization needs to be performed*/
void updateGroundFilterSettings(){
    // Called right before start of the obstacle course
    performGroundScan = 1;
    // Further handling of this is done by the calculateOptionMatrix() function, which sets the performGroundScan variable back to 0 when done.
}


// Simple Function to find the maximum element of a given array, used by the ground initialization function
// (in which case the element number is the corresponding intensity for the channel)
int find_max(int a[], int n) {
  int c, max, index_max;
  max = a[0];
  index_max = 0;

  for (c = 1; c < n; c++) {
    if (a[c] > max) {
       index_max = c;
       max = a[c];
    }
  }
  return index_max;
}
    

// Function to find the limits of an given histogram within certain given boundaries
int *find_limits(int a[], int n, float margin){
    int i, id, range = 0, sum = 0;

    id = find_max(a,n);
    int lowerlim = id;
    int upperlim = id;

    for(i=0;i<n;i++){
        sum += a[i];
    }
    float ratio = 0.0;
    while(ratio<margin){                    //Runs until the ratio is achieved
        if(a[lowerlim-1]<a[upperlim+1]){
            if(upperlim ==n){
                upperlim=n-1; //So that the upper limit remains in bounds
            }
            upperlim += 1;
        }else if(a[lowerlim-1]>a[upperlim+1]){
            if(lowerlim ==0){
                lowerlim=1; //So that lower limit remains in bounds
            }
            lowerlim -= 1;
        }else{upperlim +=1;}
        

        range = 0;
        for(i=lowerlim; i<=(upperlim);i++){
            range += a[i];
        }
        VERBOSE_PRINT("l:%d u:%d r:%d s:%d n:%d %f\n",lowerlim, upperlim, range, sum, n, ratio);
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

        // First the histogram array needs to be filled by looping through all the pixels
        uint8_t *source = input_img->buf;
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

        // Find limits for Y
        results = find_limits(histo_y, n, conf_vision_init_y);
        ymax = results[1];
        ymin = results[0];

        // Find limits for V
        results = find_limits(histo_v, n, conf_vision_init_v);
        vmax = results[1];
        vmin = results[0];

        // Find limits for U
        results = find_limits(histo_u, n, conf_vision_init_u);
        umax = results[1];
        umin = results[0];

        // Print the found limits (if printing is turned on)
        VERBOSE_PRINT("\n Y range %d to %d\n U range %d to %d\n V range %d to %d\n", ymin, ymax, umin, umax, vmin,
                      vmax);

        // Save the limits to the global variables used by the getFuzzyValue() function
        color_lum_min = ymin;
        color_lum_max = ymax;
        color_cb_min  = umin;
        color_cb_max  = umax;
        color_cr_min  = vmin;
        color_cr_max  = vmax;

        performGroundScan = 0;
        return input_img;
    }

/* Draw an arbitrary rectangle on the camera image */
void drawRectangle(struct image_t *input_img, int x_min, int x_max, int y_min, int y_max) {
    static uint8_t color[4] = {255, 255, 255, 255};

    // First calculate the corner points of the rectangle
    struct point_t BL = {
            x_min,
            y_min
    };
    struct point_t TL = {
            x_min,
            y_max
    };
    struct point_t TR = {
            x_max,
            y_max
    };
    struct point_t BR = {
            x_max,
            y_min
    };

    // Then draw lines between the 4 corners.
    image_draw_line_color(input_img, &BL, &TL, color);
    image_draw_line_color(input_img, &TL, &TR, color);
    image_draw_line_color(input_img, &TR, &BR, color);
    image_draw_line_color(input_img, &BR, &BL, color);
}

// Function to send the messages from this module to the GCS
static void send_ground_follower_info(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_GROUND_FOLLLOWER_INFO(trans, dev, AC_ID, &color_lum_min, &color_lum_max, &color_cb_min, &color_cb_max, &color_cr_min, &color_cr_max);
}

// Initialize this module
void ground_follower_init()
{
    // Initialise the settings of the ground filter
/*    color_lum_min = 37;
    color_lum_max = 209;
    color_cb_min  = 47;
    color_cb_max  = 110;
    color_cr_min  = 123;
    color_cr_max  = 173;*/

    // Apply the calculateOptionMatrix function to the images generated by the video camera
    listener = cv_add_to_device(&GROUND_FOLLOWER_CAMERA, calculateOptionMatrix);
    // Register telemetry function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GROUND_FOLLLOWER_INFO, send_ground_follower_info);
}


// This function calculated the percentage of ground pixels in the 'Speed' rectangle.
// The result is then used by the controller module to determine the speed.
float DetermineTrajectoryConfidence()
{
    // Define the size of the 'Speed' rectangle
    int x_min_tc=0;
    int x_max_tc=239;
    int y_min_tc=150;
    int y_max_tc=370;
    float GroundPercentage;

    // Calculate the percentage of ground pixels within this rectangle
    GroundPercentage=findPercentageGround(x_min_tc, x_max_tc, y_min_tc, y_max_tc);
    return GroundPercentage;
}




