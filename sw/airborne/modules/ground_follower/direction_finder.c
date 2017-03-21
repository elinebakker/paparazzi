//
// Created by eline on 17-3-17.
//

#include "direction_finder.h"
#include <stdio.h>


float findHeading(int optionMatrix[240][520]){

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
    printf("pixels right %i ",g_pixels_right);

    int threshold=1000;

    if(g_pixels_left>g_pixels_right && g_pixels_left-g_pixels_middle>threshold) {
        direction_deg = -10.0;

    }else if (g_pixels_left<g_pixels_right && g_pixels_right-g_pixels_middle>threshold){
        direction_deg = 10.0;

    } else {
        direction_deg = 0.0;
    }

    return direction_deg;
}

