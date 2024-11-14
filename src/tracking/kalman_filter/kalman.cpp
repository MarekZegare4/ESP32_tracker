#include "kalman.h"

float headingKalman(float heading){
    float q = 0.004;
    float r = 0.1452;
    float k = 0;
    static float p_m = 0;
    static float x_p = 0;
    static float p_p = p_m + q;

    p_p = p_p + q * r;
    k = p_p / (p_p + r);
    x_p = x_p + k * (heading - x_p);
    p_p = (1 - k) * p_p;
    return x_p;
}

float pitchKalman(float pitch){
    float q = 0.01;
    float r = 5;
    float k = 0;
    static float p_m = 0;
    static float x_p = 0;
    static float p_p = p_m + q;

    p_p = p_p + q * r;
    k = p_p / (p_p + r);
    x_p = x_p + k * (pitch - x_p);
    p_p = (1 - k) * p_p;
    return x_p;
}

float rollKalman(float roll){
    float q = 0.01;
    float r = 5;
    float k = 0;
    static float p_m = 0;
    static float x_p = 0;
    static float p_p = p_m + q;

    p_p = p_p + q * r;
    k = p_p / (p_p + r);
    x_p = x_p + k * (roll - x_p);
    p_p = (1 - k) * p_p;
    return x_p;
}