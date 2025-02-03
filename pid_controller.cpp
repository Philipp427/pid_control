/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

// Constructor to initialize PID errors and coefficients
PID::PID() : p_error(0), i_error(0), d_error(0), Kp(0), Ki(0), Kd(0), output_lim_max(0), output_lim_min(0), delta_time(0) {}

// Destructor
PID::~PID() {}

// Initialize PID coefficients and output limits
void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    Kp = Kpi;
    Kd = Kdi;
    Ki = Kii;
    output_limit_max = output_lim_maxi;
    output_limit_min = output_lim_mini;
}

// Update PID errors based on cross-track error (cte)
void PID::UpdateError(double cte) {
    if (prev_err == NULL) {
        prev_err = cte;
    } else {
        prev_err = err;
    }
    err = cte;
    sum_err += (cte * delta_time);
}

// Calculate and return the total error
double PID::TotalError() {
    double control;
    if (delta_time > 0.0) {
        control = (err * -1.0 * Kp) + (((err - prev_err) / delta_time) * -1.0 * Kd) + (-1.0 * Ki * sum_err);
    } else {
        control = (err * -1.0 * Kp) + (-1.0 * Ki * sum_err);
    }
    if (control < output_limit_min && control < output_limit_max) {
        return output_limit_min;
    } else if (control > output_limit_max && control > output_limit_min) {
        return output_limit_max;
    }
    return control;
}

// Update the delta time with new value
double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time = new_delta_time;
}