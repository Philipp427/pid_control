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

PID::PID() : p_error(0), i_error(0), d_error(0), Kp(0), Ki(0), Kd(0), output_lim_max(0), output_lim_min(0), delta_time(0) {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    /**
    * Initialize PID coefficients (and errors, if needed)
    **/
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
    output_lim_max = output_lim_maxi;
    output_lim_min = output_lim_mini;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    delta_time = 0;
}

void PID::UpdateError(double cte) {
    /**
    * Update PID errors based on cte.
    **/
    d_error = (cte - p_error) / delta_time;
    p_error = cte;
    i_error += cte * delta_time;
}

double PID::TotalError() {
    /**
    * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
    */
    double control = Kp * p_error + Ki * i_error + Kd * d_error;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    /**
    * Update the delta time with new value
    */
    delta_time = new_delta_time;
    return delta_time;
}