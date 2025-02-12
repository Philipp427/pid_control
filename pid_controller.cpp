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
PID::PID() {}

// Destructor
PID::~PID() {}

// Initialize PID coefficients and output limits
void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
    output_lim_max = output_lim_maxi;
    output_lim_min = output_lim_mini;

    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    delta_time = 0.0;
}

void PID::UpdateError(double cte) {
    if (delta_time > 0.0) {
        d_error = (cte- p_error) / delta_time;
    } else {
        d_error = 0.0;
    }

    p_error = cte;
    i_error += cte * delta_time;
}

double PID::TotalError() {
    double control = Kp * p_error + Ki * i_error + Kd * d_error;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time = new_delta_time;
}