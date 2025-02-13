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
    prev_cte = 0.0;
    int_cte = 0.0;
}

void PID::UpdateError(double cte) {
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;
    p_error = Kp * cte;
    i_error = Ki * int_cte;
    d_error = Kd * diff_cte;
}

double PID::TotalError() {
    double control = p_error + i_error + d_error;
    if (control > output_lim_max) {
        control = output_lim_max;
    } else if (control < output_lim_min) {
        control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time = new_delta_time;
    return delta_time;
}