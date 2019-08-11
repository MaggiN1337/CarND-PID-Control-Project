#include <vector>
#include <numeric>
#include <iostream>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() = default;

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
     */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 1;
    i_error = 1;
    d_error = 1;

}

void PID::UpdateError(double cte) {
    /**
     * TODO: Update PID errors based on cte.
     */
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;
}

double PID::TotalError() {
    /**
     * TODO: Calculate and return the total error
     */

    double err = -Kp * p_error - Ki * i_error - Kd * d_error;

    return err;  // TODO: Add your total error calc here!
}

void PID::twiddle(float tolerance) {
    std::vector<double> params = {0,0,0};
    std::vector<double> delta_params = {1,1,1};

    PID newPID;
    newPID.Init(0,0,0);
    double best_err = newPID.TotalError();

    int count = 0;
    do {
        std::cout << "Interation: " << count << " has an error of: " << best_err << std::endl;

        for (int i = 0; i< params.size(); i++){
            params[i] += delta_params[i];
            newPID.Init(0,0,0);
            double error = newPID.TotalError();

            if (best_err < error){
                best_err = error;
                delta_params[i] *= 1.1;
            } else {
                params[i] -= 2* delta_params[i];
                newPID.Init(0,0,0);
                error = newPID.TotalError();
                if (error < best_err){
                    best_err = error;
                    delta_params[i] *= 1.1;
                } else {
                    params[i] += delta_params[i];
                    delta_params[i] *= 0.9;
                }
            }
        }

        count++;
    } while (std::accumulate(delta_params.rbegin(), delta_params.rend(), 0.00) > tolerance);

}
