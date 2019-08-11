#include <vector>
#include <numeric>
#include <iostream>
#include "PID.h"

const float TWIDDLE_TOLERANCE = 0.001;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() = default;

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool twiddle) {
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
     */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = i_error = d_error = 0.0;

//    delta_params = {Kp_ * TWIDDLE_TOLERANCE, Ki_ * TWIDDLE_TOLERANCE, Kd_ * TWIDDLE_TOLERANCE};

    is_twiddle = twiddle;

}

void PID::UpdateError(double cte) {
    /**
     * TODO: Update PID errors based on cte.
     */
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;

    update_count++;
    std::cout << "Update step: " << update_count << std::endl;
}

double PID::TotalError() {
    /**
     * Calculate and return the total error
     */

    double err = -Kp * p_error - Ki * i_error - Kd * d_error;

    return err;
}

void PID::twiddle(float tolerance) {
    std::vector<double> params = {Kp, Ki, Kd};
    std::vector<double> errors ={p_error, i_error, d_error};
    std::vector<double> delta_params = {1, 1, 1};
    double sum_dp = delta_params[0] + delta_params[1] + delta_params[2];

    double best_err = TotalError();
    PID newPID;

    int count = 0;
    while (sum_dp > TWIDDLE_TOLERANCE && count < 100){
        for (unsigned long i=0; i<params.size(); i++){
            params[i] += delta_params[i];
            newPID.Init(params, errors);
            if (newPID.TotalError() < best_err){
                best_err = newPID.TotalError();
                delta_params[i] *= 1.1;
            } else {
                params[i] -= 2.0 * delta_params[i];
                newPID.Init(params, errors);
                if (newPID.TotalError() < best_err){
                    best_err = newPID.TotalError();
                    delta_params[i] *=1.1;
                } else {
                    params[i] += delta_params[i];
                    delta_params[i] *= 0.9;
                }
            }
        }
        std::cout << "Run " << count << ": Parameters: " << params[0] <<", "<< params[1]<<", "<< params[2] << std::endl;
        count++;
    }
    std::cout << "-----------" << std::endl;

//    double sum = std::accumulate(delta_params.rbegin(), delta_params.rend(), 0.00);
//    std::vector<double> params = {0, 0, 0};
//
//    if (update_count > 100 && update_count < 200 && sum > TWIDDLE_TOLERANCE) {
//        double total_error = p_error * p_error;
//        std::cout << "Sum of delta params: " << sum << std::endl;
//
////    double best_err = 9999;
////    if (!is_twiddle) {
//        double best_err = TotalError();
////        is_twiddle = true;
////    }
////
//        for (unsigned long i = 0; i < params.size(); i++) {
//
//            params[i] += delta_params[i];
//            double error = TotalError();
//
//            if (std::abs(error) < best_err) {
//                best_err = error;
//                delta_params[i] *= 1.1;
//
//            } else {
//                params[i] -= 2 * delta_params[i];
//                error = TotalError();
//
//                if (std::abs(error) < best_err) {
//                    best_err = error;
//                    delta_params[i] *= 1.1;
//
//                } else {
//                    params[i] += delta_params[i];
//                    delta_params[i] *= 0.9;
//                }
//            }
//        }
////        Init(params[0], params[1], params[2], false);
//        total_error = total_error / update_count;
//        std::cout << "New Error: " << total_error << std::endl;
//    }
//    std::cout << "Kp: " << params[0] << " - Ki: " << params[1] << " - Kd: " << params[2] << std::endl;
}

void PID::Init(std::vector<double> p, std::vector<double> err) {
    this->Kp = p[0];
    this->Ki = p[1];
    this->Kd = p[2];

    this->p_error = err[0];
    this->i_error = err[1];
    this->d_error = err[2];
}
