
#include "PID.h"

/**
 * the PID class
 */

PID::PID() = default;

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    /**
     * Initialize PID coefficients (and errors, if needed)
     */
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;

    this->p_error = this->i_error = this->d_error = 0.0;

}

void PID::UpdateError(double cte) {
    /**
     * Update PID errors based on cte.
     */
    this->d_error = cte - p_error;
    this->i_error += cte;
    this->p_error = cte;

}

double PID::TotalError() {
    /**
     * Calculate and return the total error
     */

    double err = -Kp * p_error - Ki * i_error - Kd * d_error;

    return err;
}
