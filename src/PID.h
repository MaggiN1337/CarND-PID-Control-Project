#ifndef PID_H
#define PID_H

class PID {
public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_, bool twiddle);

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();

    void twiddle(float tolerance);

private:
    /**
     * PID Errors
     */
    double p_error{};
    double i_error{};
    double d_error{};

    /**
     * PID Coefficients
     */
    double Kp{};
    double Ki{};
    double Kd{};
    /**
     * twiddle params
     */
    bool is_twiddle = false;
    int update_count = 0;
//    std::vector<double> delta_params;
    void Init(std::vector<double> p, std::vector<double> err);
};

#endif  // PID_H