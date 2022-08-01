#ifndef _LPF_H_
#define _LPF_H_

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
typedef Matrix<double,3,1> Vec3;

template <typename T>
class LowPassFilter{
private:
    double f_cut_; // Hz
    double w_cut_; // rad/s
    double dt_; // data sampling rate. (tentative value is OK.)
    double tau_; // time constant 
    double tau_taudt_;
    double dt_taudt_;

    T data_prev_;
    T data_filtered_;

    double timestamp_prev_;

    bool is_initialized_;

public:
    LowPassFilter(double cutoff_freq, double sampling_rate)
    {
        is_initialized_ = false;
        f_cut_ = cutoff_freq;
        dt_ = 1.0/sampling_rate;

        w_cut_ = f_cut_*2.0*3.141592;
        tau_ = 1.0/w_cut_;

        tau_taudt_ = tau_/(tau_+dt_);
        dt_taudt_  = dt_/(tau_+dt_);

        // y_n = tau/(tau+dt) * y_(n-1) + dt/(tau+dt)*x_n

        std::cout << "cutoff frequency of LPF:"<< f_cut_ << " Hz" << std::endl;
        std::cout << "tau_taudt_:"<< tau_taudt_ << std::endl;
        std::cout << "dt_taudt_:"<< dt_taudt_ << std::endl;
    };

    T doFilterAndGetEstimation(const T& data_curr, double timestamp_curr){
        if(is_initialized_){
            double time_elapsed = timestamp_curr - timestamp_prev_;
            if(time_elapsed >= 0.03 ){ // millisecond
                // too long time elapsed... re-intialization the filter
                data_filtered_ = data_curr;
            }
            else{
                // Do filtering
                data_filtered_ = dt_taudt_*data_curr + tau_taudt_*data_filtered_;
                // data_filtered_ = data_curr;
            }

            if(time_elapsed < 0){
                //시간,... 역행...?
                data_filtered_ = data_curr;
            }
        }
        else{
            is_initialized_ = true;

            data_filtered_  = data_curr;
        }

        timestamp_prev_ = timestamp_curr;
        data_prev_      = data_curr;
        return data_filtered_;
    };

    T getFilteredValue(){
        return data_filtered_;
    };

};

#endif