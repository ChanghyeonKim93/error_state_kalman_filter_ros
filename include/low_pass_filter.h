#ifndef _LPF_H_
#define _LPF_H_

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
typedef Matrix<double,3,1> Vec3;

template <typename T>
class LowPassFilter{
private:
    double cutoff_freq_; // Hz
    double dt_; // data sampling rate. (tentative value is OK.)
    double alpha_; 
    double RC_;
    double dt_inv_RC_plus_dt_;
    double RC_inv_RC_plus_dt_;

    T data_prev_;
    T data_filtered_;

    double timestamp_prev_;

    bool is_initialized_;

public:
    LowPassFilter(double cutoff_freq, double sampling_rate)
    {
        is_initialized_ = false;
        cutoff_freq_ = cutoff_freq;
        dt_ = 1.0/sampling_rate;

        alpha_ = 1.0 + 1.0/(cutoff_freq_*2.0*3.141592*dt_);
        RC_ = dt_*(1-alpha_)/alpha_;
        dt_inv_RC_plus_dt_ = dt_/(RC_+dt_);
        RC_inv_RC_plus_dt_ = RC_/(RC_+dt_);
    };

    T doFilterAndGetEstimation(const T& data_curr, double timestamp_curr){
        if(is_initialized_){
            double time_elapsed = timestamp_curr - timestamp_prev_;
            if(time_elapsed >= 0.01 ){ // millisecond
                // too long time elapsed... re-intialization the filter
                data_prev_ = data_curr;
                data_filtered_ = data_prev_;
            }
            else{
                // Do filtering
                data_filtered_ = dt_inv_RC_plus_dt_*data_curr + RC_inv_RC_plus_dt_*data_prev_;
                data_prev_ = data_curr;
            }

            if(time_elapsed < 0){
                //시간,... 역행...?
                data_filtered_ = data_curr;
                data_prev_ = data_curr;
            }
        }
        else{
            is_initialized_ = true;
            timestamp_prev_ = timestamp_curr;
            data_prev_ = data_curr;

            data_filtered_ = data_prev_;
        }

        return data_filtered_;
    };

    T getFilteredValue(){
        return data_filtered_;
    };

};

#endif