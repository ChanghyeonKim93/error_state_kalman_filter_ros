#include "error_state_kalman_filter.h"

ESKF::ESKF()
: measurement_noise_(), process_noise_(), X_nom_(), dX_()
{
    std::cout << "Error State Kalman Filter - constructed\n";
};

ESKF::~ESKF(){
    std::cout << "Error State Kalman Filter - destructed\n";
};

void ESKF::propagate(){
    // Do implementation
};

void ESKF::update(){
    // Do implementation

};