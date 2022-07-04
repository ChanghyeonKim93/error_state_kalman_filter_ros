#include "error_state_kalman_filter.h"

// Initialize Static member variables
Mat33 ESKF::I33 = Mat33::Identity();
Mat44 ESKF::I44 = Mat44::Identity();
Mat33 ESKF::O33 = Mat33::Zero();
Mat44 ESKF::O44 = Mat44::Zero();
Mat34 ESKF::O34 = Mat34::Zero();
Mat43 ESKF::O43 = Mat43::Zero();    

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