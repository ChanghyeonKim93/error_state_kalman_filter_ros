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
    // initialize error covariance matrix
    P_ = CovarianceMat::Identity()*0.005;

    // initialize Fi matrix
    Fi_ << O33, O33, O33, O33, 
           I33, O33, O33, O33,
           O33, I33, O33, O33,
           O33, O33, I33, O33,
           O33, O33, O33, I33;
           
    Vec3 ba_init(0.011, 0.007, 0.201);
    Vec3 bg_init(0.0043586,-0.0011758,-0.011671);

    std::cout << "Error State Kalman Filter - constructed\n";
};

ESKF::~ESKF(){
    std::cout << "Error State Kalman Filter - destructed\n";
};

void ESKF::predict(double ax, double ay, double az, double wx, double wy, double wz, double t_now){
    // Do implementation
    std::cout << "Predict...\n";
    double dt = t_now - t_prev_;
    t_prev_ = t_now;

    // Do prediction    
    // 0. measurement
    Vec3 am(ax,ay,az);
    Vec3 wm(wx,wy,wz);

    // 1. nominal state propagation
    NominalState X_nom_prev = X_nom_;
    ErrorState   dX_prev = dX_;
    // X_nom_ = predict_nominal(X_nom_prev, am, wm, dt);

    // 2. error-state propagation
    // dX_ = predict_error(X_nom_prev, dX_, )

    // 3. Error Covariance propagation
    // F0 = errorStateF(X_nom_prev, dX_prev,am,wm);
    // eF0dt = expm(F0 * dt);
    // P_ = eF0dt * P_ * eF0dt.transpose() + Fi * Q_*Fi.transpose();

    // X_nom_.replace(X_nom_update);
    // dX_.replace(dX_update);
    
};

void ESKF::update(){
    // Do implementation
    std::cout << "Update...\n";
};



void ESKF::updateNominal(const NominalState& X_nom, const Vec3& am, const Vec3& wm, double dt, 
    NominalState& X_nom_update){
    Matrix3d R_B0Ik = geometry::q2r(X_nom.q);

    // Update nominal state
    NominalState X_nom_update;
    Vec3 Adt = (R_B0Ik*(am-X_nom.ba)-fixed_param_.grav)*dt;
    X_nom_update.p = X_nom.p + (X_nom.v + 0.5*Adt)*dt;
    X_nom_update.v = X_nom.v + Adt;
    X_nom_update.q = geometry::q_right_mult(geometry::rotvec2q((wm-X_nom.bg)*dt))*X_nom.q;
    X_nom_update.ba = X_nom.ba;
    X_nom_update.bg = X_nom.bg;
};

void ESKF::updateError(const NominalState& X_nom, const ErrorState& dX, const Vec3& am, const Vec3& wm, double dt,
    ErrorState& dX_update)
{
    FMat eF0dt;

    // Update nominal state
    // dX_update = eF0dt*dX;
};

void ESKF::errorStateF(const NominalState& X_nom, const ErrorState& dX, const Vec3& am, const Vec3& wm,
    FMat& res)
{
    Matrix3d R_B0Ik = geometry::q2r(X_nom.q);

    res << O33, I33, O33, O33, O33,
           O33, O33, -R_B0Ik*geometry::skewMat(am-X_nom.ba),-R_B0Ik, O33,
           O33, O33, -geometry::skewMat(wm-X_nom.bg), O33, -I33,
           O33, O33, O33, O33, O33, 
           O33, O33, O33, O33, O33;
};