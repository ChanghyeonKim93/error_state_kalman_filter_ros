#ifndef _ERROR_STATE_KALMAN_FILTER_H_
#define _ERROR_STATE_KALMAN_FILTER_H_

#include <iostream>
#include <Eigen/Dense>

#include "geometry_library.h"
#include "low_pass_filter.h"

// #define VERBOSE_STATE

using namespace Eigen;
typedef Matrix<double,3,1> Vec3;
typedef Matrix<double,4,1> Vec4;
typedef Matrix<double,7,1> Vec7;
typedef Matrix<double,15,1> ErrorStateVec;
typedef Matrix<double,16,1> NominalStateVec;

typedef Matrix<double,3,3> Mat33;
typedef Matrix<double,4,4> Mat44;
typedef Matrix<double,3,4> Mat34;
typedef Matrix<double,4,3> Mat43;
typedef Matrix<double,15,15> Mat1515;

typedef Matrix<double,15,15> CovarianceMat;
typedef Matrix<double,15,15> FMat;
typedef Matrix<double,15,15> expmFMat;
typedef Matrix<double,7,15>  HMat;
typedef Matrix<double,15,7>  KMat;
typedef Matrix<double,7,7>   RMat;
typedef Matrix<double,12,12> QMat;

typedef Matrix<double,15,15> Mat1515;

#define GRAVITY_MAGNITUDE 9.81

#define POW2(x) ((x)*(x))
#define BLOCK33(A,i,j) ((A).block<3,3>(3*i,3*j))

class ESKF{
public:

    struct FixedParameters;
    
    struct ErrorStateCovariance;
    struct NominalState;
    struct ErrorState;

    struct Measurement;
    struct Observation;

    struct ProcessNoise;
    struct MeasurementNoise;

    struct EmergencyResetRules;


private:
    // Related to Kalman filter implementation .
    void predictNominal(const NominalState& X_nom, const Vec3& am, const Vec3& wm, double dt,
                       NominalState& X_nom_update);
    void predictError(const expmFMat& eF0dt, const ErrorState& dX,
                       ErrorState& dX_update);

    void errorStateF(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                       FMat& res);
    void expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                       expmFMat& expmF);
    void calcH(const NominalState& X_nom, HMat& H);

public:
    ESKF();
    ~ESKF();

public:
    void setRotationFromBodyToIMU(const Mat33& R_BI);
    void setRotationFromBodyToIMU(const Vec4& q_BI);

    void setBias(double bias_ax, double bias_ay, double bias_az, 
        double bias_gx, double bias_gy, double bias_gz,
        double bias_mx, double bias_my, double bias_mz);

    void setIMUNoise(double noise_acc, double noise_gyro, double noise_mag);

    void setObservationNoise(double noise_position, double noise_orientation);

public:
    bool isInitialized();

    void predict(const Vec3& am, const Vec3& wm, double t_now); // by imu 
    // void updateMagnetometer(const Vec3& p_observe, const Vec4& q_observe); // by magnetometer
    void updateOptitrack(const Vec3& p_observe, const Vec4& q_observe, double t_now); // by optitrack
    
    void resetFilter(const Vec3& p_init, const Vec4& q_init); // other states go to zeros.

    FixedParameters getFixedParameters();
    void getFilteredStates(NominalState& x_nom_filtered);
    void getGyroLowPassFiltered(Vec3& filtered_gyro);
    void getAccLowPassFiltered(Vec3& filtered_acc);
    void getCovariance(NominalState& x_nom_filtered);

    void showFilterStates();

// Test functions
public:
    void test_FMat(const NominalState& X_nom, const Vec3& am, const Vec3& wm,
                       FMat& res);
    void test_expm_FMat(const FMat& F, const double& dt, int max_approx_order,
                       expmFMat& res);

public:
    static Mat33 I33;
    static Mat44 I44;
    static Mat33 O33;
    static Mat44 O44;
    static Mat34 O34;
    static Mat43 O43;
    static Mat1515 I1515;

    struct FixedParameters{
        Mat33 R_BI; // SO(3), rotation only. drone body frame (== optitrack coordinate frame) 
        // to the IMU frame
        Mat33 R_IB; // R_BI.transpose();
        Vec4  q_BI; // quaternion, rotation only. drone body frame to IMU frame
        Vec4  q_IB; // q_BI.conjugate();
        
        Vec3 grav; // gravity w.r.t. the global frame

        FixedParameters(){
            // R_BI << 1,0,0, 0,-1,0, 0,0,-1;
            R_BI << 0,0,1, 0,-1,0, 1,0,0;
            R_IB = R_BI.transpose();

            q_BI = geometry::r2q(R_BI);
            q_IB = geometry::q_conj(q_BI);

            // grav << 0.0, 0.0, -GRAVITY_MAGNITUDE; // VN100t
            // grav << GRAVITY_MAGNITUDE,0.0,0.0; // MPU9250
            grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE);
        };

        void setRotationFromBodyToIMU(const Mat33& R_BI_input){
            R_BI = R_BI_input;
            R_IB = R_BI.transpose();

            q_BI = geometry::r2q(R_BI);
            q_IB = geometry::q_conj(q_BI);
            grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE);
        };

        void setRotationFromBodyToIMU(const Vec4& q_BI_input){
            q_BI = q_BI_input;
            q_IB = geometry::q_conj(q_BI);

            R_BI = geometry::q2r(q_BI);
            R_IB = R_BI.transpose();
            grav  = R_IB*Vec3(0.0,0.0,GRAVITY_MAGNITUDE);
        };
    };

    struct NominalState{
        Vec3 p;
        Vec3 v;
        Vec4 q;
        Vec3 ba;
        Vec3 bg;
        NominalState() {
            p  = Vec3::Zero();
            v  = Vec3::Zero();
            q  = Vec4::Zero();  q(0) = 1.0;
            ba = Vec3::Zero();
            bg = Vec3::Zero();
        };
        NominalState(const NominalState& nom){
            p  = nom.p;
            v  = nom.v;
            q  = nom.q;
            ba = nom.ba;
            bg = nom.bg;
        };
        void setValues(const Vec3& pi, const Vec3& vi, const Vec4& qi, const Vec3& bai, const Vec3& bgi){
            p  = pi;
            v  = vi;
            q  = qi; 
            ba = bai;
            bg = bgi;
        };
        void setPosition(const Vec3& pi)  { p  = pi;  };
        void setVelocity(const Vec3& vi)  { v  = vi;  };
        void setQuaternion(const Vec4& qi){ q  = qi;  };
        void setBiasAcc(const Vec3& bai)  { ba = bai; };
        void setBiasGyro(const Vec3& bgi) { bg = bgi; };
        void replace(const NominalState& nom){
            p  = nom.p;
            v  = nom.v;
            q  = nom.q;
            ba = nom.ba;
            bg = nom.bg;
        };
        void copyTo(NominalState& X_nom) const {
            X_nom.p  = p;
            X_nom.v  = v;
            X_nom.q  = q;
            X_nom.ba = ba;
            X_nom.bg = bg;
        };
        void injectErrorState(const ErrorState& dX){
            p  += dX.dp;
            v  += dX.dv;
            q = geometry::q_right_mult(geometry::rotvec2q(dX.dth))*q;
            ba += dX.dba;
            bg += dX.dbg;
        };
        void show(){
            std::cout << "X_nom.p:"  << p.transpose()  << "\n";
            std::cout << "X_nom.v:"  << v.transpose()  << "\n";
            std::cout << "X_nom.q:"  << q.transpose()  << "\n";
            std::cout << "X_nom.ba:" << ba.transpose() << "\n";
            std::cout << "X_nom.bg:" << bg.transpose() << "\n\n";
        };
    };

    struct ErrorStateCovariance{
        Vec3 cov_dp;
        Vec3 cov_dv;
        Vec3 cov_dth;
        Vec3 cov_dba;
        Vec3 cov_dbg;

        ErrorStateCovariance() {
            cov_dp  = Vec3::Zero();
            cov_dv  = Vec3::Zero();
            cov_dth = Vec3::Zero();
            cov_dba = Vec3::Zero();
            cov_dbg = Vec3::Zero();
        };
        void setValues(const CovarianceMat& cov_mat){
            cov_dp  << cov_mat(0,0),   cov_mat(1,1),   cov_mat(2,2);
            cov_dv  << cov_mat(3,3),   cov_mat(4,4),   cov_mat(5,5);
            cov_dth << cov_mat(6,6),   cov_mat(7,7),   cov_mat(8,8);
            cov_dba << cov_mat(9,9),   cov_mat(10,10), cov_mat(11,11);
            cov_dbg << cov_mat(12,12), cov_mat(13,13), cov_mat(14,14);
        };
        void show(){
            std::cout << "cov.dp:"  << cov_dp.transpose() << "\n";
            std::cout << "cov.dv:"  << cov_dv.transpose() << "\n";
            std::cout << "cov.dth:" << cov_dth.transpose() << "\n";
            std::cout << "cov.dba:" << cov_dba.transpose() << "\n";
            std::cout << "cov.dbg:" << cov_dbg.transpose() << "\n\n";
        };
    };

    struct ErrorState{
        Vec3 dp;
        Vec3 dv;
        Vec3 dth;
        Vec3 dba;
        Vec3 dbg;
        
        ErrorStateCovariance covariance;

        ErrorState() {
            dp  = Vec3::Zero();
            dv  = Vec3::Zero();
            dth = Vec3::Zero();
            dba = Vec3::Zero();
            dbg = Vec3::Zero();
            covariance.setValues(CovarianceMat::Identity()*0.005);
        };
        ErrorState(const ErrorState& dX, const CovarianceMat& cov_mat){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
            covariance.setValues(cov_mat);
        };
        void replace(const ErrorState& dX){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
        };
        void replace(const ErrorState& dX, const CovarianceMat& cov_mat){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
            covariance.setValues(cov_mat);
        };
        void replace(const ErrorStateVec& dX_vec){
            dp = dX_vec.block<3,1>(0,0);
            dv = dX_vec.block<3,1>(3,0);
            dth = dX_vec.block<3,1>(6,0);
            dba = dX_vec.block<3,1>(9,0);
            dbg = dX_vec.block<3,1>(12,0);
        };
        void replace(const ErrorStateVec& dX_vec, const CovarianceMat& cov_mat){
            dp = dX_vec.block<3,1>(0,0);
            dv = dX_vec.block<3,1>(3,0);
            dth = dX_vec.block<3,1>(6,0);
            dba = dX_vec.block<3,1>(9,0);
            dbg = dX_vec.block<3,1>(12,0);
            covariance.setValues(cov_mat);
        };
        ErrorStateCovariance getCovariance(){
            return covariance;
        };  

        ErrorStateVec getVectorform() const {
            ErrorStateVec vec;
            vec << dp, dv, dth, dba, dbg;
            return vec;
        };
    };

    struct Measurement{
        Vec3 acc;
        Vec3 gyro;
        Measurement(){
            acc  = Vec3::Zero();
            gyro = Vec3::Zero();
        };
        // When using measurement, 
    };

    struct Observation{
        Vec3 p_optitrack;
        Vec4 q_optitrack;
        Observation(){
            p_optitrack = Vec3::Zero();
            q_optitrack = Vec4::Zero(); q_optitrack(0) = 1.0;
        };
    };

    struct ProcessNoise{
        double sig_na; // acceleration measurement noise (0.0008)
        double sig_ng; // gyro measurement noise (0.000006)
        double sig_nba; // acc. bias noise (1e-15)
        double sig_nbg; // gyro bias noise (1e-15)
        int dim;
        QMat Q;
        ProcessNoise() : 
        // sig_na(0.0008), sig_ng(0.000006), sig_nba(1e-12), sig_nbg(1e-12), dim(12) {
        sig_na(0.009), sig_ng(0.00006), sig_nba(1e-9), sig_nbg(1e-9), dim(12) {
            Q = QMat::Identity();
            for(int i = 0; i < 3; ++i){
                Q(i,i) = POW2(sig_na);
                Q(3+i,3+i) = POW2(sig_ng);
                Q(6+i,6+i) = POW2(sig_nba);
                Q(9+i,9+i) = POW2(sig_nbg);
            }
        };

        void setNoise(double noise_acc, double noise_gyro, double noise_ba, double noise_bg)
        {
            if(noise_acc <= 0.0000001) 
                throw std::runtime_error("sig_na should be larger then 0.0000001.");
                
            if(noise_gyro <= 0.0000001) 
                throw std::runtime_error("sig_ng should be larger then 0.0000001.");
            
            if(noise_ba <= 0.0) 
                throw std::runtime_error("sig_nba should be larger then 0.0.");
            
            if(noise_bg <= 0.0) 
                throw std::runtime_error("sig_nbg should be larger then 0.0.");
                
            sig_na  = noise_acc;
            sig_ng  = noise_gyro;
            sig_nba = noise_ba;
            sig_nbg = noise_bg;

            Q = QMat::Identity();
            for(int i = 0; i < 3; ++i){
                Q(i,i) = POW2(sig_na);
                Q(3+i,3+i) = POW2(sig_ng);
                Q(6+i,6+i) = POW2(sig_nba);
                Q(9+i,9+i) = POW2(sig_nbg);
            }
        };
    };

    struct MeasurementNoise{
        double sig_p; // optitrack position noise // 0.005 (5 mm)
        double sig_q; // quaternion noise // 0.015 ( 0.015 rad)
        int dim;
        RMat R; 
        MeasurementNoise() : sig_p(0.005), sig_q(0.01), dim(7) { 
            R = RMat::Identity();
            for(int i = 0; i < 3; ++i) R(i,i) = POW2(sig_p);
            for(int i = 0; i < 4; ++i) R(3+i,3+i) = POW2(sig_q);
        };

        void setNoise(double noise_position, double noise_orientation){
            if(noise_position <= 0.0000001) 
                throw std::runtime_error("noise_position should be larger then 0.0000001.");
                
            if(noise_orientation <= 0.0000001) 
                throw std::runtime_error("noise_orientation should be larger then 0.0000001.");
            
            sig_p = noise_position;
            sig_q = noise_orientation;

            R = RMat::Identity();
            for(int i = 0; i < 3; ++i) R(i,i) = POW2(sig_p);
            for(int i = 0; i < 4; ++i) R(3+i,3+i) = POW2(sig_q);
        };
    };

    struct EmergencyResetRules{ // for Emergency state changer.
        double thres_quaternion; // in radian
        double thres_position;   // in meters
        double thres_cov_p;      // in meter^2
        double thres_cov_v;      // in m/2^2
        double thres_cov_q;      // in ???

        EmergencyResetRules(){
            // default values
            thres_position    = 0.03;
            thres_quaternion  = cos(0.05); // 0.05 radians == 2.645916 degrees
            thres_cov_p       = 1.0;
            thres_cov_v       = 1.0;
            thres_cov_q       = 1.0;
        };
        bool isPositionInRange(const NominalState& X_nom){
            double position_limit[2] = {-10.0, 10.0};
            bool isOK=true;
            if(X_nom.p(0) >= position_limit[0] && X_nom.p(0) <= position_limit[1] &&
               X_nom.p(1) >= position_limit[0] && X_nom.p(1) <= position_limit[1] &&
               X_nom.p(2) >= position_limit[0] && X_nom.p(2) <= position_limit[1]){
                isOK = true;
            }
            else isOK = false;

            return isOK;
        };
        bool isStateOK(const Vec3& p_measure, const Vec4& q_measure, const NominalState& X_nom){            
            // check position
            Vec3 diff_p = p_measure - X_nom.p;
            if(diff_p.norm() >= thres_position) {
                std::cout << "==========diff p: " << diff_p.norm() <<" / thres:" << thres_position << std::endl;
                return false;
            }

            // check quaternion
            Vec4 diff_q = geometry::q1_mult_q2(q_measure, geometry::q_conj(X_nom.q));
            if(diff_q(0) <= thres_quaternion) {
                std::cout << "==========diff q: " << diff_q(0) <<" / thres:" << thres_quaternion  << std::endl;
                return false;
            }

            // check covariance
            // NOT IMPLEMENTED...

            return true;    
        };
    };


private:
    FixedParameters fixed_param_;

    Vec3 ba_init_;
    Vec3 bg_init_;

    NominalState X_nom_;
    ErrorState   dX_;
    CovarianceMat P_;

    ProcessNoise process_noise_;
    MeasurementNoise measurement_noise_;
    
    EmergencyResetRules emergency_reset_rules_;
    
    Matrix<double,15,12> Fi_;

    double t_prev_;
    double t_init_;

    bool isInitialized_;


    // LPF for angular rate 
    LowPassFilter<Vec3>* lpf_gyro_;
    LowPassFilter<Vec3>* lpf_acc_;

public:

};

#endif