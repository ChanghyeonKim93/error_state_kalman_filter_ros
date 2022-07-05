#ifndef _ERROR_STATE_KALMAN_FILTER_H_
#define _ERROR_STATE_KALMAN_FILTER_H_

#include <iostream>
#include <Eigen/Dense>

#include "geometry_library.h"

using namespace Eigen;
typedef Matrix<double,3,1> Vec3;
typedef Matrix<double,4,1> Vec4;
typedef Matrix<double,3,3> Mat33;
typedef Matrix<double,4,4> Mat44;
typedef Matrix<double,3,4> Mat34;
typedef Matrix<double,4,3> Mat43;

typedef Matrix<double,15,15> CovarianceMat;
typedef Matrix<double,15,15> FMat;

#define GRAVITY_MAGNITUDE 9.81

#define POW2(x) ((x)*(x))

class ESKF{
    struct FixedParameters;
    struct NominalStateIndex;
    struct ErrorStateIndex;
    struct NominalState;
    struct ErrorState;
    struct EstmiatedState;
    struct Measurement;
    struct Observation;
    struct ProcessNoise;
    struct MeasurementNoise;

private:
    // Related to Kalman filter implementation .
    void updateNominal(const NominalState& X_nom, const Vec3& am, const Vec3& wm, double dt,
                       NominalState& X_nom_update);
    void updateError(const NominalState& X_nom, const ErrorState& dX, const Vec3& am, const Vec3& wm, double dt,
                       ErrorState& dX_update);

    void errorStateF(const NominalState& X_nom, const ErrorState& dX, const Vec3& am, const Vec3& wm,
                       FMat& res);

public:
    ESKF();
    ~ESKF();

    void predict(double ax, double ay, double az, 
                 double wx, double wy, double wz, double t_now); // by imu 
    void update(); // by optitrack
    
    void resetFilter();

private:
    static Mat33 I33;
    static Mat44 I44;
    static Mat33 O33;
    static Mat44 O44;
    static Mat34 O34;
    static Mat43 O43;

    struct FixedParameters{
        Mat33 R_BI; // SO(3), rotation only. drone body frame (== optitrack coordinate frame) 
        // to the IMU frame
        Mat33 R_IB; // R_BI.transpose();
        Vec4  q_BI; // quaternion, rotation only. drone body frame to IMU frame
        Vec4  q_IB; // q_BI.conjugate();
        
        Vec3 grav; // gravity w.r.t. the global frame

        FixedParameters(){
            R_BI << 1,0,0, 0,-1,0, 0,0,-1;
            R_IB = R_BI.transpose();
            q_BI << 0,-1,0,0;
            q_IB << 0, 1,0,0;

            grav << 0.0, 0.0, -GRAVITY_MAGNITUDE;
        };
    };

    struct NominalStateIndex{
        uint32_t p[2];
        uint32_t v[2];
        uint32_t q[2];
        uint32_t ba[2];
        uint32_t bg[2];
        NominalStateIndex(){
            p[0]=0; p[1]=2;
            v[0]=3; v[1]=5;
            q[0]=6; q[1]=9;
            ba[0]=10; ba[1]=12;
            bg[0]=13; bg[1]=15;
        };
    };

    struct ErrorStateIndex{
        uint32_t dp[2];
        uint32_t dv[2];
        uint32_t dth[2];
        uint32_t dba[2];
        uint32_t dbg[2];
        ErrorStateIndex(){
            dp[0]=0; dp[1]=2;
            dv[0]=3; dv[1]=5;
            dth[0]=6; dth[1]=8;
            dba[0]=9; dba[1]=11;
            dbg[0]=12; dbg[1]=14;
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
        void replace(const NominalState& nom){
            p  = nom.p;
            v  = nom.v;
            q  = nom.q;
            ba = nom.ba;
            bg = nom.bg;
        };
    };

    struct ErrorState{
        Vec3 dp;
        Vec3 dv;
        Vec3 dth;
        Vec3 dba;
        Vec3 dbg;
        ErrorState() {
            dp  = Vec3::Zero();
            dv  = Vec3::Zero();
            dth = Vec3::Zero();
            dba = Vec3::Zero();
            dbg = Vec3::Zero();
        };
        ErrorState(const ErrorState& dX){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
        };
        void replace(const ErrorState& dX){
            dp  = dX.dp;
            dv  = dX.dv;
            dth = dX.dth;
            dba = dX.dba;
            dbg = dX.dbg;
        };
    };

    struct EstmiatedState{
        Vec3 p; // w.r.t. global frame
        Vec3 v; // w.r.t. global frame
        Vec4 q; // w.r.t. global frame
        Vec3 w; // w.r.t. body frame
        EstmiatedState(){
            p = Vec3::Zero();
            v = Vec3::Zero();
            q = Vec4::Zero(); q(0) = 1.0;
            w = Vec3::Zero();
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
        Matrix<double, 12,12> Q;
        ProcessNoise() : 
        sig_na(0.0008), sig_ng(0.000006), sig_nba(1e-12), sig_nbg(1e-12), dim(12) {
            Q = Matrix<double, 12,12>::Identity();
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
        Matrix<double, 7, 7> R; 
        MeasurementNoise() : sig_p(0.005), sig_q(0.015), dim(7) { 
            R = Matrix<double, 7,7>::Identity();
            for(int i = 0; i < 3; ++i) R(i,i) = POW2(sig_p);
            for(int i = 0; i < 4; ++i) R(3+i,3+i) = POW2(sig_q);
        };
    };

    FixedParameters fixed_param_;

    NominalState X_nom_;
    ErrorState   dX_;
    CovarianceMat P_;

    ProcessNoise process_noise_;
    MeasurementNoise measurement_noise_;
    

    Matrix<double,15,12> Fi_;

    double t_prev_;

public:

};

#endif