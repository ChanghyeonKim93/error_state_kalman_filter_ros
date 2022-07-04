#ifndef _ERROR_STATE_KALMAN_FILTER_H_
#define _ERROR_STATE_KALMAN_FILTER_H_

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,3,1> Vec3;
typedef Matrix<double,4,1> Vec4;
typedef Matrix<double,3,3> Mat33;
typedef Matrix<double,4,4> Mat44;
typedef Matrix<double,3,4> Mat34;
typedef Matrix<double,4,3> Mat43;

#define POW2(x) ((x)*(x))

class ESKF{
private:
    // Related to Kalman filter implementation .

public:
    ESKF();
    ~ESKF();

    void propagate(); // by imu 
    void update(); // by optitrack

private:
    static Mat33 I33;
    static Mat44 I44;
    static Mat33 O33;
    static Mat44 O44;
    static Mat34 O34;
    static Mat43 O43;

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

        void initialize(const Vec3& pi, const Vec3& vi, const Vec4& qi, const Vec3& bai, const Vec3& bgi){
            p  = pi;
            v  = vi;
            q  = qi; 
            ba = bai;
            bg = bgi;
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
        ProcessNoise() : sig_na(0.0008), sig_ng(0.000006), sig_nba(1e-12), sig_nbg(1e-12), dim(12) {
            Q = Matrix<double, 12,12>::Identity();
            for(int i = 0; i < 3; ++i){
                Q(i,i) = POW2(sig_na);
                Q(3+i,3+i) = POW2(sig_na);
                Q(6+i,6+i) = POW2(sig_na);
                Q(9+i,9+i) = POW2(sig_na);
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

    NominalState X_nom_;
    ErrorState   dX_;

    ProcessNoise process_noise_;
    MeasurementNoise measurement_noise_;

public:

};

#endif