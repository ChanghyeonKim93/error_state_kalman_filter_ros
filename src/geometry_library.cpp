#include "geometry_library.h"
using namespace Eigen;
namespace geometry {
    Matrix3d skewMat(const Vector3d& v){
        Matrix3d res_mat;
        res_mat << 0,-v(2),v(1),
                   v(2),0,-v(0),
                  -v(1),v(0),0;
        return res_mat;
    };

    Matrix4d q_right_mult(const Vector4d& q){
        Matrix4d omega_r;
        omega_r << q(0),-q(1),-q(2),-q(3),
                   q(1), q(0), q(3),-q(2),
                   q(2),-q(3), q(0), q(1),
                   q(3), q(2),-q(1), q(0);
        return omega_r;
    };
    Matrix4d q_left_mult(const Vector4d& q){
        Matrix4d omega_l;
        omega_l << q(0),-q(1),-q(2),-q(3),
                   q(1), q(0),-q(3), q(2),
                   q(2), q(3), q(0),-q(1),
                   q(3),-q(2), q(1), q(0);
        return omega_l;
    };
    Vector4d q_conj(const Vector4d& q){
        Vector4d q_c;
        q_c << q(0), -q(1),-q(2),-q(3);
        return q_c;
    };
    Vector4d q1_mult_q2(const Vector4d& q1, const Vector4d& q2){
        Vector4d q;
        q << q1(0)*q2(0)-q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3),
             q1(0)*q2(1)+q1(1)*q2(0)+q1(2)*q2(3)-q1(3)*q2(2),
             q1(0)*q2(2)-q1(1)*q2(3)+q1(2)*q2(0)+q1(3)*q2(1),
             q1(0)*q2(3)+q1(1)*q2(2)-q1(2)*q2(1)+q1(3)*q2(0);
        return q;
    };

    Matrix3d q2r(const Vector4d& q){
        Matrix3d R;
        double qw = q(0);
        double qx = q(1);
        double qy = q(2);
        double qz = q(3);

        double qw2 = qw*qw;
        double qx2 = qx*qx;
        double qy2 = qy*qy;
        double qz2 = qz*qz;

        double qxqy = qx*qy;
        double qwqz = qw*qz;
        double qxqz = qx*qz;
        double qwqy = qw*qy;
        double qwqx = qw*qx;
        double qyqz = qy*qz;

        R <<  qw2+qx2-qy2-qz2, 2.0*(qxqy-qwqz), 2.0*(qxqz+qwqy),
              2.0*(qxqy+qwqz), qw2-qx2+qy2-qz2, 2.0*(qyqz-qwqx),
              2.0*(qxqz-qwqy), 2.0*(qyqz+qwqx), qw2-qx2-qy2+qz2;    

        return R;
    };

    Vector4d rotvec2q(const Vector3d& w){
        Vector4d q_res;
        double th = w(0)*w(0) + w(1)*w(1) + w(2)*w(2);
        th = std::sqrt(th);
        if(th < 1e-7){
            q_res << 1.0, 0.0, 0.0, 0.0;
        }
        else{
            double invthsinth05 = sin(th*0.5)/th;
            q_res << cos(th*0.5),w(0)*invthsinth05, w(1)*invthsinth05, w(2)*invthsinth05;
            q_res /= q_res.norm();
        }
        return q_res;
    };

    Matrix3d a2r(double r, double p, double y){
        Matrix3d Rx;
        Matrix3d Ry;
        Matrix3d Rz;

        Rx << 1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r);
        Ry << cos(p),0,sin(p),0,1,0,-sin(p),0,cos(p);
        Rz << cos(y),-sin(y),0,sin(y),cos(y),0,0,0,1;

        return (Rz*Ry*Rx);
    };

    Matrix4d expm_FMat(const FMat& F, const double& dt){
        Matrix4d expmF;
        double dt2 = dt*dt;
        double dt3 = dt2*dt;
        double dt4 = dt3*dt;
        double dt5 = dt4*dt;
        // fourth order approx?
        // expmF = I + F0*dt + 0.5*F0F0*dtdt  + 1/6*F0F0F0*dtdtdt
        //           + 1/24*F0F0F0F0*dtdtdtdt + 1/120*F0F0F0F0F0*dtdtdtdtdt;

        Matrix3d skW     =  F.block<6,6>(3,3);
        Matrix3d skW2    =  skW *skW;
        Matrix3d skW3    =  skW2*skW;
        Matrix3d skW4    =  skW3*skW;

        Matrix3d RBI_skA       = -F.block<3,6>(3,3);
        Matrix3d RBI_skA_skW   = RBI_skA*skW;
        Matrix3d RBI_skA_skW2  = RBI_skA_skW*skW;
        Matrix3d RBI_skA_skW3  = RBI_skA_skW2*skW;
        Matrix3d RBI_skA_skW4  = RBI_skA_skW3*skW;

        double mul1 = 0.5;
        double mul2 = 1.0/6.0;
        double mul3 = 1.0/24.0;
        double mul4 = 1.0/120.0;
        double mul5 = 1.0/720.0;

        Matrix3d RBI       = -F.block<3,9>(3,3);

        // R_BIskewA = R_BI*skewA;
        // skewW2 = skewW*skewW;
        // skewW3 = skewW2*skewW;
        // skewW4 = skewW3*skewW;
        // skewW5 = skewW4*skewW;

        // if skewW ~ 10
        // skewW * dt ~ 10*1e-3 = 1e-2
        // skewW2 * dt2 ~ 1e-4
        // skewW3 * dt3 ~ 1e-6
        // skewW4 * dt4 ~ 1e-8
        // skewW5 * dt5 ~ 1e-10

        // F0 =...
        // [0, 1,           0,     0,  0;...
        //  0, 0, -R_BI*skewA, -R_BI,  0;...
        //  0, 0,      -skewW,     0, -1;...
        //  0, 0,           0,     0,  0;...
        //  0, 0,           0,     0,  0];
        
        // F0F0 =...
        // [0, 0,      -R_BI*skewA, -R_BI,          0;...
        //  0, 0, R_BI*skewA*skewW,     0, R_BI*skewA;...
        //  0, 0,          skewW^2,     0,      skewW;...
        //  0, 0,                0,     0,          0;...
        //  0, 0,                0,     0,          0];

        // F0F0F0 =...
        // [0, 0,    R_BI*skewA*skewW, 0,        R_BI*skewA;...
        //  0, 0, -R_BI*skewA*skewW^2, 0, -R_BI*skewA*skewW;...
        //  0, 0,            -skewW^3, 0,          -skewW^2;...
        //  0, 0,                   0, 0,                 0;...
        //  0, 0,                   0, 0,                 0];

        // F0F0F0F0 =...
        // [0, 0, -R_BI*skewA*skewW^2, 0,  -R_BI*skewA*skewW;...
        //  0, 0,  R_BI*skewA*skewW^3, 0, R_BI*skewA*skewW^2;...
        //  0, 0,             skewW^4, 0,            skewW^3;...
        //  0, 0,                   0, 0,                  0;...
        //  0, 0,                   0, 0,                  0];


        // F0F0F0F0F0=... 
        // [0, 0,  R_BI*skewA*skewW^3, 0,  R_BI*skewA*skewW^2;...
        //  0, 0, -R_BI*skewA*skewW^4, 0, -R_BI*skewA*skewW^3;...
        //  0, 0,            -skewW^5, 0,            -skewW^4;...
        //  0, 0,                   0, 0,                   0;...
        //  0, 0,                   0, 0,                   0];
        return expmF;
    };
};
