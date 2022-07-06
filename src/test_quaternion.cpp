#include <iostream>
#include <Eigen/Dense>
#include "error_state_kalman_filter.h"

using namespace Eigen;
int main(int argc, char **argv) {

	try{
        ESKF eskf;
        ESKF::NominalState X_nom;
        X_nom.p << 0.016663507,-0.00197155,-0.02029580;
        X_nom.v << 0,0,0;
        X_nom.q << -0.999995172,-0.00225203,0.001940096,0.000904628;
        X_nom.ba << 0.011,0.008,0.201;
        X_nom.bg << 0.004358589, -0.00117578, -0.01167082;

        Vec3 am(0.083957598, -0.17998086, -9.61113542);
        Vec3 wm(0.003904163, -0.000661324, -0.01211664);

        double dt = 0.005151271;

        // q2r
        Mat33 R = geometry ::q2r(X_nom.q);
        std::cout << "R:\n" << R<< std::endl;

        // skewMat
        Vec3 w(-1.446789,2.231,3.65451);
        std::cout << "skewMat:\n" << geometry::skewMat(w) << std::endl;

        // rotvec2q
        Vec4 q;
        std::cout << "rotvec2q:\n" << (q = geometry::rotvec2q(w)) << std::endl;

        Vec3 w1(1.678,-6.33,0.2521);
        Vec4 q1 = geometry::rotvec2q(w1);
        
        // qRightMult
        std::cout << "q_right_mult:\n" << geometry::q_right_mult(q) << std::endl;

        // qLeftMult
        std::cout << "q_left_mult:\n" << geometry::q_left_mult(q) << std::endl;

        // q_conj
        std::cout << "q_conj:\n" << geometry::q_conj(q) <<std::endl;

        // q1Mulq2
        std::cout << "q1_mult_q2:\n" << geometry::q1_mult_q2(q,q1) << std::endl;
    }
	catch (std::exception& e){
        std::cout << e.what();
	}

	return 0;
}