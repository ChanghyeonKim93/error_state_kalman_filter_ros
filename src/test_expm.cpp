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

        FMat F;
        expmFMat exp_F;
        eskf.test_FMat(X_nom,am, wm, F);

        int max_order = 1;
        expmFMat exp_F1, exp_F2, exp_F3, exp_F4, exp_F5;
        eskf.test_expm_FMat(F, dt, 1, exp_F1);
        eskf.test_expm_FMat(F, dt, 1, exp_F2);
        eskf.test_expm_FMat(F, dt, 1, exp_F3);
        eskf.test_expm_FMat(F, dt, 1, exp_F4);
        eskf.test_expm_FMat(F, dt, 1, exp_F5);

        std::cout << " F5-F1:\n";
        std::cout <<exp_F5-exp_F1 << std::endl; 
        std::cout << " F5-F2:\n";
        std::cout <<exp_F5-exp_F2 << std::endl; 
        std::cout << " F5-F3:\n";
        std::cout <<exp_F5-exp_F3 << std::endl; 
        std::cout << " F5-F4:\n";
        std::cout <<exp_F5-exp_F4 << std::endl; 
        
    }
	catch (std::exception& e){
        std::cout << e.what();
	}

	return 0;
}