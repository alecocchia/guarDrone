#pragma once
#include <Eigen/Dense>


struct QM {
    
    Eigen::Matrix<double,3,3> Q;
    
};

struct RM {
    
    Eigen::Matrix<double,3,3> R;
    
};

struct wedg {
    
    Eigen::Vector3d v;
    
};

QM QMatrix(
    double phi, double theta);


RM RMatrix(
    double phi, double theta, double psi);

wedg wedge(Eigen::Matrix<double,3,3> S);