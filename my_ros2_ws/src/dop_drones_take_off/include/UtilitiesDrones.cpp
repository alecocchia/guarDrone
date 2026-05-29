#include "UtilitiesDrones.hpp"
#include <cmath>

QM QMatrix(
    double phi, double theta){
        QM res;
        res.Q<< 1,0,-sin(phi),0,cos(phi),cos(theta)*sin(phi),0,-sin(phi),cos(theta)*cos(phi);
        return res;

    };


RM RMatrix(
    double phi, double theta, double psi)
    {
        RM res;
        res.R << 
        cos(theta) * cos(psi),                        sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),   cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi),
        cos(theta) * sin(psi),                        sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi),   cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi),
        -sin(theta),                                  sin(phi) * cos(theta),                                     cos(phi) * cos(theta);

    return res;


    };

wedg wedge(Eigen::Matrix<double,3,3> S){
    wedg res;

    res.v << S(2,1),S(0,2),S(1,0);
    return res;

};