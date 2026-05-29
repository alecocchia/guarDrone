#include "cbf.hpp"
#include <cmath>
#include "UtilitiesDrones.hpp"

CBFObsDron cbf_obs_dist_Dron(
    const Eigen::Vector2d& xobs,
    const Eigen::VectorXd& q,   // size 12: [x,y,z,r,p,yaw, x2,y2,z2,r2,p2,yaw2]
    int tk,
    double R_r,
    double r)
{
    // Extract 2D position
    Eigen::Vector2d x = q.head<2>();

    // J = [1 0 0 0 0 0; 0 1 0 0 0 0]  (2x6)
    Eigen::Matrix<double, 2, 6> J = Eigen::Matrix<double, 2, 6>::Zero();
    J(0, 0) = 1.0;
    J(1, 1) = 1.0;

    // Obstacle velocity


    // CBF value
    CBFObsDron res;
    res.h = (x - xobs).squaredNorm() - R_r * R_r - r * r-2*r*R_r;

    // Gradient of h w.r.t. position
    Eigen::RowVector2d grad = 2.0 * (x - xobs).transpose();

    res.dh_dq.setZero();    // size 12
    res.dh_dt_u.setZero();  // size 12

    if (tk == 1) {
        // dh_dq: first 6 entries  →  grad * J  (1x6)
        res.dh_dq.segment<6>(0) = grad * J;
        // dh_dt_u: first 2 entries  →  grad * R_2d  (1x2)
        res.dh_dt_u.segment<2>(0) = grad;
    }
    else { // tk == 2
        // dh_dq: second block of 6
        res.dh_dq.segment<6>(6) = grad * J;
        // dh_dt_u: second block of 2 (offset 6 to match second drone's controls)
        //res.dh_dt_u.segment<2>(6) = grad * R_2d;
        res.dh_dt_u.segment<2>(6) = grad;
    }

    return res;
}


CBFDroneAvResult cbf_av_centr_Drone(
    const Eigen::VectorXd& dro1,
    const Eigen::VectorXd& dro2,
    double R_r,
    double r
){
    Eigen::Vector3d x1 = dro1.segment<3>(0);
    Eigen::Vector3d x2 = dro2.segment<3>(0);

    // J1 (2x12): picks x1,y1
    Eigen::Matrix<double, 3, 12> J1 = Eigen::Matrix<double, 3, 12>::Zero();
    J1(0, 0) = 1.0;
    J1(1, 1) = 1.0;
    J1(2, 2) = 1.0;

    // J2 (2x12): picks x2,y2
    Eigen::Matrix<double, 3, 12> J2 = Eigen::Matrix<double, 3, 12>::Zero();
    J2(0, 6) = 1.0;
    J2(1, 7) = 1.0;
    J2(2, 8) = 1.0;

    // J (4x12): stacked J1 and J2
    Eigen::Matrix<double, 6, 12> J;
    J.topRows<3>()    = J1;
    J.bottomRows<3>() = J2;

    // CBF value
    CBFDroneAvResult res;
    res.h = (x1 - x2).squaredNorm()/2 - R_r * R_r - r * r -2*R_r*r;

    // dh_dsigma (1x4): [(x1-x2)'  -(x1-x2)']
    Eigen::Matrix<double, 1, 6> dh_dsigma;
    dh_dsigma.segment<3>(0) =  (x1 - x2).transpose();
    dh_dsigma.segment<3>(3) = -(x1 - x2).transpose();

    // dh_dq (1x12) = dh_dsigma * J
    res.dh_dq = dh_dsigma * J;

    // dh_dt_u (1x4): [2*(x1-x2)'*R1 , -2*(x1-x2)'*R2]
    res.dh_dt_u.setZero();  // size 12, assuming same layout
//    res.dh_dt_u.segment<2>(0) =  2.0 * (x1 - x2).transpose() * R1;
//    res.dh_dt_u.segment<2>(6) = -2.0 * (x1 - x2).transpose() * R2;

    res.dh_dt_u.segment<3>(0) =   (x1 - x2).transpose();
    res.dh_dt_u.segment<3>(6) = - (x1 - x2).transpose();


    return res;
}


CBFDroneResult cbf_drone(
    const Eigen::Vector3d& xd,
    const Eigen::VectorXd& q,
    int tk
)
{
    CBFDroneResult res;

    // =========================
    // Posizione
    // =========================
    Eigen::Vector3d x = q.segment<3>(0);   // MATLAB q(1:2)

    // =========================
    // Matrici J e J1
    // =========================
    Eigen::Matrix<double, 3, 6> J;
    J << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;




    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


    res.h = -0.5 * (x - xd.segment<3>(0)).squaredNorm();


    Eigen::RowVector<double,6> grad6 =
        -(x - xd.segment<3>(0)).transpose() * J;
            

    res.dh_dq = Eigen::RowVectorXd::Zero(12);

    if (tk == 1)
    {
        // primi 6 elementi
        res.dh_dq.segment<6>(0) = grad6;
    }
    else if (tk == 2)
    {
        // ultimi 6 elementi (shift di 7 posizioni)
        res.dh_dq.segment<6>(6) = grad6;
    }

    return res;
}

CBFObsDron cbf_obs_dist_terra(
    const Eigen::VectorXd& q,   // size 12: [x,y,z,r,p,yaw, x2,y2,z2,r2,p2,yaw2]
    int tk,
    double R_r)
{
    // Extract 2D position
    double x = q(2);

    // J = [1 0 0 0 0 0; 0 1 0 0 0 0]  (2x6)
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    J(0, 2) = 1.0;
    

    // Obstacle velocity


    // CBF value
    CBFObsDron res;
    res.h = x - R_r ;

    // Gradient of h w.r.t. position
    double grad = 1;

    res.dh_dq.setZero();    // size 12
    res.dh_dt_u.setZero();  // size 12

    if (tk == 1) {
        // dh_dq: first 6 entries  →  grad * J  (1x6)
        res.dh_dq.segment<6>(0) = grad * J;
        // dh_dt_u: first 2 entries  →  grad * R_2d  (1x2)
        res.dh_dt_u(2) = grad;
    }
    else { // tk == 2
        // dh_dq: second block of 6
        res.dh_dq.segment<6>(6) = grad * J;
        // dh_dt_u: second block of 2 (offset 6 to match second drone's controls)
        //res.dh_dt_u.segment<2>(6) = grad * R_2d;
        res.dh_dt_u(8) = grad;
    }

    return res;
}

CBFObsDron cbf_obs_dist_soffitto(
    const Eigen::VectorXd& q,   // size 12: [x,y,z,r,p,yaw, x2,y2,z2,r2,p2,yaw2]
    int tk,
    double R_r)
{
    // Extract 2D position
    double x = q(2);

    // J = [1 0 0 0 0 0; 0 1 0 0 0 0]  (2x6)
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    J(0, 2) = 1.0;
    

    // Obstacle velocity


    // CBF value
    CBFObsDron res;
    res.h = -x + R_r ;

    // Gradient of h w.r.t. position
    double grad = -1;

    res.dh_dq.setZero();    // size 12
    res.dh_dt_u.setZero();  // size 12

    if (tk == 1) {
        // dh_dq: first 6 entries  →  grad * J  (1x6)
        res.dh_dq.segment<6>(0) = grad * J;
        // dh_dt_u: first 2 entries  →  grad * R_2d  (1x2)
        res.dh_dt_u(2) = grad;
    }
    else { // tk == 2
        // dh_dq: second block of 6
        res.dh_dq.segment<6>(6) = grad * J;
        // dh_dt_u: second block of 2 (offset 6 to match second drone's controls)
        //res.dh_dt_u.segment<2>(6) = grad * R_2d;
        res.dh_dt_u(8) = grad;
    }

    return res;
}


