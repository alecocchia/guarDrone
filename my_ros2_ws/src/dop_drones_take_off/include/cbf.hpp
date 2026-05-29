#pragma once
#include <Eigen/Dense>



struct CBFDroneResult {
    double h;
    Eigen::RowVector<double,12> dh_dq;
};


struct CBFDroneAvResult {
    double h;
    Eigen::RowVector<double,12> dh_dq;
    Eigen::RowVector<double,12> dh_dt_u;
    double dh_dt_b;
};

struct CBFObsDron {
    double h;
    Eigen::RowVector<double,12> dh_dq;
    Eigen::RowVector<double,12> dh_dt_u;
    double dh_dt_b;
};

struct CBFAvResult {
    double h;
    Eigen::RowVector<double,8> dh_dq;
    Eigen::RowVector<double,6> dh_dt_u;
};


struct CBFResultTeleopObs {
    double h;
    Eigen::RowVector<double,8> dh_dq;
    Eigen::RowVector<double,6> dh_dt_u;
    double dh_dt_b;
};



CBFDroneResult cbf_drone(
    const Eigen::Vector3d& xd,
    const Eigen::VectorXd& q,
    int tk
);


CBFObsDron cbf_obs_dist_Dron(
    const Eigen::Vector2d& xobs,
    const Eigen::VectorXd& q,
    int tk,
    double R,
    double r
);





CBFDroneAvResult cbf_av_centr_Drone(
    const Eigen::VectorXd& dro1,
    const Eigen::VectorXd& dro2,
    double R_r,
    double r
);

CBFObsDron cbf_obs_dist_terra(
    const Eigen::VectorXd& q,   // size 12: [x,y,z,r,p,yaw, x2,y2,z2,r2,p2,yaw2]
    int tk,
    double R_r);

CBFObsDron cbf_obs_dist_soffitto(
    const Eigen::VectorXd& q,   // size 12: [x,y,z,r,p,yaw, x2,y2,z2,r2,p2,yaw2]
    int tk,
    double R_r);
