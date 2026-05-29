#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "cbf.hpp"
#include "UtilitiesDrones.hpp"

#include <stdint.h>
#include <cmath>
#include <array>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace Eigen;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace px4_ros_com::frame_transforms;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto px4_qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // ── SUBSCRIBER — scrive current_pos_ ─────────────────────────
        subscriber_drone1 = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_1/fmu/out/vehicle_odometry", px4_qos,
            std::bind(&OffboardControl::on_odometry_drone1, this, std::placeholders::_1)
        );

        subscriber_drone2 = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_2/fmu/out/vehicle_odometry", px4_qos,
        std::bind(&OffboardControl::on_odometry_drone2, this, std::placeholders::_1)
        );

        offboard_setpoint_counter_ = 0;

        // ── PUBLISHER ────────────────────────────────────────────────
        offboard_control_mode_publisher_drone1 =
            this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_drone1 =
            this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_drone1 =
            this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);

        offboard_control_mode_publisher_drone2 =
            this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_drone2 =
            this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_drone2 =
            this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);


        start_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    	"/Cbf_controll/start", 10,
    	std::bind(&OffboardControl::on_start, this, std::placeholders::_1)
		);

		// Publisher: manda lo stato al BT
		status_publisher_ = this->create_publisher<std_msgs::msg::String>(
    	"/Cbf_controll/status", 10
		);

		RCLCPP_INFO(this->get_logger(), "Nodo pronto. Aspetto /Cbf_controll/start ...");
	    

        //inizzializzare il solver
        if(solver_ptr) { delete solver_ptr; }
        solver_ptr  = new qpOASES::SQProblem(NVARS, NCONS);
        first_solve = true;

        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        solver_ptr->setOptions(options);


        // ── TIMER PLANNER — 5 Hz ─────────────────────────────────────
        // Legge current_pos_ e aggiorna cmd_vel_.
        // Gira più lento: pubblica_timer manda lo stesso cmd_vel_ più volte
        // finché il planner non produce un nuovo valore — va bene.
        planner_timer_ = this->create_wall_timer(100ms, [this]() {
            compute_velocity_command();


        });
    }


    void disarm();

private:
    // ── TIMER ────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr planner_timer_;

    // ── PUBLISHER / SUBSCRIBER ───────────────────────────────────────
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscriber_drone1;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_drone1;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_drone1;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_drone1;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscriber_drone2;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_drone2;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_drone2;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_drone2;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   start_subscriber_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    status_publisher_;

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_ = 0;

    // ── STATO CONDIVISO ──────────────────────────────────────────────
    // Scritto da on_odometry, letto da compute_velocity_command.
    // Scritto da compute_velocity_command, letto da publish_trajectory_setpoint.
    // Thread-safe con spin() single-threaded: le callback non si sovrappongono.
    //solver 
    qpOASES::SQProblem *solver_ptr = nullptr;
    bool first_solve = true;

    //variabili problema ottimizzazione 
    static const int N  = 12;   // variabili di controllo
    static const int M  = 9;   // numero task CBF
    static const int NVARS = N + M + (M-1);   // = 11
    static const int NCONS = M + (M-1);        // = 5

    
    
    double q[12]{};
    VectorXd u_last = VectorXd::Zero(12);
    bool running_ = false; 
    int n=1;
                     



    // ── CALLBACKS ────────────────────────────────────────────────────
    void on_odometry_drone1(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        if (std::isnan(msg->position[0]) ||
            std::isnan(msg->position[1]) ||
            std::isnan(msg->position[2]))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Posizione non disponibile (NaN)");
            return;
        }

        // Aggiorna lo stato — il planner lo leggerà al prossimo tick

        Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);

        Eigen::Vector3d pos_enu = enu_to_ned_local_frame(pos_ned);
        q[0] = pos_enu(0)+6;
        q[1] = pos_enu(1)+7;
        q[2] = pos_enu(2);
        Eigen::Quaterniond q_ned(
            msg->q[0],   // w
            msg->q[1],   // x
            msg->q[2],   // y
            msg->q[3]    // z
            );
        Eigen::Quaterniond q_enu = enu_to_ned_orientation(q_ned);
        double qw = q_enu.w();
        double qx = q_enu.x();   
        double qy = q_enu.y();
        double qz = q_enu.z();
        q[3] = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)); // roll
        q[4] = asin( 2*(qw*qy - qz*qx));                          // pitch
        q[5] = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)); // yaw
    }

    void on_odometry_drone2(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        if (std::isnan(msg->position[0]) ||
            std::isnan(msg->position[1]) ||
            std::isnan(msg->position[2]))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Posizione non disponibile (NaN)");
            return;
        }

        // Aggiorna lo stato — il planner lo leggerà al prossimo tick

        Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);

        Eigen::Vector3d pos_enu = enu_to_ned_local_frame(pos_ned);
        q[6] = pos_enu(0)+7;
        q[7] = pos_enu(1)-8;
        q[8] = pos_enu(2);
        Eigen::Quaterniond q_ned(
            msg->q[0],   // w
            msg->q[1],   // x
            msg->q[2],   // y
            msg->q[3]    // z
            );
        Eigen::Quaterniond q_enu = enu_to_ned_orientation(q_ned);
        double qw = q_enu.w();
        double qx = q_enu.x();   
        double qy = q_enu.y();
        double qz = q_enu.z();
        q[9] = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)); // roll
        q[10] = asin( 2*(qw*qy - qz*qx));                          // pitch
        q[11] = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)); // yaw
    }

        


void on_start(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !running_) {   // solo se true e non già partito
        RCLCPP_INFO(this->get_logger(), "Ricevuto start! Avvio takeoff...");
        running_ = true;
        offboard_setpoint_counter_ = 0;
        

        // Ora creo il timer — stesso callback che avevi prima
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                n=2;
            }
            this->publish_vehicle_command_drone1(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->publish_vehicle_command_drone2(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            
            publish_offboard_control_mode_drone1(n);
            publish_trajectory_setpoint_drone1(n);

            publish_offboard_control_mode_drone2(n);
            publish_trajectory_setpoint_drone2(n);

            publish_status("RUNNING");
            offboard_setpoint_counter_++;
        };
        publish_timer_ = this->create_wall_timer(100ms, timer_callback);
    }
}

void publish_status(const std::string & s)
{
    std_msgs::msg::String msg;
    msg.data = s;
    status_publisher_->publish(msg);
}    

    // Planner: legge current_pos_, scrive cmd_vel_
    void compute_velocity_command()
    {
    double l_u = 1e-2;
    double l_delta = 1e-1;
    double l_v = 1e-1;
    double k = 1e4;
    double u_max = 3;

    // Task goals (hardcoded come nel tuo main — puoi metterli in IDS)
    Vector3d task1; task1<<7,-8,3.5;
    Vector3d task2; task2 << -3,-5,0.0;
    Vector3d task3; task3 <<6,7,1.0;
    Vector3d task4; task4 <<4,-2,0.0;
    Vector2d task5; task5 <<0,0;

    /* --- 3. Mappa VectorXd sugli array IDS ---------------------------- */
        Map<VectorXd> q_vec(q, 12);

    /* --- 4. Calcola CBF ----------------------------------------------- */
        auto t1 = cbf_drone(task1, q_vec.segment<6>(0),1);
        //auto t2 = cbf_drone(task2, q_vec.segment<6>(0),1);
        auto t3 = cbf_drone(task3, q_vec.segment<6>(6),2);
        //auto t4 = cbf_drone(task4, q_vec.segment<6>(6),2);
        auto t5 = cbf_obs_dist_Dron(task5, q_vec.segment<6>(0),1,2.0,1.0);
        auto t6 = cbf_obs_dist_Dron(task5, q_vec.segment<6>(6),2,2.0,1.0);
        auto t7 = cbf_av_centr_Drone(q_vec.segment<6>(0), q_vec.segment<6>(6),0.60,0.60);
        auto t8 = cbf_obs_dist_terra(q_vec.segment<6>(0), 1,0.75);
        auto t9 = cbf_obs_dist_terra(q_vec.segment<6>(6), 2,0.75);
        auto t10 = cbf_obs_dist_soffitto(q_vec.segment<6>(0), 1,4.0);
        auto t11 = cbf_obs_dist_soffitto(q_vec.segment<6>(6), 2,4.0);
        
        

    VectorXd h(M);
     h << t1.h,t3.h,t5.h,t6.h,t7.h,t8.h,t9.h,t10.h,t11.h;

    MatrixXd dh_dq(M, 12);

    dh_dq << t1.dh_dq,
            t3.dh_dq,
            t5.dh_dq,
            t6.dh_dq,
            t7.dh_dq,
            t8.dh_dq,
            t9.dh_dq,
            t10.dh_dq,
            t11.dh_dq;

    auto gamma = [](double s){ return 0.20 * s; };
    auto gamma2 = [](double s){ return 0.7 * s; };
    auto gamma3 = [](double s){ return 1.5 * s; };
    VectorXd gammaP(M);
    gammaP << gamma(t1.h),
                  gamma(t3.h),
                  gamma2(t5.h),gamma2(t6.h),
                  gamma2(t7.h),gamma3(t8.h),
                  gamma3(t9.h),gamma3(t10.h),
                  gamma3(t11.h);


    VectorXd dh_dt_b = VectorXd::Zero(M);
    MatrixXd dh_dt_u = MatrixXd::Zero(M, N);

    dh_dt_u.row(2) = t5.dh_dt_u;
    dh_dt_u.row(3) = t6.dh_dt_u;
    dh_dt_u.row(4) = t7.dh_dt_u;
    
    
    /* --- 6. Matrici QP ------------------------------------------------ */

    MatrixXd V = MatrixXd::Zero(8,8);
    V.diagonal() << 1000/k,1000/k,1000/k,10/k,1000/k,1000/k,1000/k,10/k; //modificare 

    MatrixXd K1(M-1, M);

    K1 <<    0,0,0,0,0,999/k,0,1,0,
             0,0,-100/k,0,0,1,0,0,0, 
             0,0,1,0,-100/k,0,0,0,0,
             -10/k,0,0,0,1,0,0,0,0,
              0,0,0,0,0,0,-999/k,0,0,
              0,0,0,-100/k,0,0,1,0,0,
              0,0,0,1,-100/k,0,0,0,0,
              0,-10/k,0,0,1,0,0,0,0;

    MatrixXd A_full = MatrixXd::Zero(NCONS, NVARS);
    A_full.block(0,  0,   M,   N)   = -dh_dq -dh_dt_u;
    A_full.block(0,  N,   M,   M)   = -MatrixXd::Identity(M, M);
    A_full.block(M,  N,   M-1, M)   = K1;
    A_full.block(M,  N+M, M-1, M-1) = -V;

    MatrixXd H_mat = MatrixXd::Zero(NVARS, NVARS);
    H_mat.block(0,   0,   N,   N)   = l_u    * MatrixXd::Identity(N,   N);
    H_mat.block(N,   N,   M,   M)   = l_delta * MatrixXd::Identity(M,   M);
    H_mat.block(N+M, N+M, M-1, M-1) = l_v     * MatrixXd::Identity(M-1, M-1);

    VectorXd f_vec  = VectorXd::Zero(NVARS);
    VectorXd lb_var = VectorXd::Constant(NVARS, -1e20);
    VectorXd ub_var = VectorXd::Constant(NVARS,  1e20);
    lb_var.head(N).setConstant(-u_max);
    ub_var.head(N).setConstant( u_max);

    VectorXd lbA = VectorXd::Constant(NCONS, -1e20);
    VectorXd ubA = VectorXd::Zero(NCONS);
    ubA.head(M) = gammaP;

    /* --- 7. Risolvi QP (row-major per qpOASES) ------------------------ */
    Matrix<double, Dynamic, Dynamic, RowMajor> H_rm = H_mat;
    Matrix<double, Dynamic, Dynamic, RowMajor> A_rm = A_full;

    int nWSR = 10000;
    qpOASES::returnValue status;

    if(first_solve){
        status = solver_ptr->init(
            H_rm.data(), f_vec.data(), A_rm.data(),
            lb_var.data(), ub_var.data(),
            lbA.data(), ubA.data(), nWSR);
        first_solve = false;
    } else {
        status = solver_ptr->hotstart(
            H_rm.data(), f_vec.data(), A_rm.data(),
            lb_var.data(), ub_var.data(),
            lbA.data(), ubA.data(), nWSR);
    }

        if (status != qpOASES::SUCCESSFUL_RETURN) {
            RCLCPP_WARN(this->get_logger(), "QP solver fallito, velocità azzerata");
            u_last.setZero();
            return;
        }
    /* --- 8. Estrai soluzione e scrivi sulla porta --------------------- */
        VectorXd sol(NVARS);
        solver_ptr->getPrimalSolution(sol.data());
        VectorXd u = sol.head(N);
        
        u_last=u;

        printf("[CBF] q1: %.3f %.3f %.3f %.3f %.3f %.3f | u: %.3f %.3f %.3f \n",
        q[0], q[1], q[2], 
        q[3], q[4], q[5],
        u_last(0), u_last(1), u_last(2));

        printf("[CBF] q2: %.3f %.3f %.3f %.3f %.3f %.3f | u: %.3f %.3f %.3f \n",
        q[6], q[7], q[8], 
        q[9], q[10], q[11],
        u_last(7), u_last(8), u_last(9));
    
    }

    // ── PUBLISH ──────────────────────────────────────────────────────
    void publish_offboard_control_mode_drone1(int n)
    {
        OffboardControlMode msg{};
        if (n == 1){
        msg.position     = true;
        msg.velocity     = false;
        } else {
            msg.position     = false;
            msg.velocity     = true;
        }   // ← controllo in velocità
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_drone1->publish(msg);
    }

        void publish_offboard_control_mode_drone2(int n)
    {
        OffboardControlMode msg{};
        if (n == 1){
        msg.position     = true;
        msg.velocity     = false;
        } else {
            msg.position     = false;
            msg.velocity     = true;
        }   // ← controllo in velocità
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_drone2->publish(msg);
    }


    void publish_trajectory_setpoint_drone1(int n)
    {
        Eigen::Vector3d vel_enu(u_last(0), u_last(1), u_last(2));
        Eigen::Vector3d vel_ned = ned_to_enu_local_frame(vel_enu);

        Eigen::Vector3d pos_enu(6.0, 7.0, 2.0);
        Eigen::Vector3d pos_ned = ned_to_enu_local_frame(pos_enu);

        TrajectorySetpoint msg{};
        if (n == 1){
        //msg.position     = {(float)pos_ned(0), (float)pos_ned(1), (float)pos_ned(2)};
        msg.position     = {0.0, 0.0, -2.0};
        msg.velocity     = {NAN, NAN, NAN};
        } else {
            msg.position     = {NAN, NAN, NAN};
            msg.velocity     = {(float)vel_ned(0), (float)vel_ned(1), (float)vel_ned(2)};
           // RCLCPP_INFO(this->get_logger(),
           // "Publishing velocity: vx=%.3f vy=%.3f vz=%.3f",
           // msg.velocity[0],
           // msg.velocity[1],
           // msg.velocity[2]);
        }     
        msg.acceleration = {NAN, NAN, NAN};
        msg.jerk         = {NAN, NAN, NAN};
        msg.yawspeed     = NAN;
        msg.yaw          = NAN;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;


        trajectory_setpoint_publisher_drone1->publish(msg);
    }

        void publish_trajectory_setpoint_drone2(int n)
    {
        Eigen::Vector3d vel_enu(u_last(6), u_last(7), u_last(8));
        Eigen::Vector3d vel_ned = ned_to_enu_local_frame(vel_enu);

        Eigen::Vector3d pos_enu(7.0, -8.0, 2.0);
        Eigen::Vector3d pos_ned = ned_to_enu_local_frame(pos_enu);

        TrajectorySetpoint msg{};
        if (n == 1){
        msg.position     = msg.position     = {0.0, 0.0, -2.0};
        msg.velocity     = {NAN, NAN, NAN};
        } else {
            msg.position     = {NAN, NAN, NAN};
            msg.velocity     = {(float)vel_ned(0), (float)vel_ned(1), (float)vel_ned(2)};
           // RCLCPP_INFO(this->get_logger(),
           // "Publishing velocity: vx=%.3f vy=%.3f vz=%.3f",
           // msg.velocity[0],
           // msg.velocity[1],
           // msg.velocity[2]);
        }     
        msg.acceleration = {NAN, NAN, NAN};
        msg.jerk         = {NAN, NAN, NAN};
        msg.yawspeed     = NAN;
        msg.yaw          = NAN;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;


        trajectory_setpoint_publisher_drone2->publish(msg);
    }

    void publish_vehicle_command_drone1(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1          = param1;
        msg.param2          = param2;
        msg.command         = command;
        msg.target_system   = 2;
        msg.target_component = 1;
        msg.source_system   = 1;
        msg.source_component = 1;
        msg.from_external   = true;
        msg.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_drone1->publish(msg);
    }

    void publish_vehicle_command_drone2(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1          = param1;
        msg.param2          = param2;
        msg.command         = command;
        msg.target_system   = 3;
        msg.target_component = 1;
        msg.source_system   = 1;
        msg.source_component = 1;
        msg.from_external   = true;
        msg.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_drone2->publish(msg);
    }
};

const int OffboardControl::N;
const int OffboardControl::M;
const int OffboardControl::NVARS;
const int OffboardControl::NCONS;

void OffboardControl::disarm()
{
    //publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}