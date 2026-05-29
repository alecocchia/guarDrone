/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <px4_ros_com/frame_transforms.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

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

struct trajectory_point {
    
    Eigen::Vector3d pos;
};


class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

    	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    	auto px4_qos = rclcpp::QoS(
      	rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
 
    	// ── SUBSCRIBER ───────────────────────────────────────────────────
    	// Argomenti: <TipoMessaggio>(topic, qos, callback)
        //subscriber_drone1 = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        //    "/px4_1/fmu/out/vehicle_odometry", px4_qos,
        //    std::bind(&OffboardControl::on_odometry_drone1, this, std::placeholders::_1)
        //);

        //subscriber_drone2 = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        //    "/px4_2/fmu/out/vehicle_odometry", px4_qos,
        //std::bind(&OffboardControl::on_odometry_drone2, this, std::placeholders::_1)
        //);


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

		offboard_setpoint_counter_ = 0;

		// Subscriber: aspetta il comando di start dal BT
		start_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    	"/takeoff/start", 10,
    	std::bind(&OffboardControl::on_start, this, std::placeholders::_1)
		);

		// Publisher: manda lo stato al BT
		status_publisher_ = this->create_publisher<std_msgs::msg::String>(
    	"/takeoff/status", 10
		);

    cbf_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/Cbf_controll/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        cbf_status_ = msg->data;
    });

		RCLCPP_INFO(this->get_logger(), "Nodo pronto. Aspetto /takeoff/start ...");
	}

	void arm();
	void disarm();

private:

	rclcpp::TimerBase::SharedPtr timer_;
	
  //rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscriber_drone1;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_drone1;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_drone1;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_drone1;

  //rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscriber_drone2;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_drone2;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_drone2;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_drone2;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   start_subscriber_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    status_publisher_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cbf_status_sub_;
  std::string cbf_status_ = "";

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode_drone1();
	void publish_trajectory_setpoint_drone1(Vector3d trajInit_, Vector3d trajEnd_);
	void publish_vehicle_command_drone1(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	
  void publish_offboard_control_mode_drone2();
	void publish_trajectory_setpoint_drone2(Vector3d trajInit_, Vector3d trajEnd_);
	void publish_vehicle_command_drone2(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	trajectory_point compute_trajectory(double time,Vector3d trajInit_, Vector3d trajEnd_);
	void compute_trapezoidal_velocity_point(double t,double & s,double & sdot,double & sdotdot);

	double dt_t=0.1;
	double ti=0; 

	bool running_ = false;

  Vector3d trajEnd_1{0.0, 0.0, -2.0};
  Vector3d trajInit_1{0.0,0.0,0};


  Vector3d trajEnd_2{0.0,0.0,-2};
  Vector3d trajInit_2{0.0,0.0,0};




	//void on_odometry(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
  //{
    // ── Controlla validità (PX4 usa NaN per dati non disponibili) ────
    //if (std::isnan(msg->position[0]) ||
     //   std::isnan(msg->position[1]) ||
     //   std::isnan(msg->position[2]))
    //{
    //  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //    "Posizione non disponibile (NaN)");
    //  return;
    //}

    // ────────────────────────────────────────────────────────────────
    // POSIZIONE
    // Il frame di default PX4 è NED (North-East-Down):
    //   position[0] = Nord  [m]
    //   position[1] = Est   [m]
    //   position[2] = Giù   [m]  ← negativo = in quota
    // ────────────────────────────────────────────────────────────────

    // ────────────────────────────────────────────────────────────────    // LOG — stampa i dati principali
    // ────────────────────────────────────────────────────────────────
    //RCLCPP_INFO(this->get_logger(),
    //  "pos=[%.2f, %.2f, %.2f m]",
    //  msg->position[0], msg->position[1], msg->position[2]);
  //}

  void on_start(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !running_) {   // solo se true e non già partito
        RCLCPP_INFO(this->get_logger(), "Ricevuto start! Avvio takeoff...");
        running_ = true;
        offboard_setpoint_counter_ = 0;
        ti = 0.0;

        // Ora creo il timer — stesso callback che avevi prima
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command_drone1(
                    VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->publish_vehicle_command_drone2(
                    VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);    
                this->arm();
            }



            publish_offboard_control_mode_drone1();
            publish_offboard_control_mode_drone2();
            publish_trajectory_setpoint_drone1(trajInit_1,trajEnd_1);
            publish_trajectory_setpoint_drone2(trajInit_2,trajEnd_2);

            if (offboard_setpoint_counter_ >= 11) {
                ti += 0.1;
                if (ti >= 10.0) {
                    ti = 10.0;
                    publish_status("SUCCESS");   // ← dice al BT che abbiamo finito
                if (cbf_status_ == "RUNNING") {
                    publish_status("SUCCESS");
                    timer_->cancel();
                    return;}           // ← stoppa il timer
                    return;
                }
            }

            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;

            publish_status("RUNNING");
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }
}

void publish_status(const std::string & s)
{
    std_msgs::msg::String msg;
    msg.data = s;
    status_publisher_->publish(msg);
}

};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command_drone1(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    publish_vehicle_command_drone2(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command_drone1(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    publish_vehicle_command_drone2(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode_drone1()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_drone1->publish(msg);

}

void OffboardControl::publish_offboard_control_mode_drone2()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_drone2->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint_drone1(Vector3d trajInit_, Vector3d trajEnd_)
{

	trajectory_point traj;
	traj = compute_trajectory(ti, trajInit_,trajEnd_);
	

	Eigen::Vector3d pos_ned(traj.pos(0), traj.pos(1), traj.pos(2));
  //Eigen::Vector3d pos_ned = ned_to_enu_local_frame(pos_enu);

	TrajectorySetpoint msg{};
	msg.position = {(float)pos_ned(0), (float)pos_ned(1), (float)pos_ned(2)};
	msg.yaw          = NAN;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_drone1->publish(msg);
    RCLCPP_INFO(this->get_logger(),
    "Publishing pos: px=%.3f py=%.3f pz=%.3f",
    msg.position[0],
    msg.position[1],
    msg.position[2]);
}

void OffboardControl::publish_trajectory_setpoint_drone2(Vector3d trajInit_, Vector3d trajEnd_)
{

	trajectory_point traj;
	traj = compute_trajectory(ti, trajInit_,trajEnd_);
	

	Eigen::Vector3d pos_ned(traj.pos(0), traj.pos(1), traj.pos(2));
  //Eigen::Vector3d pos_ned = ned_to_enu_local_frame(pos_enu);

	TrajectorySetpoint msg{};
	msg.position = {(float)pos_ned(0), (float)pos_ned(1), (float)pos_ned(2)};
	msg.yaw          = NAN;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_drone2->publish(msg);
    RCLCPP_INFO(this->get_logger(),
    "Publishing pos: px=%.3f py=%.3f pz=%.3f",
    msg.position[0],
    msg.position[1],
    msg.position[2]);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command_drone1(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 2;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_drone1->publish(msg);
}

void OffboardControl::publish_vehicle_command_drone2(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 3;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_drone2->publish(msg);
}

void OffboardControl::compute_trapezoidal_velocity_point(double t,double & s,double & sdot,double & sdotdot)
{
   double trajDuration_=10;
  double tc=2;
  double maxAcc_=1.0/(-(std::pow(tc,2))+trajDuration_*tc);  
  if(t <= tc)
  {
    s= 0.5*maxAcc_*std::pow(t,2);
    sdot=maxAcc_*t;
    sdotdot=maxAcc_;
  }
  else if(t <= trajDuration_-tc)
  {
    s =maxAcc_*tc*(t-tc/2);
    sdot=maxAcc_*tc;
    sdotdot=0;  
  }
  else
  {
    s= 1 - 0.5*maxAcc_*std::pow(trajDuration_-t,2);
    sdot=maxAcc_*(trajDuration_-t);
    sdotdot=-maxAcc_;  
  }

}

trajectory_point OffboardControl::compute_trajectory(double time,Vector3d trajInit_, Vector3d trajEnd_)
{
    double s=0;
    double sdot=0;
    double sdotdot=0;

   trajectory_point traj;
   
    compute_trapezoidal_velocity_point(time,s,sdot,sdotdot);        
    //traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
    //traj.vel = sdot*(trajEnd_-trajInit_);
    //traj.acc = sdotdot*(trajEnd_-trajInit_);
    std::cout<<"ascissa: "<<s<<std::endl;       
	traj.pos= trajInit_ + s*(trajEnd_-trajInit_);
 
return traj;
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