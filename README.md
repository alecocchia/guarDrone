# GuarDRONE

This repository contains tools and configurations for PX4 SITL (Software In The Loop) simulation and hardware-specific deployment.
It has inside:
- my_ros2_ws/, with src/ and SimulationScripts/
- docker/, with docker image and docker run script

## Architecture Overview
1) Clone this repo in your host PC
2) Clone the firmware from official github repo, release/1.16 
git clone -b release/1.16 --recursive https://github.com/PX4/PX4-Autopilot.git
3) Build image px4_humble_harmonic_dockerfile.txt
4) Start and execute container with run_px4_cnt.sh

## Setup for gz simulation
For correct time syncronization (use_sim_time) over a drone PX4 modify firmware in folder PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes, select the airframe of the correct drone, set the parameter UXRCE_DDS_SYNCT to 0 (that is FALSE)
