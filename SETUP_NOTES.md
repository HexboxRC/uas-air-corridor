# Setup Notes (captured from Jetson Orin)

## PX4 version
tag:    v1.18.0-alpha1-256-g8be2590c15
commit: 8be2590c1529517ccd54fbc09ae138402517baf4
branch: main

## px4_msgs version
commit: a5aec95ed69086467b1f92de30093a04d03fd1d4
branch: main

## Airframe file: 4001_gz_x500
```
#!/bin/sh
#
# @name Gazebo x500
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
# PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

param set-default MPC_THR_HOVER 0.60
param set-default NAV_DLL_ACT 2

param set-default GF_MAX_HOR_DIST 0
param set-default GF_MAX_VER_DIST 0
param set-default GF_ACTION 0
param set-default COM_DIST_MAX 0
param set-default COM_RCL_EXCEPT 4
param set-default NAV_RCL_ACT 0

param set-default NAV_RCL_ACT 0
param set-default NAV_DLL_ACT 0
param set-default COM_RCL_EXCEPT 4
param set-default COM_ARM_WO_GPS 0

uxrce_dds_client start -t udp -p 8888
```
