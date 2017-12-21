#ifndef CONSTANTS_H
#define CONSTANTS_H
const double Timestep = 0.02;

// At each timestep, ego can set acceleration to value between 
// -Max_Accel and Max_Accel
const double Max_Acceleration = 10; // 10m/s/s
const double Max_Jerk = 10; // 10m/s/s/s

const double Expected_Acceleration_Per_Sec = 10; // 10m/s/s
const double Expected_Jerk_Per_Sec = 10; // 10m/s/s/s

// 1mile = 1609.34m, 50mph = 80.4672kmh = 22.352m/s
const double Speed_Limit = 20;
const double Mile2Meter = 1609.34;

const double Num_Lanes = 3;
const double Lane_Width = 4;

const double Vehicle_Radius = 1.5;
const double Lane_Change_Buffer = 30;
const double Lane_Change_Search_Range = 100;
const double Follow_Ahead_Range = 50;
const double Max_S = 6945.554;

/*
N_SAMPLES = 10
SIGMA_S = [10.0, 4.0, 2.0] # s, s_dot, s_double_dot
SIGMA_D = [1.0, 1.0, 1.0]
SIGMA_T = 2.0

EXPECTED_JERK_IN_ONE_SEC = 2 # m/s/s
EXPECTED_ACC_IN_ONE_SEC = 1 # m/s

*/
#endif
