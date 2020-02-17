#ifndef EXTEND_KALMAN_FILTER_H
#define EXTEND_KALMAN_FILTER_H

#include "TU_Ilmenau_SOP.h"
#include "Eigen/Core"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;


#ifdef AUTO_A
#define FF 1.0
#define HH 1.0
#define QQ 0.85  // The covariance matrix of the process noise
#define RR 0.001  // The covariance matrix of the observation noise
#define II 1.0
#else
#define FF 1.0
#define HH 1.0
#define QQ 0.85  // 20 The covariance matrix of the process noise
#define RR 0.001  // 10 The covariance matrix of the observation noise
#define II 1.0
#endif



typedef struct _EKF_STRUCT
{
    float last_heading;
    double delta_time;                // Length of the time intervals in [s]

    float steering_angle;
    CAR_POSITION_STRUCT estimates_position;
    CAR_POSITION_STRUCT currents_position;

}EKF_STRUCT;



void InitialExtendedKalmanFilter(EKF_STRUCT *EKF);
void ResetExtendedKalmanFilter(EKF_STRUCT *EKF);
void RunExtendedKalmanFilter(EKF_STRUCT *EKF);

double dotX1(double *sVars, double *uu, double h);
double dotX2(double *sVars, double *uu, double h);
double dotX3(double *sVars, double *uu, double h);


#endif // EXTEND_KALMAN_FILTER_H
