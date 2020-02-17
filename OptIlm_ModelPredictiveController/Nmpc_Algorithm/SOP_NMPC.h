#ifndef SOP_NMPC_H_
#define SOP_NMPC_H_

#include "IpIpoptApplication.hpp"
#include "TU_Ilmenau_SOP.h"
#include <time.h>
#include "Eigen/Core"
#include "Eigen/Dense"



using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

#define IPOPT_DEFAULT_STATUS 10

typedef struct _COORDINATE_STRUCT
{
    double *X;
    double *Y;
    double *Theta;

}COORDINATE_STRUCT;


typedef struct _ADDITIONAL_MATRIX
{
    double *ini;
    double *lower;
    double *upper;

}ADDITIONAL_MATRIX;


typedef struct _WEIGHTING_MATRIX
{
    double x;           // Weighting matrix for X-coordinate
    double final_x;     // Weighting matrix for X-coordinate at final point
    double y;           // Weighting matrix for Y-coordinate
    double final_y;     // Weighting matrix for Y-coordinate at final point
    double theta;         // Weighting factor for heading
    double speed;    // Weighting matrix for velocity
    double steering;    // Weighting matrix for steering angle
    double steering_ChangeRate;
    double speed_ChangeRate;
    double Qrd;
//    double vcc;         //no_lane_follow_speed * direction;

}WEIGHTING_MATRIX;


typedef struct _NMPC_STRUCT
{
    int driving_model_flag;
    float target_speed;
    float vehicle_speed;
    int car_heading;
    float car_travel_direction;
    CAR_POSITION_STRUCT car_position;
    CONTROL_SIGNAL_STRUCT car_control;
    CONTROL_SIGNAL_STRUCT last_car_control;


    /* Model parameters */
    short N_x;          // Number of state variables
    short N_u;        // Number of control variables
    short N_xu;    // Number of optimization variables per time intervall


    /* Optimization parameters */
    short N_dt;           // Number of time intervals
    double delta_time;                // Length of the time intervals in [s]
    WEIGHTING_MATRIX weight;
    COORDINATE_STRUCT trajectory_setpoint;
    int number_of_trajectory_setpoint;


    /* Optimizer parameters */
    double *optim_variables;          // Optimization variables
    double *lag_mul_lower_bounds;	  // Lagrange multipliers for lower bounds
    double *lag_mul_upper_bounds;	  // Lagrange multipliers for upper bounds
    double *lag_mul_constraints;      // Lagrange multipliers for constraints
    double *OUTPUT;                   // MISC!
    MatrixXd collocation_matrix;      // Collocation matrix, derivatives of the Lagrange polynomials at collocation points
    MatrixXd dxdu;



    /* Vehicle parameters */
    double rear_axis_length;     // Distance from cog to the rear axis [m]
    double front_axis_length;    // Distance from cog to the front axis [m]
    double total_axis_length;    // Total distance between axis [m]


    /* Additional */
    ADDITIONAL_MATRIX additional_x;


    /* IPOPT  */
    float ipopt_operation_time;

    /* Speed limitation*/
    double vcc;


    FILE *log_file; // debug file

}NMPC_STRUCT;









//Function Type

//Ipopt
int RunIpopt(NMPC_STRUCT * Nmpc);
int SetIpopt(void);
bool CloseIpopt(void);


//for SOP_nlp.cpp
void ObjectiveFunction(int n, const double* x, double& obj_value, NMPC_STRUCT * Nmpc);
//// Method to solve nonlinear equation system
//void newton_solver(const double *x, NMPC_STRUCT *Nmpc);
//// Method to compute first order sensitivities
//MatrixXd compute_sensitivities(VectorXd xx, const double u1, const double u2, NMPC_STRUCT *Nmpc);
//// Method to return function values inside the nonlinear solver
//VectorXd lhs_nonlin_eq(VectorXd xx, VectorXd x0, const double u1, const double u2, NMPC_STRUCT * Nmpc);
//// Method to compute sensitivities with respect to state variables at collocation points
//MatrixXd dGdX(VectorXd xx, const double u1, const double u2, NMPC_STRUCT * Nmpc);




//SOP Nmpc
void NmpcInitial(NMPC_STRUCT * Nmpc, bool initial_flg);
void NmpcFree(NMPC_STRUCT * Nmpc);
void SetNmpcStructBuffer(NMPC_STRUCT * Nmpc);
void NmpcStructBufferFree(NMPC_STRUCT * Nmpc);
void SetNMPCParameter(NMPC_STRUCT * Nmpc, bool initial_flg);
void SetNMPCWeighting(NMPC_STRUCT * Nmpc);
void initialize_bounds(NMPC_STRUCT * Nmpc);
void collocation_matrix(NMPC_STRUCT * Nmpc);
void collocation_matrix_2(NMPC_STRUCT * Nmpc);
void ChangeWeitingFromVehicleDirection(NMPC_STRUCT * Nmpc);
int CalculateMPC(int input_car_state_flag, float direction, NMPC_STRUCT * Nmpc);
double WeightsNetworkFunction(const double x1[2]);


//Addition
int VehicleDirectionDecision(float car_heading);

#endif

