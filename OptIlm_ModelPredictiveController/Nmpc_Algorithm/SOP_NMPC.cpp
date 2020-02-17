#include "SOP_NMPC.h"
#include "MPC_weighting_parameter.h"






///////////////////////////////////////////////////////////////////////////////////////////////////////////
/// NMPC controller. For differenct maneuver, different weighting factor, speed should be configured for MPC
/// The positions are updated using Extended Kalman Filter (EKF)
/// Coordinate direction is x ↑ , y ->
/// wheels to left is plus, right is minus
///
///

bool headingInvertFlag = false;



int CalculateMPC(int input_driving_model, float direction, NMPC_STRUCT * Nmpc)
{
    int ipopt_status = IPOPT_DEFAULT_STATUS;
    // parameter settings

    Nmpc->driving_model_flag = input_driving_model; //CAR_STOP

    Nmpc->car_travel_direction = direction;



    if(Nmpc->driving_model_flag == LANE_KEEPING)
    {
        Nmpc->additional_x.ini[0] = 0; // X Messwerte
        Nmpc->additional_x.ini[1] = 0; // Y Messwerte
        Nmpc->additional_x.ini[2] = 0; // Theta Messwerte
    }
    else
    {
        //use estimate position

        Nmpc->additional_x.ini[0] =  Nmpc->car_position.X_Position; // X Messwerte
        Nmpc->additional_x.ini[1] =  Nmpc->car_position.Y_Position; // Y Messwerte
        Nmpc->additional_x.ini[2] =  Nmpc->car_position.HeadingAngle; // Theta Messwerte

    }
    // debug
    //debug
    //    std::cout << "Current X: " << Nmpc->additional_x.ini[0]
    //              << "Current Y: " << Nmpc->additional_x.ini[1]
    //              << "Current Theta: " << Nmpc->additional_x.ini[2]* RADIAN_TO_DEGREES << std::endl;
    //    for (int idn = 0;idn<12;idn++)
    //        std::cout << "Trajectory X: " << Nmpc->trajectory_setpoint.X[idn]
    //                  << "Trajectory Y: " << Nmpc->trajectory_setpoint.Y[idn]
    //                  << "Trajectory Theta: " << Nmpc->trajectory_setpoint.Theta[idn]* RADIAN_TO_DEGREES << std::endl;
    //    for(int idn = 0;idn<12;idn++)
    //    {
    //        std::cout << "States : " << idn
    //                  << "-------Optimal x: " << Nmpc->optim_variables[idn * Nmpc->N_xu + 0]
    //                  << "-------Optimal y: " << Nmpc->optim_variables[idn * Nmpc->N_xu + 1]
    //                  << "-------Optimal theta: " << Nmpc->optim_variables[idn * Nmpc->N_xu + 2]  * RADIAN_TO_DEGREES
    //                  << std::endl;

    //        std::cout << "Control inputs : " << idn
    //                  << "-------Optimal speed: " << Nmpc->optim_variables[idn * Nmpc->N_xu + 3]
    //                  << "-------Optimal steering: " << Nmpc->optim_variables[idn * Nmpc->N_xu + 4] * RADIAN_TO_DEGREES
    //                  << std::endl;
    //    }

    SetNMPCWeighting(Nmpc);
    initialize_bounds(Nmpc);
//        std::cout << "Weighting x: " << Nmpc->weight.x << "Weighting y: " << Nmpc->weight.y << std::endl;
    ipopt_status = RunIpopt(Nmpc);

    //    std::cout << "Ipopt: " << ipopt_status << std::endl;

    if(Nmpc->optim_variables[4] == 0)
        Nmpc->car_control.steering = 0;
    else
        Nmpc->car_control.steering = Nmpc->optim_variables[4];
    //    else if(Nmpc->optim_variables[4] > 0)
    //        Nmpc->car_control.steering = (Nmpc->optim_variables[4] * RADIAN_TO_DEGREES);
    //    else if(Nmpc->optim_variables[4] < 0)
    //        Nmpc->car_control.steering = (Nmpc->optim_variables[4] * RADIAN_TO_DEGREES);


    Nmpc->car_control.steering = -Nmpc->car_control.steering;
    Nmpc->car_control.speed    = Nmpc->optim_variables[3];

    Nmpc->last_car_control.steering = Nmpc->optim_variables[4];
    Nmpc->last_car_control.speed    = Nmpc->optim_variables[3];



    return ipopt_status;
}


/**
 *  J(x,u) = sum_{k=0}^{k=N} {(Q_x*(x(k)-x_{ref}(k))^2 + (Q_y*(y(k)-y_{ref}(k))^2 + (Q_delta*delta^2)} + (x(N)-x_{ref}(N))^2 + (y(N)-y_{ref}(N))^2
 *           + sum_{k=0}^{k=N-1} {(Q_speed_ChangeRate*(v(k+1)-v(k))^2 + (Q_steering_ChangeRate*(delta(k+1)-delta(k))^2}
 *
 **/
void ObjectiveFunction(int n, const double* x, double& obj_value, NMPC_STRUCT * Nmpc)
{
    int index;
    obj_value = 0;

    //Lane keeping: y, steering, delta_steering
    if (Nmpc->driving_model_flag==LANE_KEEPING)
    {
        for(index = 0; index < Nmpc->N_dt; index++)
        {

            obj_value += Nmpc->weight.y * pow((x[index * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[index]), 2)
                    +Nmpc->weight.steering * pow((x[index * Nmpc->N_xu + 4]), 2);

        }


        //minimize speed and steering change rate
        for(index = 0; index < (Nmpc->N_dt - 1); index++)
        {

            obj_value += Nmpc->weight.steering_ChangeRate * pow((x[(index + 1) * Nmpc->N_xu + 4] - x[index * Nmpc->N_xu + 4]), 2);

        }
    }
    //x,y,theta,steering,final_x,final_y,delta_steering
    else
    {
        for(index = 0; index < Nmpc->N_dt; index++)
        {

            obj_value += Nmpc->weight.x * pow((x[index * Nmpc->N_xu + 0] - Nmpc->trajectory_setpoint.X[index]), 2) + Nmpc->weight.y * pow((x[index * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[index]), 2)
                    //                    + Nmpc->weight.theta * pow((x[index * Nmpc->N_xu + 2] - Nmpc->trajectory_setpoint.Theta[index]), 2)
                    /*+ Nmpc->weight.speed * pow((x[index * Nmpc->N_xu + 3] - Nmpc->vcc), 2)*/ + Nmpc->weight.steering * pow((x[index * Nmpc->N_xu + 4]), 2);

        }

        //        obj_value += Nmpc->weight.final_x * pow((x[Nmpc->N_dt * Nmpc->N_xu + 0] - Nmpc->trajectory_setpoint.X[Nmpc->N_dt]), 2) + Nmpc->weight.final_y * pow((x[Nmpc->N_dt * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[Nmpc->N_dt]), 2);

        //minimize speed and steering change rate
        for(index = 0; index < (Nmpc->N_dt - 1); index++)
        {

            obj_value += /*Nmpc->weight.speed_ChangeRate * pow((x[(index + 1) * Nmpc->N_xu + 3] - x[index * Nmpc->N_xu + 3]),2) + */Nmpc->weight.steering_ChangeRate * pow((x[(index + 1) * Nmpc->N_xu + 4] - x[index * Nmpc->N_xu + 4]), 2);

        }
    }





}

void SetNMPCWeighting(NMPC_STRUCT * Nmpc)
{
    double data[2] = {0,0};

    switch (Nmpc->driving_model_flag)
    {

    case LANE_KEEPING:
        headingInvertFlag = false;
        Nmpc->weight.x         = 0;     // Weighting matrix for X-coordinate
        Nmpc->weight.final_x   = 0;     // Weighting matrix for X-coordinate at final point

        // Weighting matrix for Y-coordinate
//        if(Nmpc->target_speed > lane_keeping_halfSpeed)
//            Nmpc->weight.y  = weightFact_Lanekeeping_HY;
//        else if(Nmpc->target_speed <=  lane_keeping_halfSpeed && Nmpc->target_speed > 0.5)
//            Nmpc->weight.y  = weightFact_Lanekeeping_LY;
//        else if(Nmpc->target_speed <=  0.5 && Nmpc->target_speed >= 0.4)
//            Nmpc->weight.y  = weightFact_Lanekeeping_DY;
//        else
//            Nmpc->weight.y  = 9;

//        if(Nmpc->vehicle_speed > lane_keeping_halfSpeed)
//            Nmpc->weight.y  = 6;
//        else if(Nmpc->vehicle_speed <=  lane_keeping_halfSpeed && Nmpc->vehicle_speed > 0.4)
//            Nmpc->weight.y  = 8;
//        else
//            Nmpc->weight.y  = 9;
//                Nmpc->weight.y = Nmpc->target_speed * weightFact_Lanekeeping_a + weightFact_Lanekeeping_b;
//                if(Nmpc->weight.y > weightFact_Lanekeeping_LY)
//                    Nmpc->weight.y  = weightFact_Lanekeeping_LY;
//                else if(Nmpc->weight.y <  weightFact_Lanekeeping_HY)
//                    Nmpc->weight.y  = weightFact_Lanekeeping_HY;

//        Nmpc->weight.y = Nmpc->vehicle_speed * weightFact_Lanekeeping_a + weightFact_Lanekeeping_b;
//        if(Nmpc->weight.y > weightFact_Lanekeeping_LY)
//            Nmpc->weight.y  = weightFact_Lanekeeping_LY;
//        else if(Nmpc->weight.y <  weightFact_Lanekeeping_HY)
//            Nmpc->weight.y  = weightFact_Lanekeeping_HY;

        data[0] = Nmpc->vehicle_speed;
        data[1] = Nmpc->trajectory_setpoint.Y[5];
        Nmpc->weight.y = WeightsNetworkFunction(&data[0]);


       // if(Nmpc->trajectory_setpoint.Y[5] < 10)
//            if (Nmpc->log_file) fprintf(Nmpc->log_file,"%.3f %.3f %.3f %.3f\n",Nmpc->vehicle_speed, Nmpc->trajectory_setpoint.Y[5], Nmpc->last_car_control.steering, Nmpc->weight.y);



        //        /* weighting factor changes linearly with the target speed */

        //        std::cout << "------weightFact_Lanekeeping_a: " << weightFact_Lanekeeping_a << std::endl;
        //        std::cout << "------weightFact_Lanekeeping_b: " << weightFact_Lanekeeping_b << std::endl;
        //        std::cout << "------Speed: " << Nmpc->target_speed << std::endl;
        //        std::cout << "------Weights Y: " << Nmpc->weight.y << std::endl;


        Nmpc->weight.final_y   = 1;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta     = 2;     // Weighting factor for heading
        Nmpc->weight.speed  = 5;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1.8;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1.5;//1.5;
        Nmpc->weight.speed_ChangeRate     = 0;
        Nmpc->vcc       = Nmpc->target_speed;
        break;

    case AVOIDANCE:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_Avoidance_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_Avoidance_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }
        Nmpc->weight.final_x   = 2;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 1;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 0;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1.2;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1.2;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed;   //no_lane_follow_speed * direction;

        break;

    case MERGE_LEFT:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_MergeLeft_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_MergeLeft_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 0;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 0;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 0;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed;   //no_lane_follow_speed * direction;


        break;

    case TURN_LEFT:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_TurnLeft_X;
            Nmpc->weight.y         = weightFact_TurnLeft_Y;

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 0;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 0;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 2;     // Weighting factor for heading
        Nmpc->weight.speed  = 5;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1.5;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1.5;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;


        break;

    case TURN_RIGHT:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_TurnRight_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_TurnRight_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 2;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 1;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 2;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;

        break;

    case STRAIGHT:
        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = /*15*/weightFact_Straight_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = /*8*/weightFact_Straight_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 0;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 5;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 8;     // Weighting factor for heading
        Nmpc->weight.speed  = 5;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 2.0;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 3.0;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;

        break;

    case PULL_OUT_LEFT:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_PullOutLeft_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_PullOutLeft_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 2;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 5;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 2;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1.5;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 2;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;


        break;

    case PULL_OUT_RIGHT:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_PullOutRight_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_PullOutRight_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_x   = 5;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.final_y   = 5;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 2;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;

        break;

    case PARKING:

        if(headingInvertFlag==false)
        {
            Nmpc->weight.x         = weightFact_Parking_X;    // Weighting matrix for X-coordinate
            Nmpc->weight.y         = weightFact_Parking_Y;     // Weighting matrix for Y-coordinate

            ChangeWeitingFromVehicleDirection(Nmpc);
            headingInvertFlag = true;
        }

        Nmpc->weight.final_y   = 0;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 0;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for velocity
        Nmpc->weight.steering  = 1;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 0;
        Nmpc->weight.speed_ChangeRate     = 3.0;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;

        break;

    default:
        Nmpc->weight.x         = 20;    // Weighting matrix for X-coordinate
        Nmpc->weight.final_x   = 2;     // Weighting matrix for X-coordinate at final point
        Nmpc->weight.y         = 5;     // Weighting matrix for Y-coordinate
        Nmpc->weight.final_y   = 5;     // Weighting matrix for Y-coordinate at final point
        Nmpc->weight.theta       = 0;     // Weighting factor for heading
        Nmpc->weight.speed  = 1;     // Weighting matrix for
        Nmpc->weight.steering  = 1;     // Weighting matrix for steering angle
        Nmpc->weight.steering_ChangeRate     = 1;
        Nmpc->weight.speed_ChangeRate     = 1;
        Nmpc->weight.Qrd       = 1.5;
        Nmpc->vcc       = Nmpc->target_speed * Nmpc->car_travel_direction;   //no_lane_follow_speed * direction;
        headingInvertFlag = false;
        break;
    }
}


void initialize_bounds(NMPC_STRUCT * Nmpc)
{
    Nmpc->additional_x.upper[0] = 1e19;    // x-coordinate
    Nmpc->additional_x.upper[1] = 1e19;//0.05;     // y-coordinate
    Nmpc->additional_x.upper[2] = 2 * PI;    // Theta

    Nmpc->additional_x.upper[3] = Nmpc->vcc;    //Nmpc->vcc// velocity //upper bound of speed
    Nmpc->additional_x.upper[4] = MAX_NEGATIVE_STEERING_ANGLE*DEGREES_TO_RADIAN;//0.45;//0.585//0.436//0.38;   // steering angle [rad]


    Nmpc->additional_x.lower[0] = -1e19;//0;
    Nmpc->additional_x.lower[1] = -1e19;//-0.05;
    Nmpc->additional_x.lower[2] = -2 * PI;

    Nmpc->additional_x.lower[3] = Nmpc->vcc;    //Nmpc->vcc// velocity //lower bound of speed
    Nmpc->additional_x.lower[4] = -MAX_POSITIVE_STEERING_ANGLE*DEGREES_TO_RADIAN;//-0.585//-0.49//-0.33


}



void NmpcInitial(NMPC_STRUCT * Nmpc, bool initial_flg)
{
    int index = 0;

    if(initial_flg == false)
    {
        Nmpc->number_of_trajectory_setpoint = 12;
        SetNMPCParameter(Nmpc, initial_flg);
        SetNmpcStructBuffer(Nmpc);
    }
    else
    {
        SetNMPCParameter(Nmpc, initial_flg);
        NmpcStructBufferFree(Nmpc);
        SetNmpcStructBuffer(Nmpc);
    }


    initialize_bounds(Nmpc);
    collocation_matrix(Nmpc);
    //collocation_matrix_2(Nmpc);


    for(index = 0; index < ((Nmpc->N_dt + 1) * (Nmpc->N_x + 1)); index++)
    {
        Nmpc->lag_mul_constraints[index] = 1;
    }

    for(index = 0; index < ((Nmpc->N_dt * Nmpc->N_xu) + Nmpc->N_x); index++)
    {
        Nmpc->optim_variables[index] = 0;
        Nmpc->lag_mul_lower_bounds[index] = 1;
        Nmpc->lag_mul_upper_bounds[index] = 1;
    }

    Nmpc->additional_x.ini[0] = 0;
    Nmpc->additional_x.ini[1] = 0;
    Nmpc->additional_x.ini[2] = 0;

    for(index = 0; index < Nmpc->N_dt; index++)
    {
        Nmpc->optim_variables[((index * Nmpc->N_xu) + 3)] = 0.25; // Velocity
    }
    for(index = 0; index < (Nmpc->N_dt + 1); index++)
    {
        Nmpc->optim_variables[((index * Nmpc->N_xu) + 0)] = Nmpc->additional_x.ini[0];
        Nmpc->optim_variables[((index * Nmpc->N_xu) + 1)] = Nmpc->additional_x.ini[1];
    }


//    Nmpc->log_file = fopen("/home/aadc/Desktop/Test.txt","w");
}

void NmpcFree(NMPC_STRUCT * Nmpc)
{
    CloseIpopt();

//    fclose(Nmpc->log_file);
}

void SetNMPCParameter(NMPC_STRUCT * Nmpc, bool initial_flg)
{
    /* Optimization parameters */
    Nmpc->N_dt = Nmpc->number_of_trajectory_setpoint; 		   // Number of time intervals  //12
    Nmpc->delta_time = 0.100;  // Length of the time intervals in [s]


    /* Model parameters */
    Nmpc->N_x     = 3;    // Number of state variables
    Nmpc->N_u     = 2;    // Number of control variables
    Nmpc->N_xu = (Nmpc->N_x + Nmpc->N_u);    // Number of optimization variables per time intervall


    /* Vehicle parameters */
    Nmpc->front_axis_length = lf;     // Distance from cog to the rear axis [m]
    Nmpc->rear_axis_length  = lr;     // Distance from cog to the front axis [m]
    Nmpc->total_axis_length = l;     // Total distance between axis [m]


    /* NMPC Weighting Seting*/
    if(initial_flg == false)
        Nmpc->driving_model_flag = CAR_STOP;
    SetNMPCWeighting(Nmpc);
}

void collocation_matrix(NMPC_STRUCT * Nmpc)
{
    Nmpc->collocation_matrix.resize(3,4);

    Nmpc->collocation_matrix(0,0) = 3.224744871392;
    Nmpc->collocation_matrix(0,1) = 1.16784008469;
    Nmpc->collocation_matrix(0,2) = -0.253197264742;
    Nmpc->collocation_matrix(0,3) = -4.13938769134;

    Nmpc->collocation_matrix(1,0) = -3.56784008469;
    Nmpc->collocation_matrix(1,1) = 0.775255128608;
    Nmpc->collocation_matrix(1,2) = 1.053197264742;
    Nmpc->collocation_matrix(1,3) = 1.73938769134;

    Nmpc->collocation_matrix(2,0) = 5.531972647422;
    Nmpc->collocation_matrix(2,1) = -7.531972647422;
    Nmpc->collocation_matrix(2,2) = 5.0;
    Nmpc->collocation_matrix(2,3) = -3.0;
}

void collocation_matrix_2(NMPC_STRUCT * Nmpc)
{
    Nmpc->collocation_matrix.resize(3,4);

    Nmpc->collocation_matrix(0,0) = 4.436491673104;
    Nmpc->collocation_matrix(0,1) = 1.032795558989;
    Nmpc->collocation_matrix(0,2) = -0.145497224368;
    Nmpc->collocation_matrix(0,3) = -5.323790007724;

    Nmpc->collocation_matrix(1,0) = -5.081988897472;
    Nmpc->collocation_matrix(1,1) = 1.774596669241;
    Nmpc->collocation_matrix(1,2) = 0.645497224368;
    Nmpc->collocation_matrix(1,3) = 2.661895003862;

    Nmpc->collocation_matrix(2,0) = 9.018480570575 ;
    Nmpc->collocation_matrix(2,1) = -8.131182235955;
    Nmpc->collocation_matrix(2,2) = 4.436491673104;
    Nmpc->collocation_matrix(2,3) = -5.323790007724;

}

void SetNmpcStructBuffer(NMPC_STRUCT * Nmpc)
{
    int number_of_time_intevals = 20;

    Nmpc->trajectory_setpoint.X   = (double *) calloc(number_of_time_intevals, sizeof(double));
    Nmpc->trajectory_setpoint.Y   = (double *) calloc(number_of_time_intevals, sizeof(double));
    Nmpc->trajectory_setpoint.Theta = (double *) calloc(number_of_time_intevals, sizeof(double));

    Nmpc->additional_x.ini   = (double *) calloc(Nmpc->N_x, sizeof(double));
    Nmpc->additional_x.lower = (double *) calloc(Nmpc->N_xu, sizeof(double));
    Nmpc->additional_x.upper = (double *) calloc(Nmpc->N_xu, sizeof(double));

    Nmpc->OUTPUT = (double *) calloc(((Nmpc->N_dt + 1) * Nmpc->N_x), sizeof(double));

    Nmpc->optim_variables      = (double *) calloc(((Nmpc->N_dt * Nmpc->N_xu) + Nmpc->N_x), sizeof(double));
    Nmpc->lag_mul_lower_bounds = (double *) calloc(((Nmpc->N_dt * Nmpc->N_xu) + Nmpc->N_x), sizeof(double));
    Nmpc->lag_mul_upper_bounds = (double *) calloc(((Nmpc->N_dt * Nmpc->N_xu) + Nmpc->N_x), sizeof(double));
    Nmpc->lag_mul_constraints  = (double *) calloc((((Nmpc->N_dt + 1) * Nmpc->N_x) + (Nmpc->N_dt + 1)), sizeof(double));

    Nmpc->dxdu.resize((Nmpc->N_dt * Nmpc->N_x), Nmpc->N_xu);
}

void NmpcStructBufferFree(NMPC_STRUCT * Nmpc)
{
   if(Nmpc->lag_mul_constraints != NULL) free(Nmpc->lag_mul_constraints);
   if(Nmpc->lag_mul_upper_bounds != NULL) free(Nmpc->lag_mul_upper_bounds);
   if(Nmpc->lag_mul_lower_bounds != NULL) free(Nmpc->lag_mul_lower_bounds);
   if(Nmpc->optim_variables != NULL) free(Nmpc->optim_variables);

   if(Nmpc->OUTPUT != NULL) free(Nmpc->OUTPUT);

   if(Nmpc->additional_x.upper != NULL) free(Nmpc->additional_x.upper);
   if(Nmpc->additional_x.lower != NULL) free(Nmpc->additional_x.lower);
   if(Nmpc->additional_x.ini != NULL) free(Nmpc->additional_x.ini);

   if(Nmpc->trajectory_setpoint.Theta != NULL) free(Nmpc->trajectory_setpoint.Theta);
   if(Nmpc->trajectory_setpoint.Y != NULL) free(Nmpc->trajectory_setpoint.Y);
   if(Nmpc->trajectory_setpoint.X != NULL) free(Nmpc->trajectory_setpoint.X);
}



int VehicleDirectionDecision(float car_heading)
{
    int car_direction = DEGREE_0;

    if (car_heading >= -PI/4 && car_heading <= PI/4 )
        car_direction = DEGREE_0;
    else if (car_heading >= PI/4 && car_heading <= 3*PI/4 )
        car_direction = DEGREE_90;
    else if (car_heading >= -3*PI/4  && car_heading <= -PI/4 )
        car_direction = DEGREE_MINUS_90;
    else
        car_direction = DEGREE_180;

    return car_direction;
}

void ChangeWeitingFromVehicleDirection(NMPC_STRUCT * Nmpc)
{
    Nmpc->car_heading = VehicleDirectionDecision(Nmpc->car_position.HeadingAngle);
    double temp = 0;

    if (Nmpc->car_heading == DEGREE_90 || Nmpc->car_heading == DEGREE_MINUS_90)
    {
        temp = Nmpc->weight.x;
        Nmpc->weight.x = Nmpc->weight.y;
        Nmpc->weight.y = temp;
    }

}




/////////////////////////////////////////////////
///
///
///
//void newton_solver(const double* x, NMPC_STRUCT * Nmpc)
//{

//    int i,k;
//    int index = 0;
//    double u1;
//    double u2;
//    VectorXd xlocal(Nmpc->N_x);
//    VectorXd xcoll(Nmpc->N_x * 3);

//    for(index = 0; index < Nmpc->N_x; index++)
//    {
//        Nmpc->OUTPUT[index] = Nmpc->additional_x.ini[index];
//    }

//    // k - interval counter
//    // i - state counter
//    for(k=0;k<Nmpc->N_dt;k++)
//    {
//        //std::cout << "Interval k " << k << std::endl;
//        // Set initial guess for all collocation points
//        int idx = 0;
//        for(i=0;i<3;i++)
//        {
//            xcoll(idx)   = x[k*Nmpc->N_xu+0];
//            xcoll(idx+1) = x[k*Nmpc->N_xu+1];
//            xcoll(idx+2) = x[k*Nmpc->N_xu+2];
//            idx = idx + 3;
//        }

//        // Set parameterized initial state variables
//        xlocal(0)  = x[k*Nmpc->N_xu+0];
//        xlocal(1)  = x[k*Nmpc->N_xu+1];
//        xlocal(2)  = x[k*Nmpc->N_xu+2];

//        // Set parameterized control variables
//        u1 = x[k*Nmpc->N_xu+3];
//        u2 = x[k*Nmpc->N_xu+4];
//        //std::cout << xlocal(0) << " " << xlocal(1) << " "<< xlocal(2) << " " << u1 << " " << u2<< std::endl;
//        //std::cout << "----------------"<< std::endl;
//        // Evaluate nonlinear equation system
//        VectorXd G = lhs_nonlin_eq(xcoll, xlocal, u1, u2, Nmpc);

//        // Evaluate Jacobian matrix w.r.t. all collocation points
//        MatrixXd dGdxcoll = dGdX(xcoll, u1, u2, Nmpc);
//        //std::cout << dGdxcoll << endl;
//        while(G.norm() > 1e-5)
//        {
//            xcoll = xcoll - dGdxcoll.lu().solve(G);
//            dGdxcoll = dGdX(xcoll, u1, u2, Nmpc);
//            G = lhs_nonlin_eq(xcoll, xlocal, u1, u2, Nmpc);
//        }

//        for(i = 0; i < Nmpc->N_x; i++)
//            Nmpc->OUTPUT[(k + 1) * Nmpc->N_x + i] = xcoll(2 * Nmpc->N_x + i);

//        Nmpc->dxdu.block(k*Nmpc->N_x,0,Nmpc->N_x,Nmpc->N_xu) = compute_sensitivities(xcoll, u1, u2, Nmpc);
//        //std::cout << dxdu.block(k*Nmpc->N_x,0,Nmpc->N_x,Nmpc->N_xu) << endl;
//    }

//}

//MatrixXd compute_sensitivities(VectorXd xx, const double u1, const double u2, NMPC_STRUCT * Nmpc)
//{

//    MatrixXd out = MatrixXd::Zero(Nmpc->N_x,Nmpc->N_xu);
//    MatrixXd dGdxcoll = dGdX(xx, u1, u2, Nmpc);
//    MatrixXd rhs = MatrixXd::Zero(3*Nmpc->N_x,Nmpc->N_xu);

//    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);
//    double one_l = (1 / Nmpc->total_axis_length);
//    double u1_l  = (u1 / Nmpc->total_axis_length);

//    rhs( 0 , 0 )= Nmpc->collocation_matrix(0,3) ;
//    rhs( 3 , 0 )= Nmpc->collocation_matrix(1,3) ;
//    rhs( 6 , 0 )= Nmpc->collocation_matrix(2,3) ;
//    rhs( 1 , 1 )= Nmpc->collocation_matrix(0,3) ;
//    rhs( 4 , 1 )= Nmpc->collocation_matrix(1,3) ;
//    rhs( 7 , 1 )= Nmpc->collocation_matrix(2,3) ;
//    rhs( 2 , 2 )= Nmpc->collocation_matrix(0,3) ;
//    rhs( 5 , 2 )= Nmpc->collocation_matrix(1,3) ;
//    rhs( 8 , 2 )= Nmpc->collocation_matrix(2,3) ;
//    rhs( 0 , 3 )= (Nmpc->delta_time * (-cos((xx(2) + (lf_l * u2))))) ;
//    rhs( 1 , 3 )= (Nmpc->delta_time * (-sin((xx(2) + (lf_l * u2))))) ;
//    rhs( 2 , 3 )= (Nmpc->delta_time * (-(one_l * tan(u2)))) ;
//    rhs( 3 , 3 )= (Nmpc->delta_time *(-cos((xx(5) + (lf_l * u2))))) ;
//    rhs( 4 , 3 )= (Nmpc->delta_time *(-sin((xx(5) + (lf_l * u2))))) ;
//    rhs( 5 , 3 )= (Nmpc->delta_time *(-(one_l * tan(u2)))) ;
//    rhs( 6 , 3 )= (Nmpc->delta_time *(-cos((xx(8) + (lf_l * u2))))) ;
//    rhs( 7 , 3 )= (Nmpc->delta_time *(-sin((xx(8) + (lf_l * u2))))) ;
//    rhs( 8 , 3 )= (Nmpc->delta_time *(-(one_l * tan(u2)))) ;
//    rhs( 0 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(2) + (lf_l * u2)))))))) ;
//    rhs( 1 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(2) + (lf_l * u2))))))) ;
//    rhs( 2 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2), 2)))) ;
//    rhs( 3 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(5) + (lf_l * u2)))))))) ;
//    rhs( 4 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(5) + (lf_l * u2))))))) ;
//    rhs( 5 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2), 2)))) ;
//    rhs( 6 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(8) + (lf_l * u2)))))))) ;
//    rhs( 7 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(8) + (lf_l * u2))))))) ;
//    rhs( 8 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2),2)))) ;

//    MatrixXd dXdX0U = dGdxcoll.lu().solve(-rhs);
//    out << dXdX0U.row(6), dXdX0U.row(7), dXdX0U.row(8);


//    return out;
//}

//VectorXd lhs_nonlin_eq(VectorXd xx, VectorXd x0, const double u1, const double u2, NMPC_STRUCT * Nmpc)
//{

//    VectorXd F(Nmpc->N_x*3);

//    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);
//    double u1_l  = (u1 / Nmpc->total_axis_length);

//    /* xx - State variables at collocation points
//     * x0 - Parameterized initial state variables
//     * u1 - Control variable -> Velocity
//     * u2 - Control variable -> Steering angle
//     * */

//    F(0) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(0)) + (Nmpc->collocation_matrix(0,1) * xx(3))) + (Nmpc->collocation_matrix(0,2) * xx(6))) + (Nmpc->collocation_matrix(0,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(2) + (lf_l * u2)))));
//    F(1) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(1)) + (Nmpc->collocation_matrix(0,1) * xx(4))) + (Nmpc->collocation_matrix(0,2) * xx(7))) + (Nmpc->collocation_matrix(0,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(2) + (lf_l * u2)))));
//    F(2) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(2)) + (Nmpc->collocation_matrix(0,1) * xx(5))) + (Nmpc->collocation_matrix(0,2) * xx(8))) + (Nmpc->collocation_matrix(0,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));
//    F(3) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(0)) + (Nmpc->collocation_matrix(1,1) * xx(3))) + (Nmpc->collocation_matrix(1,2) * xx(6))) + (Nmpc->collocation_matrix(1,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(5) + (lf_l * u2)))));
//    F(4) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(1)) + (Nmpc->collocation_matrix(1,1) * xx(4))) + (Nmpc->collocation_matrix(1,2) * xx(7))) + (Nmpc->collocation_matrix(1,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(5) + (lf_l * u2)))));
//    F(5) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(2)) + (Nmpc->collocation_matrix(1,1) * xx(5))) + (Nmpc->collocation_matrix(1,2) * xx(8))) + (Nmpc->collocation_matrix(1,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));
//    F(6) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(0)) + (Nmpc->collocation_matrix(2,1) * xx(3))) + (Nmpc->collocation_matrix(2,2) * xx(6))) + (Nmpc->collocation_matrix(2,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(8) + (lf_l * u2)))));
//    F(7) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(1)) + (Nmpc->collocation_matrix(2,1) * xx(4))) + (Nmpc->collocation_matrix(2,2) * xx(7))) + (Nmpc->collocation_matrix(2,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(8) + (lf_l * u2)))));
//    F(8) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(2)) + (Nmpc->collocation_matrix(2,1) * xx(5))) + (Nmpc->collocation_matrix(2,2) * xx(8))) + (Nmpc->collocation_matrix(2,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));

//    return F;
//}

//MatrixXd dGdX(VectorXd xx, const double u1, const double u2, NMPC_STRUCT * Nmpc)
//{

//    MatrixXd mdGdX = MatrixXd::Zero(3*Nmpc->N_x,3*Nmpc->N_x);

//    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);

//    mdGdX( 0 , 0 )= Nmpc->collocation_matrix(0,0) ;
//    mdGdX( 3 , 0 )= Nmpc->collocation_matrix(1,0) ;
//    mdGdX( 6 , 0 )= Nmpc->collocation_matrix(2,0) ;
//    mdGdX( 1 , 1 )= Nmpc->collocation_matrix(0,0) ;
//    mdGdX( 4 , 1 )= Nmpc->collocation_matrix(1,0) ;
//    mdGdX( 7 , 1 )= Nmpc->collocation_matrix(2,0) ;
//    mdGdX( 0 , 2 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(2) + (lf_l * u2))))))) ;
//    mdGdX( 1 , 2 )= (Nmpc->delta_time * (-(u1 *   cos((xx(2) + (lf_l * u2)))))) ;
//    mdGdX( 2 , 2 )= Nmpc->collocation_matrix(0,0) ;
//    mdGdX( 5 , 2 )= Nmpc->collocation_matrix(1,0) ;
//    mdGdX( 8 , 2 )= Nmpc->collocation_matrix(2,0) ;
//    mdGdX( 0 , 3 )= Nmpc->collocation_matrix(0,1) ;
//    mdGdX( 3 , 3 )= Nmpc->collocation_matrix(1,1) ;
//    mdGdX( 6 , 3 )= Nmpc->collocation_matrix(2,1) ;
//    mdGdX( 1 , 4 )= Nmpc->collocation_matrix(0,1) ;
//    mdGdX( 4 , 4 )= Nmpc->collocation_matrix(1,1) ;
//    mdGdX( 7 , 4 )= Nmpc->collocation_matrix(2,1) ;
//    mdGdX( 2 , 5 )= Nmpc->collocation_matrix(0,1) ;
//    mdGdX( 3 , 5 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(5)+(lf_l  *u2))))))) ;
//    mdGdX( 4 , 5 )= (Nmpc->delta_time * (-(u1 *   cos((xx(5)+(lf_l * u2)))))) ;
//    mdGdX( 5 , 5 )= Nmpc->collocation_matrix(1,1) ;
//    mdGdX( 8 , 5 )= Nmpc->collocation_matrix(2,1) ;
//    mdGdX( 0 , 6 )= Nmpc->collocation_matrix(0,2) ;
//    mdGdX( 3 , 6 )= Nmpc->collocation_matrix(1,2) ;
//    mdGdX( 6 , 6 )= Nmpc->collocation_matrix(2,2) ;
//    mdGdX( 1 , 7 )= Nmpc->collocation_matrix(0,2) ;
//    mdGdX( 4 , 7 )= Nmpc->collocation_matrix(1,2) ;
//    mdGdX( 7 , 7 )= Nmpc->collocation_matrix(2,2) ;
//    mdGdX( 2 , 8 )= Nmpc->collocation_matrix(0,2) ;
//    mdGdX( 5 , 8 )= Nmpc->collocation_matrix(1,2) ;
//    mdGdX( 6 , 8 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(8) + (lf_l * u2))))))) ;
//    mdGdX( 7 , 8 )= (Nmpc->delta_time * (-(u1 *   cos((xx(8) + (lf_l * u2)))))) ;
//    mdGdX( 8 , 8 )= Nmpc->collocation_matrix(2,2) ;

//    return mdGdX;
//}

double WeightsNetworkFunction(const double x1[2])
{
//  double xp1[2];
//  int k;
//  double av[2];
//  static const double b[2] = { 0.099, -0.285 };

//  double d0;
//  double d1;
//  int i0;
//  static const double a[15] = { 0.13181593253348814, 0.42134026099296096,
//    -0.13147181041304948, -0.094076694952236126, -0.53256905938993637,
//    -1.1033208673535537, -0.0031186043856376508, 0.53743171712970939,
//    0.55453562011418056, 0.15360451931790139, 0.32237327639058633,
//    -0.83858832765023272, 0.43615718730344211, 0.095951393632841275,
//    -0.27439838490515422 };

//  static const double b_a[15] = { -5.7813083897340052, -4.486262629154913,
//    -3.8275310541396697, 3.1227559036706487, -2.4460311416126119,
//    1.5467191251133763, 0.82323776058131248, 0.031395445255909246,
//    -0.83563488573646716, -1.5048858803674172, 2.2616586312350138,
//    -3.011333970499551, 3.869021142941107, 4.94699581156847, 5.4254843020370664
//  };

//  static const double c_a[30] = { 5.0338955203894011, 1.0773426839267559,
//    1.2435702499545747, -1.8025240100793161, 4.0397928503641776,
//    -2.559945642357969, -5.4103076248540134, -4.6273605940690157,
//    -4.8733407783038123, -3.0725995918995856, 3.795044018947229,
//    -3.9148977367218953, 3.1381481055257106, 4.8865034843482409,
//    0.27709286703826969, -0.56294831305685167, -5.4286275230169956,
//    -5.2803173364922218, 5.0829264786166624, -3.5159785373199126,
//    -4.7854016794315148, 0.17655449048382307, 2.885360438294394,
//    -2.0325574592125779, -4.42740104457047, 3.9771579155527759,
//    3.8076791425499006, -4.4212074648686732, 1.6361402847751738,
//    -5.4117173643031125 };

//  /* MYNEURALNETWORKFUNCTION neural network simulation function. */
//  /*  */
//  /*  Generated by Neural Network Toolbox function genFunction, 06-Nov-2018 17:26:58. */
//  /*  */
//  /*  [y1] = myNeuralNetworkFunction(x1) takes these arguments: */
//  /*    x = Qx2 matrix, input #1 */
//  /*  and returns: */
//  /*    y = Qx1 matrix, output #1 */
//  /*  where Q is the number of samples. */
//  /*  ===== NEURAL NETWORK CONSTANTS ===== */
//  /*  Input 1 */
//  /*  Layer 1 */
//  /*  Layer 2 */
//  /*  Output 1 */
//  /*  ===== SIMULATION ======== */
//  /*  Dimensions */
//  /*  samples */
//  /*  Input 1 */
//  /*  ===== MODULE FUNCTIONS ======== */
//  /*  Map Minimum and Maximum Input Processing Function */
//  for (k = 0; k < 2; k++) {
//    xp1[k] = x1[k] - b[k];
//  }

//  for (k = 0; k < 2; k++) {
//    av[k] = xp1[k] * (1.15141047783535 + -0.643280396534537 * (double)k);
//  }

//  for (k = 0; k < 2; k++) {
//    xp1[k] = av[k];
//  }

//  for (k = 0; k < 2; k++) {
//    av[k] = xp1[k] + -1.0;
//  }

//  for (k = 0; k < 2; k++) {
//    xp1[k] = av[k];
//  }

//  /*  Layer 1 */
//  /*  Sigmoid Symmetric Transfer Function */
//  /*  Layer 2 */
//  /*  Output 1 */
//  /*  Map Minimum and Maximum Output Reverse-Processing Function */
//  d0 = 0.0;
//  for (k = 0; k < 15; k++) {
//    d1 = 0.0;
//    for (i0 = 0; i0 < 2; i0++) {
//      d1 += c_a[k + 15 * i0] * xp1[i0];
//    }

//    d0 += a[k] * (2.0 / (1.0 + std::exp(-2.0 * (b_a[k] + d1))) - 1.0);
//  }

//  return ((0.16293473039924483 + d0) - -1.0) / 0.285714285714286 + 3.0;

    double xp1[2];
    int k;
    double av[2];
    static const double b[2] = { 0.218, -0.113 };

    static const double b_b[2] = { 2.75482093663912, 7.96812749003984 };

    double d0;
    double d1;
    int i0;
    static const double a[10] = { -0.60767044734289954, -0.3824366236056661,
      -0.22019234906419202, 0.39697817571281635, -0.29197686907726789,
      -0.013519388508301875, 0.030149975275006798, 0.17094741385230586,
      0.36185978415346609, -0.19415681625330106 };

    static const double b_a[10] = { -4.3401646648953092, -3.3038734751949579,
      2.5006631876754928, 1.0705927480165931, 0.00424481389026447,
      -0.46017639949308758, -1.9751012610670755, -1.6691655186247154,
      -3.4782369614714974, -4.5889910071406144 };

    static const double c_a[20] = { 2.6680245126428668, 3.2785027711324637,
      -0.90715688814991657, -3.4805392128316726, 3.3955954282220002,
      -2.3569664288153973, -0.96215707146439244, -4.5403703555656421,
      -2.8701626255408326, -3.8796420619137741, 3.5978839764269765,
      -3.075490346656709, -4.2626806826353087, -2.58013440558798,
      -2.6493772333690915, 3.6219860072361265, -4.0280143158018724,
      -0.72177943902667, -3.2624806052315365, 1.6796513731590552 };

    /* WEIGHTNETWORKFUNCTION neural network simulation function. */
    /*  */
    /*  Generated by Neural Network Toolbox function genFunction, 10-Nov-2018 15:34:50. */
    /*   */
    /*  [y1] = WeightNetworkFunction(x1) takes these arguments: */
    /*    x = 2xQ matrix, input #1 */
    /*  and returns: */
    /*    y = 1xQ matrix, output #1 */
    /*  where Q is the number of samples. */
    /*  ===== NEURAL NETWORK CONSTANTS ===== */
    /*  Input 1 */
    /*  Layer 1 */
    /*  Layer 2 */
    /*  Output 1 */
    /*  ===== SIMULATION ======== */
    /*  Dimensions */
    /*  samples */
    /*  Input 1 */
    /*  ===== MODULE FUNCTIONS ======== */
    /*  Map Minimum and Maximum Input Processing Function */
    for (k = 0; k < 2; k++) {
      xp1[k] = x1[k] - b[k];
    }

    for (k = 0; k < 2; k++) {
      av[k] = xp1[k] * b_b[k];
    }

    for (k = 0; k < 2; k++) {
      xp1[k] = av[k];
    }

    for (k = 0; k < 2; k++) {
      av[k] = xp1[k] + -1.0;
    }

    for (k = 0; k < 2; k++) {
      xp1[k] = av[k];
    }

    /*  Layer 1 */
    /*  Sigmoid Symmetric Transfer Function */
    /*  Layer 2 */
    /*  Output 1 */
    /*  Map Minimum and Maximum Output Reverse-Processing Function */
    d0 = 0.0;
    for (k = 0; k < 10; k++) {
      d1 = 0.0;
      for (i0 = 0; i0 < 2; i0++) {
        d1 += c_a[k + 10 * i0] * xp1[i0];
      }

      d0 += a[k] * (2.0 / (1.0 + std::exp(-2.0 * (b_a[k] + d1))) - 1.0);
    }

    return ((-0.40934075391087416 + d0) - -1.0) / 0.285714285714286 + 3.0;
}



