#include "ExtendedKalmanFilter.h"
#include "rungekutta.h"



///* Data for Kalman filter*/
RungeKutta *rk4;

vector <double (*)(double*, double*, double)> funcs;

Matrix3d F;
Matrix3d H;
Matrix3d Q;
Matrix3d R;
Matrix3d I;

Matrix3d P;
Vector3d sensors;
Vector3d estimates;

double PP = 0.5; // The a posteriori error covariance matrix

/**
 * @brief InitialExtendedKalmanFilter
 * @param EKF
 * PP: Predicted covariance estimate
 * QQ: The covariance matrix of the process noise
 * RR: The covariance matrix of the observation noise
 * TT: Innovation coviance
 * KG: Kalman gain
 *
 * Predict:
 *          x(k|k-1) = f(x(k),u(k))
 *          P(k|k-1) = F(k)*P(k-1|k-1)*F(k)^T+Q(k)
 * Update:
 *          y(k) = z(k) - h(x(k|k-1))
 *          S(k) = H(k)*P(k|k-1)*H(k)^T+R(k)
 *          KG(k) = P(k|k-1)*H(k)^T*inv(S(k))
 *          x(k|k) = x(k|k-1) + K(k)*y(k)
 *          P(k|k) = (I-KG(k)*H(k))*P(k|k-1)
 */
void InitialExtendedKalmanFilter(EKF_STRUCT *EKF)
{
    // Kalman filter initialization
    P<<10, 0, 0,
       0, 10, 0,
       0, 0, 0.5;
    sensors << 0,0,0;
    estimates << 0,0,0;

    estimates(0) = EKF->currents_position.X_Position ;
    estimates(1) = EKF->currents_position.Y_Position;
    estimates(2) = EKF->currents_position.HeadingAngle;

    EKF->estimates_position.X_Position   =  estimates(0); // X Messwerte
    EKF->estimates_position.Y_Position   =  estimates(1); // Y Messwerte
    EKF->estimates_position.HeadingAngle =  estimates(2); // Psi Messwerte
    EKF->last_heading = EKF->currents_position.HeadingAngle;

    EKF->steering_angle = 0;
    EKF->delta_time = 0.100;       // Length of the time intervals in [s]
}

void ResetExtendedKalmanFilter(EKF_STRUCT *EKF)
{
    estimates(0) = EKF->currents_position.X_Position ;
    estimates(1) = EKF->currents_position.Y_Position;
    estimates(2) = EKF->currents_position.HeadingAngle;
    EKF->last_heading = EKF->currents_position.HeadingAngle;

    EKF->steering_angle = 0;
    PP = 0.5;
}

void RunExtendedKalmanFilter(EKF_STRUCT *EKF)
{
    /* KALMAN FILTER */
    const double h = EKF->delta_time/2; // Length of the time intervals in [s]0.00500
    funcs.resize(3);

    funcs[0] = dotX1;
    funcs[1] = dotX2;
    funcs[2] = dotX3;
    rk4 = new RungeKutta(funcs, 3);
    vector <double> res;


    for(double t = 0; t < EKF->delta_time; t += h)
    {
        // Control and State arrays for Runge-Kutta method
        double uu[] = {EKF->currents_position.speed, EKF->steering_angle};
        double xx[] = {estimates(0), estimates(1), estimates(2)};

        // Calculate next step
        res = rk4->calcState(xx, uu, h);
        estimates(0) += res[0];
        estimates(1) += res[1];
        estimates(2) += res[2];
    }


    if(EKF->currents_position.radius < 0.5 /*&& road_marker_ID != NO_TRAFFIC_SIGN*/)
    {
        sensors(0) = EKF->currents_position.X_Position; // Should be the data from sensors X
        sensors(1) = EKF->currents_position.Y_Position;  // Should be the data from sensors Y
        sensors(2) = EKF->currents_position.HeadingAngle; // Should be the data from sensors PSI

        estimates(0) = EKF->currents_position.X_Position; // Should be the data from sensors X
        estimates(1) = EKF->currents_position.Y_Position;  // Should be the data from sensors Y

//        estimates(2) = EKF->currents_position.HeadingAngle; // Should be the data from sensors PSI

        PP = 0.5;

//        P<<10, 0, 0,
//           0, 10, 0,
//           0, 0, 0.5;
    }
    else
    {
//        sensors(0) = estimates(0); // Should be the data from sensors X
//        sensors(1) = estimates(1);  // Should be the data from sensors Y
        sensors(0) = EKF->currents_position.X_Position; // Should be the data from sensors X
        sensors(1) = EKF->currents_position.Y_Position;  // Should be the data from sensors Y
        sensors(2) = EKF->currents_position.HeadingAngle; // Should be the data from sensors PSI
    }




    EKF->estimates_position.X_Position      =  estimates(0); // X Messwerte
    EKF->estimates_position.Y_Position      =  estimates(1); // Y Messwerte
    EKF->estimates_position.HeadingAngle    =  estimates(2); // Psi Messwerte


    // Compute transition matrix
    //
    /*F << 1, 0, -sin(estimates(2) + ((lf/(lf+lr))*XX[4]))*XX[3]*DT,
          0, 1,  cos(estimates(2) + ((lf/(lf+lr))*XX[4]))*XX[3]*DT,
          0, 0, 1;*/

//    F << 1, 0, -sin(estimates(2) + ((lf/(lf+lr))*EKF->steering_angle))*EKF->currents_position.speed*EKF->delta_time,
//          0, 1,  cos(estimates(2) + ((lf/(lf+lr))*EKF->steering_angle))*EKF->currents_position.speed*EKF->delta_time,
//          0, 0, 1;

//    H << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;

//    Q << 10, 0,  0,
//          0, 10, 0,
//          0,  0, 10;

//    R << 15, 0,  0,
//          0,  15, 0,
//          0,  0, 15;


//    if(car_cur_position.radius > 0.5)
    {
        PP = FF * PP * FF + QQ;
//        P = F * P * F.transpose() + Q;
//        Matrix3d T = (H * P * H.transpose() + R);
        double TT = (HH * PP * HH + RR);

//        I << 1, 0, 0,
//             0, 1, 0,
//             0, 0, 1;

//        T = T.inverse().eval();
        TT = 1.0/TT;
//        Matrix3d KG = P * H.transpose() * T;
        double KG = PP * HH * TT;
        if(fabs(EKF->last_heading - EKF->currents_position.HeadingAngle) > 3.0)
            estimates(2) =( estimates(2) * -1.0 );
//        estimates = estimates + KG * (sensors - H * estimates);

        estimates(2) = estimates(2) + KG * (sensors(2) - HH * estimates(2));

//        P = (I - KG * H) * P;
        PP = (1.0 - KG * HH) * PP;

//        EKF->estimates_position.X_Position      =  estimates(0); // X Messwerte
//        EKF->estimates_position.Y_Position      =  estimates(1); // Y Messwerte
//        EKF->estimates_position.HeadingAngle    =  estimates(2); // Psi Messwerte
    }
    EKF->last_heading = EKF->currents_position.HeadingAngle;


    //    Xini[0] = estimates(0); // X Messwerte
    //    Xini[1] = estimates(1); // Y Messwerte
    //    Xini[2] = estimates(2); // Psi Messwerte

    /* KALMAN FILTER END */
}


double dotX1(double *sVars, double *uu, double h)
{
    return h * (uu[0]*cos((sVars[2]+((lf / l) * uu[1]))));
}

double dotX2(double *sVars, double *uu, double h)
{
    return h * (uu[0]*sin((sVars[2]+((lf / l)*uu[1]))));
}

double dotX3(double *sVars, double *uu, double h)
{
    return h * ((uu[0]/l)*tan(uu[1]));
}
