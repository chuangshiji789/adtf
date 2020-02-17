#include "SOP_NMPC.h"
#include "SOP_nlp.h"

#include <cassert>
#include <typeinfo>

const double a_obs=1600;
const double b_obs=100;
const double xobs=800;
const double yobs=66;

extern NMPC_STRUCT *Nmpc;

SmartPtr<TNLP> mynlp;
SmartPtr<IpoptApplication> app;
ApplicationReturnStatus status;


// Constructor
AUDI_Q2_NLP::AUDI_Q2_NLP()
{}
// Destructor
AUDI_Q2_NLP::~AUDI_Q2_NLP()
{}

/* Returns the size of the problem */
bool AUDI_Q2_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                               Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    ///Total number of variables: (N_x + N_u) * N + N_x
    n = Nmpc->N_dt * Nmpc->N_xu + Nmpc->N_x;


    /// Total number of constraints: (N+1)*N_x - continuity condition between time intervals
    ///					 		    N+1     - obstacle constraints (inequalities)
        m = (Nmpc->N_dt + 1) * Nmpc->N_x + Nmpc->N_dt  + 1;

    // without obstacle constraints!
    //m = (Nmpc->N_dt + 1) * Nmpc->N_x;


    ///Number of non-zeros in Jacobian and Hessian matrices.
    ///Due to the BFGS method, the number of non-zeros in Hessian is equal to zero
    //    // Continuity + Obstacle + Last collocation point
        nnz_jac_g = (Nmpc->N_dt + 1) * Nmpc->N_x + (Nmpc->N_dt + 1) * 2 + Nmpc->N_dt * Nmpc->N_x * Nmpc->N_xu;

    // Continuity + Last collocation point
    //nnz_jac_g = (Nmpc->N_dt + 1) * Nmpc->N_x + Nmpc->N_dt * Nmpc->N_x * Nmpc->N_xu;

    nnz_h_lag = 0;
    index_style = C_STYLE;
    return true;
}

// Returns the variable bounds
bool AUDI_Q2_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                  Index m, Number* g_l, Number* g_u)
{
    int i,k;

    //// Construct bounds of variables for all time intervals
    // Boundaries for optimization variables
    for(k = 0; k < Nmpc->N_dt; k++)
    {
        for(i=0;i<Nmpc->N_xu;i++)
        {
            x_l[k * Nmpc->N_xu + i] = Nmpc->additional_x.lower[i];
            x_u[k * Nmpc->N_xu + i] = Nmpc->additional_x.upper[i];
        }
    }
    // Boundaries for terminal state variables
    for(i=0;i<Nmpc->N_x;i++)
    {
        x_l[Nmpc->N_dt * Nmpc->N_xu + i] = Nmpc->additional_x.lower[i];
        x_u[Nmpc->N_dt * Nmpc->N_xu + i] = Nmpc->additional_x.upper[i];
    }


    // Boundaries of terminal state variables
    for(i = 0;i < ((Nmpc->N_dt + 1) * Nmpc->N_x); i++)
    {
        g_l[i]=0;
        g_u[i]=0;
    }


        for(i=(Nmpc->N_dt + 1) * Nmpc->N_x; i< m; i++)
        {
            g_l[i] = 1;
            g_u[i] = 1e19;
        } // without obstacle constraints

    return true;
}

// returns the initial point for the problem
bool AUDI_Q2_NLP::get_starting_point(Index n, bool init_x, Number* x,
                                     bool init_z, Number* z_L, Number* z_U,
                                     Index m, bool init_lambda,
                                     Number* lambda)
{
    int index = 0;
    for(index = 0; index < n; index++)
    {
        x[index]   = Nmpc->optim_variables[index];
        z_L[index] = Nmpc->lag_mul_lower_bounds[index];
        z_U[index] = Nmpc->lag_mul_upper_bounds[index];
    }

    for(index = 0; index< m; index++)
    {
        lambda[index] = Nmpc->lag_mul_constraints[index];
    }


    return true;
}


// returns the value of the objective function
bool AUDI_Q2_NLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    //objective function
    ObjectiveFunction(n, x, obj_value, Nmpc);

    return true;
}


// return the gradient of the objective function grad_{x} f(x)
// change
bool AUDI_Q2_NLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    int i;
    //use memset
    for(i=0; i< n; i++)
        grad_f[i] = 0;

    //Lane keeping: y, steering, delta_steering

    if(Nmpc->driving_model_flag==LANE_KEEPING)
    {
        for(i = 0; i < Nmpc->N_dt; i++)
          {
              grad_f[i * Nmpc->N_xu + 1] = 2 * Nmpc->weight.y * (x[i * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[i]);
              grad_f[i * Nmpc->N_xu + 4] = 2 * Nmpc->weight.steering * x[i * Nmpc->N_xu + 4];
          }


          grad_f[4] += Nmpc->weight.steering_ChangeRate*(-2*x[Nmpc->N_xu+4]+2*x[4]);

          for(i = 1; i < (Nmpc->N_dt - 1); i++)
          {
              grad_f[i*Nmpc->N_xu+4] += Nmpc->weight.steering_ChangeRate*(-2*x[(i-1)*Nmpc->N_xu+4]+4*x[i*Nmpc->N_xu+4]-2*x[(i+1)*Nmpc->N_xu+4]);
          }

          grad_f[(Nmpc->N_dt-1) * Nmpc->N_xu + 4] += Nmpc->weight.steering_ChangeRate*(2*x[(Nmpc->N_dt-1) * Nmpc->N_xu+4]-2*x[(Nmpc->N_dt-2) * Nmpc->N_xu+4]);

    }
    //x,y,theta,steering,final_x,final_y,delta_steering
    else
     {
        //crossing
        for(i = 0; i < Nmpc->N_dt; i++)
        {
            grad_f[i * Nmpc->N_xu + 0] = 2 * Nmpc->weight.x * (x[i * Nmpc->N_xu + 0] - Nmpc->trajectory_setpoint.X[i]);
            grad_f[i * Nmpc->N_xu + 1] = 2 * Nmpc->weight.y * (x[i * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[i]);
//            grad_f[i * Nmpc->N_xu + 2] = 2 * Nmpc->weight.theta * (x[i * Nmpc->N_xu + 2] - Nmpc->trajectory_setpoint.Theta[i]);
    //        grad_f[i * Nmpc->N_xu + 3] = 2 * Nmpc->weight.speed * (x[i * Nmpc->N_xu + 3] - Nmpc->vcc);
            grad_f[i * Nmpc->N_xu + 4] = 2 * Nmpc->weight.steering * x[i * Nmpc->N_xu + 4];
        }

//        grad_f[Nmpc->N_dt * Nmpc->N_xu + 0] = 2*Nmpc->weight.final_x*(x[Nmpc->N_dt * Nmpc->N_xu + 0] - Nmpc->trajectory_setpoint.X[Nmpc->N_dt]);
//        grad_f[Nmpc->N_dt * Nmpc->N_xu + 1] = 2*Nmpc->weight.final_y*(x[Nmpc->N_dt * Nmpc->N_xu + 1] - Nmpc->trajectory_setpoint.Y[Nmpc->N_dt]);

//        grad_f[3] += Nmpc->weight.speed_ChangeRate*(-2*x[Nmpc->N_xu+3]+2*x[3]);
        grad_f[4] += Nmpc->weight.steering_ChangeRate*(-2*x[Nmpc->N_xu+4]+2*x[4]);

        for(i = 1; i < (Nmpc->N_dt - 1); i++)
        {
    //        grad_f[i*Nmpc->N_xu+3] +=Nmpc->weight.speed_ChangeRate*(-2*x[(i-1)*Nmpc->N_xu+3]+4*x[i*Nmpc->N_xu+3]-2*x[(i+1)*Nmpc->N_xu+3]);

            grad_f[i*Nmpc->N_xu+4] += Nmpc->weight.steering_ChangeRate*(-2*x[(i-1)*Nmpc->N_xu+4]+4*x[i*Nmpc->N_xu+4]-2*x[(i+1)*Nmpc->N_xu+4]);
        }

    //    grad_f[(Nmpc->N_dt-1) * Nmpc->N_xu + 3] += Nmpc->weight.speed_ChangeRate*(2*x[(Nmpc->N_dt-1) * Nmpc->N_xu+3]-2*x[(Nmpc->N_dt-2) * Nmpc->N_xu+3]);
        grad_f[(Nmpc->N_dt-1) * Nmpc->N_xu + 4] += Nmpc->weight.steering_ChangeRate*(2*x[(Nmpc->N_dt-1) * Nmpc->N_xu+4]-2*x[(Nmpc->N_dt-2) * Nmpc->N_xu+4]);

    }



    return true;
}



// return the value of the constraints: g(x)
bool AUDI_Q2_NLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{

    if(new_x)
        newton_solver(x);

    int i,j,k;
    for(i=0; i<Nmpc->N_x; i++)
    {
        g[i] = x[i] - Nmpc->OUTPUT[i]; // Only initials at time point t_0
    }
    for(j=1;j<Nmpc->N_dt+1;j++)
    {
        for(i=0; i<Nmpc->N_x; i++)
        {
            g[j*Nmpc->N_x + i] = x[j*Nmpc->N_xu + i] - Nmpc->OUTPUT[j*Nmpc->N_x + i]; // Continuity between intervals
        }
    }
        for(k=0; k<=Nmpc->N_dt ; k++)
        {
                g[(Nmpc->N_dt+1)*Nmpc->N_x + k] = pow(x[k*Nmpc->N_xu + 0] - xobs,2)/a_obs + pow(x[k*Nmpc->N_xu + 1] - yobs,2)/b_obs;
        } // without obstacle constraints

    // change
    //    for(i=0; i<=N ; i++)
    //    {
    //            g[(N+1)*Nmpc->N_x + i] = pow(x[i*Nmpc->N_xu + 0] - xobs,2)/a_obs + pow(x[i*Nmpc->N_xu + 1] - yobs,2)/b_obs;
    //    } // without obstacle constraints

    return true;
}

// return the structure or values of the jacobian
bool AUDI_Q2_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
                             Index m, Index nele_jac, Index* iRow, Index *jCol,
                             Number* values)
{
    int i, j, k;
    if (values == NULL)
    {
        // return the structure of the jacobian of the constraints

        for(j=0;j<Nmpc->N_dt+1;j++)
        {
            for(i=0; i<Nmpc->N_x; i++)
            {
                iRow[j*Nmpc->N_x + i] = j*Nmpc->N_x + i;
                jCol[j*Nmpc->N_x + i] = j*Nmpc->N_xu + i;
            }
        }

        for(k=0;k<Nmpc->N_dt;k++)
        {
            for(i=0; i<Nmpc->N_x; i++)
            {
                for(j=0; j<Nmpc->N_xu; j++)
                {
                    iRow[(Nmpc->N_dt+1)*Nmpc->N_x + k*Nmpc->N_x*Nmpc->N_xu + i*Nmpc->N_xu + j] = (k+1)*Nmpc->N_x + i;
                    jCol[(Nmpc->N_dt+1)*Nmpc->N_x + k*Nmpc->N_x*Nmpc->N_xu + i*Nmpc->N_xu + j] = k*Nmpc->N_xu + j;
                }
            }
        }
                for(i=0;i<=Nmpc->N_dt;i++)
                {
                    for(j=0;j<2;j++)
                    {
                        iRow[(Nmpc->N_dt+1)*Nmpc->N_x + Nmpc->N_dt*Nmpc->N_x*Nmpc->N_xu + 2*i + j] = (Nmpc->N_dt+1)*Nmpc->N_x + i;
                        jCol[(Nmpc->N_dt+1)*Nmpc->N_x + Nmpc->N_dt*Nmpc->N_x*Nmpc->N_xu + 2*i + j] = i*Nmpc->N_xu + 0 + j; // the 1 while Y-coord in vector is 1. The position of X-cooNmpc->weight.steeringinate is 0
                    }
                }//without obstacle constraints
    }
    else
    {
        //if(new_x)
        //	newton_solver(x);
        for(j=0;j<Nmpc->N_dt+1;j++)
        {
            for(i=0; i<Nmpc->N_x; i++)
            {
                values[j*Nmpc->N_x + i] = 1;
            }
        }
        for(k=0;k<Nmpc->N_dt;k++)
        {
            for(i=0; i<Nmpc->N_x; i++)
            {
                for(j=0; j<Nmpc->N_xu; j++)
                {
                    values[(Nmpc->N_dt+1)*Nmpc->N_x + k*Nmpc->N_x*Nmpc->N_xu + i * Nmpc->N_xu + j] = -Nmpc->dxdu(k * Nmpc->N_x + i, j);
                }
            }
        }
        //change
              for(i=0;i<=Nmpc->N_dt;i++)
              {
                      values[(Nmpc->N_dt+1)*Nmpc->N_x + Nmpc->N_dt*Nmpc->N_x*Nmpc->N_xu + 2*i] = 2*(x[i*Nmpc->N_xu + 0] - xobs)/a_obs;
                      values[(Nmpc->N_dt+1)*Nmpc->N_x + Nmpc->N_dt*Nmpc->N_x*Nmpc->N_xu + 2*i + 1] = 2*(x[i*Nmpc->N_xu + 1] - yobs)/b_obs;
              } // without obstacle constraint
    }

    return true;
}


//return the structure or values of the Hessian
bool AUDI_Q2_NLP::eval_h(Index n, const Number* x, bool new_x,
                         Number obj_factor, Index m, const Number* lambda,
                         bool new_lambda, Index nele_hess, Index* iRow,
                         Index* jCol, Number* values)
{

    return true;
}


void AUDI_Q2_NLP::finalize_solution(SolverReturn status,
                                    Index n, const Number* x, const Number* z_L, const Number* z_U,
                                    Index m, const Number* g, const Number* lambda,
                                    Number obj_value,
                                    const IpoptData* ip_data,
                                    IpoptCalculatedQuantities* ip_cq)
{

    int index = 0;
    for(index = 0; index < n; index++)
    {
        Nmpc->optim_variables[index] = x[index];
    }

    for(index = 0; index < m; index++)
    {
        Nmpc->lag_mul_constraints[index] = lambda[index];
    }

    for(index = 0; index < Nmpc->N_x; index++)
    {
        Nmpc->lag_mul_lower_bounds[index] = z_L[index];
        Nmpc->lag_mul_upper_bounds[index] = z_U[index];
    }
/*
        int k = 0;
        for (Index i=Nmpc->N_x; i<n; i+=Nmpc->N_xu) {
           std::cout << " Velocity  and Steering angle"<< k << "-th interval " << x[i] << "  " << x[i+1] << std::endl;
           k++;
        }

        int l = 0;
        for (Index i=3; i<n; i+=Nmpc->N_xu) {
            std::cout << " X-coordinate in the "<< l << "-th interval " << "x[" << i << "] = " << x[i] << std::endl;
            l++;
        }
*/
}

void AUDI_Q2_NLP::newton_solver(const Number* x)
{

    int i,k;
    int index = 0;
    Number u1; Number u2;
    VectorXd xlocal(Nmpc->N_x);
    VectorXd xcoll(Nmpc->N_x * 3);

    for(index = 0; index < Nmpc->N_x; index++)
    {
        Nmpc->OUTPUT[index] = Nmpc->additional_x.ini[index];
    }

    // k - interval counter
    // i - state counter
    for(k=0;k<Nmpc->N_dt;k++)
    {
        //std::cout << "Interval k " << k << std::endl;
        // Set initial guess for all collocation points
        int idx = 0;
        for(i=0;i<3;i++)
        {
            xcoll(idx)   = x[k*Nmpc->N_xu+0];
            xcoll(idx+1) = x[k*Nmpc->N_xu+1];
            xcoll(idx+2) = x[k*Nmpc->N_xu+2];
            idx = idx + 3;
        }

        // Set parameterized initial state variables
        xlocal(0)  = x[k*Nmpc->N_xu+0];
        xlocal(1)  = x[k*Nmpc->N_xu+1];
        xlocal(2)  = x[k*Nmpc->N_xu+2];

        // Set parameterized control variables
        u1 = x[k*Nmpc->N_xu+3];
        u2 = x[k*Nmpc->N_xu+4];
        //std::cout << xlocal(0) << " " << xlocal(1) << " "<< xlocal(2) << " " << u1 << " " << u2<< std::endl;
        //std::cout << "----------------"<< std::endl;
        // Evaluate nonlinear equation system
        VectorXd G = lhs_nonlin_eq(xcoll, xlocal, u1, u2);

        // Evaluate Jacobian matrix w.r.t. all collocation points
        MatrixXd dGdxcoll = dGdX(xcoll, u1, u2);
        //std::cout << dGdxcoll << endl;
        while(G.norm() > 1e-5)
        {
            xcoll = xcoll - dGdxcoll.lu().solve(G);
            dGdxcoll = dGdX(xcoll, u1, u2);
            G = lhs_nonlin_eq(xcoll, xlocal, u1, u2);
        }

        for(i = 0; i < Nmpc->N_x; i++)
            Nmpc->OUTPUT[(k + 1) * Nmpc->N_x + i] = xcoll(2 * Nmpc->N_x + i);

        Nmpc->dxdu.block(k*Nmpc->N_x,0,Nmpc->N_x,Nmpc->N_xu) = compute_sensitivities(xcoll, u1, u2);
        //std::cout << dxdu.block(k*Nmpc->N_x,0,Nmpc->N_x,Nmpc->N_xu) << endl;
    }

}

MatrixXd AUDI_Q2_NLP::compute_sensitivities(VectorXd xx, const Number u1, const Number u2)
{

    MatrixXd out = MatrixXd::Zero(Nmpc->N_x,Nmpc->N_xu);
    MatrixXd dGdxcoll = dGdX(xx, u1, u2);
    MatrixXd rhs = MatrixXd::Zero(3*Nmpc->N_x,Nmpc->N_xu);

    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);
    double one_l = (1 / Nmpc->total_axis_length);
    double u1_l  = (u1 / Nmpc->total_axis_length);

    rhs( 0 , 0 )= Nmpc->collocation_matrix(0,3) ;
    rhs( 3 , 0 )= Nmpc->collocation_matrix(1,3) ;
    rhs( 6 , 0 )= Nmpc->collocation_matrix(2,3) ;
    rhs( 1 , 1 )= Nmpc->collocation_matrix(0,3) ;
    rhs( 4 , 1 )= Nmpc->collocation_matrix(1,3) ;
    rhs( 7 , 1 )= Nmpc->collocation_matrix(2,3) ;
    rhs( 2 , 2 )= Nmpc->collocation_matrix(0,3) ;
    rhs( 5 , 2 )= Nmpc->collocation_matrix(1,3) ;
    rhs( 8 , 2 )= Nmpc->collocation_matrix(2,3) ;
    rhs( 0 , 3 )= (Nmpc->delta_time * (-cos((xx(2) + (lf_l * u2))))) ;
    rhs( 1 , 3 )= (Nmpc->delta_time * (-sin((xx(2) + (lf_l * u2))))) ;
    rhs( 2 , 3 )= (Nmpc->delta_time * (-(one_l * tan(u2)))) ;
    rhs( 3 , 3 )= (Nmpc->delta_time *(-cos((xx(5) + (lf_l * u2))))) ;
    rhs( 4 , 3 )= (Nmpc->delta_time *(-sin((xx(5) + (lf_l * u2))))) ;
    rhs( 5 , 3 )= (Nmpc->delta_time *(-(one_l * tan(u2)))) ;
    rhs( 6 , 3 )= (Nmpc->delta_time *(-cos((xx(8) + (lf_l * u2))))) ;
    rhs( 7 , 3 )= (Nmpc->delta_time *(-sin((xx(8) + (lf_l * u2))))) ;
    rhs( 8 , 3 )= (Nmpc->delta_time *(-(one_l * tan(u2)))) ;
    rhs( 0 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(2) + (lf_l * u2)))))))) ;
    rhs( 1 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(2) + (lf_l * u2))))))) ;
    rhs( 2 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2), 2)))) ;
    rhs( 3 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(5) + (lf_l * u2)))))))) ;
    rhs( 4 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(5) + (lf_l * u2))))))) ;
    rhs( 5 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2), 2)))) ;
    rhs( 6 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l * (-sin((xx(8) + (lf_l * u2)))))))) ;
    rhs( 7 , 4 )= (Nmpc->delta_time *(-(u1 * (lf_l *   cos((xx(8) + (lf_l * u2))))))) ;
    rhs( 8 , 4 )= (Nmpc->delta_time *(-(u1_l / pow(cos(u2),2)))) ;

    MatrixXd dXdX0U = dGdxcoll.lu().solve(-rhs);
    out << dXdX0U.row(6), dXdX0U.row(7), dXdX0U.row(8);


    return out;
}

VectorXd AUDI_Q2_NLP::lhs_nonlin_eq(VectorXd xx, VectorXd x0, const Number u1, const Number u2)
{

    VectorXd F(Nmpc->N_x*3);

    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);
    double u1_l  = (u1 / Nmpc->total_axis_length);

    /* xx - State variables at collocation points
     * x0 - Parameterized initial state variables
     * u1 - Control variable -> Velocity
     * u2 - Control variable -> Steering angle
     * */

    F(0) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(0)) + (Nmpc->collocation_matrix(0,1) * xx(3))) + (Nmpc->collocation_matrix(0,2) * xx(6))) + (Nmpc->collocation_matrix(0,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(2) + (lf_l * u2)))));
    F(1) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(1)) + (Nmpc->collocation_matrix(0,1) * xx(4))) + (Nmpc->collocation_matrix(0,2) * xx(7))) + (Nmpc->collocation_matrix(0,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(2) + (lf_l * u2)))));
    F(2) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(0,0) * xx(2)) + (Nmpc->collocation_matrix(0,1) * xx(5))) + (Nmpc->collocation_matrix(0,2) * xx(8))) + (Nmpc->collocation_matrix(0,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));
    F(3) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(0)) + (Nmpc->collocation_matrix(1,1) * xx(3))) + (Nmpc->collocation_matrix(1,2) * xx(6))) + (Nmpc->collocation_matrix(1,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(5) + (lf_l * u2)))));
    F(4) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(1)) + (Nmpc->collocation_matrix(1,1) * xx(4))) + (Nmpc->collocation_matrix(1,2) * xx(7))) + (Nmpc->collocation_matrix(1,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(5) + (lf_l * u2)))));
    F(5) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(1,0) * xx(2)) + (Nmpc->collocation_matrix(1,1) * xx(5))) + (Nmpc->collocation_matrix(1,2) * xx(8))) + (Nmpc->collocation_matrix(1,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));
    F(6) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(0)) + (Nmpc->collocation_matrix(2,1) * xx(3))) + (Nmpc->collocation_matrix(2,2) * xx(6))) + (Nmpc->collocation_matrix(2,3) * x0(0))) / Nmpc->delta_time) - (u1 * cos((xx(8) + (lf_l * u2)))));
    F(7) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(1)) + (Nmpc->collocation_matrix(2,1) * xx(4))) + (Nmpc->collocation_matrix(2,2) * xx(7))) + (Nmpc->collocation_matrix(2,3) * x0(1))) / Nmpc->delta_time) - (u1 * sin((xx(8) + (lf_l * u2)))));
    F(8) = Nmpc->delta_time *((((((Nmpc->collocation_matrix(2,0) * xx(2)) + (Nmpc->collocation_matrix(2,1) * xx(5))) + (Nmpc->collocation_matrix(2,2) * xx(8))) + (Nmpc->collocation_matrix(2,3) * x0(2))) / Nmpc->delta_time) - (u1_l * tan(u2)));

    return F;
}

MatrixXd AUDI_Q2_NLP::dGdX(VectorXd xx, const Number u1, const Number u2)
{

    MatrixXd mdGdX = MatrixXd::Zero(3*Nmpc->N_x,3*Nmpc->N_x);

    double lf_l  = (Nmpc->front_axis_length / Nmpc->total_axis_length);

    mdGdX( 0 , 0 )= Nmpc->collocation_matrix(0,0) ;
    mdGdX( 3 , 0 )= Nmpc->collocation_matrix(1,0) ;
    mdGdX( 6 , 0 )= Nmpc->collocation_matrix(2,0) ;
    mdGdX( 1 , 1 )= Nmpc->collocation_matrix(0,0) ;
    mdGdX( 4 , 1 )= Nmpc->collocation_matrix(1,0) ;
    mdGdX( 7 , 1 )= Nmpc->collocation_matrix(2,0) ;
    mdGdX( 0 , 2 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(2) + (lf_l * u2))))))) ;
    mdGdX( 1 , 2 )= (Nmpc->delta_time * (-(u1 *   cos((xx(2) + (lf_l * u2)))))) ;
    mdGdX( 2 , 2 )= Nmpc->collocation_matrix(0,0) ;
    mdGdX( 5 , 2 )= Nmpc->collocation_matrix(1,0) ;
    mdGdX( 8 , 2 )= Nmpc->collocation_matrix(2,0) ;
    mdGdX( 0 , 3 )= Nmpc->collocation_matrix(0,1) ;
    mdGdX( 3 , 3 )= Nmpc->collocation_matrix(1,1) ;
    mdGdX( 6 , 3 )= Nmpc->collocation_matrix(2,1) ;
    mdGdX( 1 , 4 )= Nmpc->collocation_matrix(0,1) ;
    mdGdX( 4 , 4 )= Nmpc->collocation_matrix(1,1) ;
    mdGdX( 7 , 4 )= Nmpc->collocation_matrix(2,1) ;
    mdGdX( 2 , 5 )= Nmpc->collocation_matrix(0,1) ;
    mdGdX( 3 , 5 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(5)+(lf_l  *u2))))))) ;
    mdGdX( 4 , 5 )= (Nmpc->delta_time * (-(u1 *   cos((xx(5)+(lf_l * u2)))))) ;
    mdGdX( 5 , 5 )= Nmpc->collocation_matrix(1,1) ;
    mdGdX( 8 , 5 )= Nmpc->collocation_matrix(2,1) ;
    mdGdX( 0 , 6 )= Nmpc->collocation_matrix(0,2) ;
    mdGdX( 3 , 6 )= Nmpc->collocation_matrix(1,2) ;
    mdGdX( 6 , 6 )= Nmpc->collocation_matrix(2,2) ;
    mdGdX( 1 , 7 )= Nmpc->collocation_matrix(0,2) ;
    mdGdX( 4 , 7 )= Nmpc->collocation_matrix(1,2) ;
    mdGdX( 7 , 7 )= Nmpc->collocation_matrix(2,2) ;
    mdGdX( 2 , 8 )= Nmpc->collocation_matrix(0,2) ;
    mdGdX( 5 , 8 )= Nmpc->collocation_matrix(1,2) ;
    mdGdX( 6 , 8 )= (Nmpc->delta_time * (-(u1 * (-sin((xx(8) + (lf_l * u2))))))) ;
    mdGdX( 7 , 8 )= (Nmpc->delta_time * (-(u1 *   cos((xx(8) + (lf_l * u2)))))) ;
    mdGdX( 8 , 8 )= Nmpc->collocation_matrix(2,2) ;

    return mdGdX;
}


int RunIpopt(NMPC_STRUCT * Nmpc)
{
//    timeval time_value;
//    long int ipopt_start_time = 0;
//    long int ipopt_endt_time = 0;
//    Nmpc->ipopt_operation_time = 0;
//    gettimeofday(&time_value, 0);
//    ipopt_start_time = time_value.tv_sec * 1000000 + time_value.tv_usec;

    status = app->OptimizeTNLP(mynlp);

//    gettimeofday(&time_value, 0);
//    ipopt_endt_time = time_value.tv_sec * 1000000 + time_value.tv_usec;
//    Nmpc->ipopt_operation_time = (float)(ipopt_endt_time - ipopt_start_time) / 1000000.;
//        std::cout << "Ipopt time measurement: " << Nmpc->ipopt_operation_time << std::endl;

    return status;
}



////Ipopt seting
int SetIpopt(void)
{

    mynlp = new AUDI_Q2_NLP();
    app = IpoptApplicationFactory();
    app->RethrowNonIpoptException(true);

    // Set options for optimizer
    app->Options()->SetNumericValue("tol", 1e-5);
    app->Options()->SetNumericValue("acceptable_tol", 1e-5);
    app->Options()->SetIntegerValue("acceptable_iter", 0);
    //app->Options()->SetStringValue ("mu_strategy", "adaptive");
    app->Options()->SetStringValue ("linear_solver", "mumps");
    app->Options()->SetStringValue ("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level", 0); //print ipopt information
    app->Options()->SetStringValue ("warm_start_init_point", "yes");
    app->Options()->SetIntegerValue("max_iter",23);
    //app->Options()->SetNumericValue("constr_viol_tol",0.001);
    app->Options()->SetNumericValue("warm_start_bound_push",1e-6);
    app->Options()->SetNumericValue("warm_start_mult_bound_push",1e-6);
    app->Options()->SetStringValue("sb", "yes");

    status = app->Initialize();

    if (status != Solve_Succeeded)
    {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return (int) status;
    }


    return status;
}



bool CloseIpopt(void)
{
    app->~IpoptApplication();
    app->~ReferencedObject();
    return true;
}



