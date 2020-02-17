#ifndef SOP_NLP_HPP_
#define SOP_NLP_HPP_

#include "IpTNLP.hpp"
#include "SOP_NMPC.h"
#include <sys/time.h>

#include <iostream>


using namespace Ipopt;


class AUDI_Q2_NLP : public TNLP
{

public:
  /** default constructor */
  AUDI_Q2_NLP();

  /** default destructor */
  virtual ~AUDI_Q2_NLP();

  /** Method to return some info about the NLP */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

  /** Method to return the constraint vector */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  /** Method to return:
   *   1) The structure of the Jacobian (if "values" is NULL)
   *   2) The values of the Jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  /** Method to return:
   *   1) The structure of the Hessian of the Lagrangian (if "values" is NULL)
   *   2) The values of the Hessian of the Lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values);

  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
				 const IpoptData* ip_data,
				 IpoptCalculatedQuantities* ip_cq);

  /** Method to solve nonlinear equation system */
  virtual void     newton_solver(const Number* x);

  /** Method to compute first order sensitivities */
  virtual MatrixXd compute_sensitivities(VectorXd xx, const Number u1, const Number u2);

  /** Method to return function values inside the nonlinear solver */
  virtual VectorXd lhs_nonlin_eq(VectorXd xx, VectorXd x0, const Number u1, const Number u2);

  /** Method to compute sensitivities with respect to state variables at collocation points */
  virtual MatrixXd dGdX(VectorXd xx, const Number u1, const Number u2);

private:

  AUDI_Q2_NLP(const AUDI_Q2_NLP&);
  AUDI_Q2_NLP& operator=(const AUDI_Q2_NLP&);
};

#endif /* AUDI_Q2_NLP_HPP_ */
