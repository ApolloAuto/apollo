/*----------------------------------------------------------------------------
 ADOL-C -- Automatic Differentiation by Overloading in C++
 File:     ADOL-C_sparseNLP.cpp
 Revision: $$
 Contents: class myADOLC_sparseNPL for interfacing with Ipopt
 
 Copyright (c) Andrea Walther
   
 This file is part of ADOL-C. This software is provided as open source.
 Any use, reproduction, or distribution of the software constitutes 
 recipient's acceptance of the terms of the accompanying license file.
 
 This code is based on the file  MyNLP.cpp contained in the Ipopt package
 with the authors:  Carl Laird, Andreas Waechter   
----------------------------------------------------------------------------*/

/** C++ Example NLP for interfacing a problem with IPOPT and ADOL-C.
 *  MyADOLC_sparseNLP implements a C++ example showing how to interface 
 *  with IPOPT and ADOL-C through the TNLP interface. This class 
 *  implements the Example 5.1 from "Sparse and Parially Separable
 *  Test Problems for Unconstrained and Equality Constrained
 *  Optimization" by L. Luksan and J. Vlcek taking sparsity
 *  into account.
 *
 *  exploitation of sparsity !!
 *
 */

#include <cassert>

#include "ADOL-C_sparseNLP.hpp"

using namespace Ipopt;

/* Constructor. */
MyADOLC_sparseNLP::MyADOLC_sparseNLP()
{}

MyADOLC_sparseNLP::~MyADOLC_sparseNLP()
{}

bool MyADOLC_sparseNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  n = 4000;

  m = n-2;

  generate_tapes(n, m, nnz_jac_g, nnz_h_lag);

  // use the C style indexing (0-based)
  index_style = C_STYLE;

  return true;
}

bool MyADOLC_sparseNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
  // none of the variables have bounds
  for (Index i=0; i<n; i++) {
    x_l[i] = -1e20;
    x_u[i] =  1e20;
  }

  // Set the bounds for the constraints
  for (Index i=0; i<m; i++) {
    g_l[i] = 0;
    g_u[i] = 0;
  }

  return true;
}

bool MyADOLC_sparseNLP::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the others if
  // you wish.
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  // set the starting point
  for (Index i=0; i<n/2; i++) {
    x[2*i] = -1.2;
    x[2*i+1] = 1.;
  }
  if (n != 2*(n/2)) {
    x[n-1] = -1.2;
  }

  return true;
}

template<class T> bool  MyADOLC_sparseNLP::eval_obj(Index n, const T *x, T& obj_value)
{
  T a1, a2;
  obj_value = 0.;
  for (Index i=0; i<n-1; i++) {
    a1 = x[i]*x[i]-x[i+1];
    a2 = x[i] - 1.;
    obj_value += 100.*a1*a1 + a2*a2;
  }

  return true;
}

template<class T> bool  MyADOLC_sparseNLP::eval_constraints(Index n, const T *x, Index m, T* g)
{
  for (Index i=0; i<m; i++) {
    g[i] = 3.*pow(x[i+1],3.) + 2.*x[i+2] - 5.
           + sin(x[i+1]-x[i+2])*sin(x[i+1]+x[i+2]) + 4.*x[i+1]
           - x[i]*exp(x[i]-x[i+1]) - 3.;
  }

  return true;
}

//*************************************************************************
//
//
//         Nothing has to be changed below this point !!
//
//
//*************************************************************************


bool MyADOLC_sparseNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  eval_obj(n,x,obj_value);

  return true;
}

bool MyADOLC_sparseNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{

  gradient(tag_f,n,x,grad_f);

  return true;
}

bool MyADOLC_sparseNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{

  eval_constraints(n,x,m,g);

  return true;
}

bool MyADOLC_sparseNLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{

  if (values == NULL) {
    // return the structure of the jacobian

    for(Index idx=0; idx<nnz_jac; idx++)
      {
	iRow[idx] = rind_g[idx];
	jCol[idx] = cind_g[idx];
      }
  }
  else {
    // return the values of the jacobian of the constraints

    sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g); 

    for(Index idx=0; idx<nnz_jac; idx++)
      {
	values[idx] = jacval[idx];

      }
  }
  return true;
}

bool MyADOLC_sparseNLP::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{

  if (values == NULL) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    for(Index idx=0; idx<nnz_L; idx++)
      {
	iRow[idx] = rind_L[idx];
	jCol[idx] = cind_L[idx];
      }
  }
  else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only

    obj_lam[0] = obj_factor;
    for(Index idx = 0; idx<m ; idx++)
      obj_lam[1+idx] = lambda[idx];

    set_param_vec(tag_L,m+1,obj_lam);
    sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
     
    for(Index idx = 0; idx <nnz_L ; idx++)
      {
          values[idx] = hessval[idx];
      }
  }

  return true;
}

void MyADOLC_sparseNLP::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			      const IpoptData* ip_data,
			      IpoptCalculatedQuantities* ip_cq)
{

  printf("\n\nObjective value\n");
  printf("f(x*) = %e\n", obj_value);

// memory deallocation of ADOL-C variables

  delete[] obj_lam;
  free(rind_g);
  free(cind_g);
  free(rind_L);
  free(cind_L);
  free(jacval);
  free(hessval);

}


//***************    ADOL-C part ***********************************

void MyADOLC_sparseNLP::generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag)
{
  Number *xp    = new double[n];
  Number *lamp  = new double[m];
  Number *zl    = new double[m];
  Number *zu    = new double[m];

  adouble *xa   = new adouble[n];
  adouble *g    = new adouble[m];
  double *lam   = new double[m];
  double sig;
  adouble obj_value;
  
  double dummy;

  int i,j,k,l,ii;

  obj_lam   = new double[m+1];

  get_starting_point(n, 1, xp, 0, zl, zu, m, 0, lamp);

  trace_on(tag_f);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

  trace_off();
  
  trace_on(tag_g);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];

    eval_constraints(n,xa,m,g);


    for(Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

  trace_off();

  trace_on(tag_L);
    
    for(Index idx=0;idx<n;idx++)
      xa[idx] <<= xp[idx];
    for(Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);
 
    for(Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

  trace_off();

  rind_g = NULL; 
  cind_g = NULL;
  rind_L = NULL;
  cind_L = NULL;

  options_g[0] = 0;          /* sparsity pattern by index domains (default) */ 
  options_g[1] = 0;          /*                         safe mode (default) */ 
  options_g[2] = 0;
  options_g[3] = 0;          /*                column compression (default) */ 
  
  jacval=NULL;
  hessval=NULL;
  sparse_jac(tag_g, m, n, 0, xp, &nnz_jac, &rind_g, &cind_g, &jacval, options_g); 

  nnz_jac_g = nnz_jac;

  options_L[0] = 0;         
  options_L[1] = 1;        

  sparse_hess(tag_L, n, 0, xp, &nnz_L, &rind_L, &cind_L, &hessval, options_L);
  nnz_h_lag = nnz_L;

  delete[] lam;
  delete[] g;
  delete[] xa;
  delete[] zu;
  delete[] zl;
  delete[] lamp;
  delete[] xp;
}
