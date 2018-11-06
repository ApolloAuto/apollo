/*----------------------------------------------------------------------------
 ADOL-C -- Automatic Differentiation by Overloading in C++
 File:     ADOL-C_sparseNLP.hpp
 Revision: $$
 Contents:  class myADOL-C_sparseNPL for interfacing with Ipopt
 
 Copyright (c) Andrea Walther
   
 This file is part of ADOL-C. This software is provided as open source.
 Any use, reproduction, or distribution of the software constitutes 
 recipient's acceptance of the terms of the accompanying license file.
 
 This code is based on the file  MyNLP.hpp contained in the Ipopt package
 with the authors:  Carl Laird, Andreas Waechter   
----------------------------------------------------------------------------*/

//*************************************************************************
//
//
//         Nothing has to be changed in this file !!
//
//
//*************************************************************************

#ifndef __MYADOLCNLP_HPP__
#define __MYADOLCNLP_HPP__

#include <IpTNLP.hpp>
#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>

#define tag_f 1
#define tag_g 2
#define tag_L 3
#define HPOFF 30

using namespace Ipopt;

class MyADOLC_sparseNLP : public TNLP
{
public:
  /** default constructor */
  MyADOLC_sparseNLP();

  /** default destructor */
  virtual ~MyADOLC_sparseNLP();

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
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

  /** Template to return the objective value */
  template<class T> bool eval_obj(Index n, const T *x, T& obj_value);

  
  /** Template to compute contraints */
  template<class T> bool eval_constraints(Index n, const T *x, Index m, T *g);

  /** Original method from Ipopt to return the objective value */
  /** remains unchanged */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

  /** Original method from Ipopt to return the gradient of the objective */
  /** remains unchanged */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

  /**  Original method from Ipopt to return the constraint residuals */
  /** remains unchanged */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  /** Original method from Ipopt to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  /** remains unchanged */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  /** Original method from Ipopt to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  /** remains unchanged */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values);

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
				 const IpoptData* ip_data,
				 IpoptCalculatedQuantities* ip_cq);
  //@}

//***************    start ADOL-C part ***********************************

 /** Method to generate the required tapes */
  virtual void generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag);

//***************    end   ADOL-C part ***********************************

private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually 
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *  
   */
  //@{
  //  MyADOLC_sparseNLP();
  MyADOLC_sparseNLP(const MyADOLC_sparseNLP&);
  MyADOLC_sparseNLP& operator=(const MyADOLC_sparseNLP&);
  //@}

  //@{

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
  
  //@}

};

#endif
