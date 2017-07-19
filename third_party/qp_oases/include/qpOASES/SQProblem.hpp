/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qpOASES/SQProblem.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the SQProblem class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming
 *	with varying matrices.
 */



#ifndef QPOASES_SQPROBLEM_HPP
#define QPOASES_SQPROBLEM_HPP


#include <qpOASES/QProblem.hpp>


BEGIN_NAMESPACE_QPOASES


/**
 *	\brief Implements the online active set strategy for QPs with varying matrices.
 *
 *	A class for setting up and solving quadratic programs with varying QP matrices.
 *	The main feature is the possibily to use the newly developed online active set strategy
 *	for parametric quadratic programming.
 *
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 */
class SQProblem : public QProblem
{
	/* allow SolutionAnalysis class to access private members */
	friend class SolutionAnalysis;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		SQProblem( );

		/** Constructor which takes the QP dimension and Hessian type
		 *  information. If the Hessian is the zero (i.e. HST_ZERO) or the
		 *  identity matrix (i.e. HST_IDENTITY), respectively, no memory
		 *  is allocated for it and a NULL pointer can be passed for it
		 *  to the init() functions. */
		SQProblem(	int_t _nV,	  							/**< Number of variables. */
					int_t _nC,  							/**< Number of constraints. */
					HessianType _hessianType = HST_UNKNOWN,	/**< Type of Hessian matrix. */
					BooleanType allocDenseMats = BT_TRUE	/**< Enable allocation of dense matrices. */
					);

		/** Copy constructor (deep copy). */
		SQProblem(	const SQProblem& rhs	/**< Rhs object. */
					);

		/** Destructor. */
		virtual ~SQProblem( );

		/** Assignment operator (deep copy). */
		virtual SQProblem& operator=(	const SQProblem& rhs	/**< Rhs object. */
								);


		/** Solves an initialised QP sequence with matrix shift using
		 *	the online active set strategy.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MAX_NWSR_REACHED \n
		 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
					RET_HOTSTART_FAILED \n
					RET_MATRIX_SHIFT_FAILED \n
					RET_SHIFT_DETERMINATION_FAILED \n
					RET_STEPDIRECTION_DETERMINATION_FAILED \n
					RET_STEPLENGTH_DETERMINATION_FAILED \n
					RET_HOMOTOPY_STEP_FAILED \n
					RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS \n
					RET_SETUP_AUXILIARYQP_FAILED */
		returnValue hotstart(	SymmetricMatrix *H_new,							/**< Hessian matrix of neighbouring QP to be solved (a shallow copy is made). \n
																					 If Hessian matrix is trivial, a NULL pointer can be passed. */
								const real_t* const g_new,						/**< Gradient of neighbouring QP to be solved. */
								Matrix *A_new,									/**< Constraint matrix of neighbouring QP to be solved (a shallow copy is made). \n
																					 If QP sequence does not involve constraints, a NULL pointer can be passed. */
								const real_t* const lb_new,						/**< Lower bounds of neighbouring QP to be solved. \n
													 								 If no lower bounds exist, a NULL pointer can be passed. */
								const real_t* const ub_new,						/**< Upper bounds of neighbouring QP to be solved. \n
													 		 						 If no upper bounds exist, a NULL pointer can be passed. */
								const real_t* const lbA_new,					/**< Lower constraints' bounds of neighbouring QP to be solved. \n
													 		 						 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const real_t* const ubA_new,					/**< Upper constraints' bounds of neighbouring QP to be solved. \n
												 			 						 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,									/**< Input: Maximum number of working set recalculations; \n
																					 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP solution. \n
																					 Output: CPU time spen for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of bounds is kept!) */
								const Constraints* const guessedConstraints = 0	/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of constraints is kept!) */
								);

		/** Solves an initialised QP sequence with matrix shift using
		 *	the online active set strategy.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MAX_NWSR_REACHED \n
		 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
					RET_HOTSTART_FAILED \n
					RET_MATRIX_SHIFT_FAILED \n
					RET_SHIFT_DETERMINATION_FAILED \n
					RET_STEPDIRECTION_DETERMINATION_FAILED \n
					RET_STEPLENGTH_DETERMINATION_FAILED \n
					RET_HOMOTOPY_STEP_FAILED \n
					RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS \n
					RET_SETUP_AUXILIARYQP_FAILED */
		returnValue hotstart(	const real_t* const H_new,						/**< Hessian matrix of neighbouring QP to be solved (a shallow copy is made). \n
																					 If Hessian matrix is trivial, a NULL pointer can be passed. */
								const real_t* const g_new,						/**< Gradient of neighbouring QP to be solved. */
								const real_t* const A_new,						/**< Constraint matrix of neighbouring QP to be solved (a shallow copy is made). \n
																					 If QP sequence does not involve constraints, a NULL pointer can be passed. */
								const real_t* const lb_new,						/**< Lower bounds of neighbouring QP to be solved. \n
													 							 	 If no lower bounds exist, a NULL pointer can be passed. */
								const real_t* const ub_new,						/**< Upper bounds of neighbouring QP to be solved. \n
													 							 	 If no upper bounds exist, a NULL pointer can be passed. */
								const real_t* const lbA_new,					/**< Lower constraints' bounds of neighbouring QP to be solved. \n
													 							 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const real_t* const ubA_new,					/**< Upper constraints' bounds of neighbouring QP to be solved. \n
												 								 	 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,									/**< Input: Maximum number of working set recalculations; \n
																					 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP solution. \n
																					 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of bounds is kept!) */
								const Constraints* const guessedConstraints = 0	/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of constraints is kept!) */
								);

		/** Solves an initialised QP sequence with matrix shift using
		 *	the online active set strategy, where QP data is read from files.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MAX_NWSR_REACHED \n
		 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
					RET_HOTSTART_FAILED \n
					RET_MATRIX_SHIFT_FAILED \n
					RET_SHIFT_DETERMINATION_FAILED \n
					RET_STEPDIRECTION_DETERMINATION_FAILED \n
					RET_STEPLENGTH_DETERMINATION_FAILED \n
					RET_HOMOTOPY_STEP_FAILED \n
					RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS \n
					RET_SETUP_AUXILIARYQP_FAILED \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS */
		returnValue hotstart(	const char* const H_file,						/**< Name of file where Hessian matrix is stored. \n
																					 If Hessian matrix is trivial, a NULL pointer can be passed. */
								const char* const g_file,						/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
								const char* const A_file,						/**< Name of file where constraint matrix is stored. \n
																					 If QP sequence does not involve constraints, a NULL pointer can be passed. */
								const char* const lb_file,						/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no lower bounds exist, a NULL pointer can be passed. */
								const char* const ub_file, 						/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no upper bounds exist, a NULL pointer can be passed. */
								const char* const lbA_file,						/**< Name of file where lower constraints' bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const char* const ubA_file,						/**< Name of file where upper constraints' bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,									/**< Input: Maximum number of working set recalculations; \n
																					 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP solution. \n
																					 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of bounds is kept!) */
								const Constraints* const guessedConstraints = 0	/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of constraints is kept!) */
								);

		/** Solves an initialised QP sequence (without matrix shift) using
		 *	the online active set strategy.
		 *	By default, QP solution is started from previous solution. If a guess
		 *	for the working set is provided, an initialised homotopy is performed.
		 *
		 *  Note: This functions just forwards to the corresponding
		 *  	  QProblem::hotstart member function.
		 *
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MAX_NWSR_REACHED \n
		 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
					RET_HOTSTART_FAILED \n
					RET_SHIFT_DETERMINATION_FAILED \n
					RET_STEPDIRECTION_DETERMINATION_FAILED \n
					RET_STEPLENGTH_DETERMINATION_FAILED \n
					RET_HOMOTOPY_STEP_FAILED \n
					RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS */
		returnValue hotstart(	const real_t* const g_new,						/**< Gradient of neighbouring QP to be solved. */
								const real_t* const lb_new,						/**< Lower bounds of neighbouring QP to be solved. \n
													 							 	 If no lower bounds exist, a NULL pointer can be passed. */
								const real_t* const ub_new,						/**< Upper bounds of neighbouring QP to be solved. \n
													 							 	 If no upper bounds exist, a NULL pointer can be passed. */
								const real_t* const lbA_new,					/**< Lower constraints' bounds of neighbouring QP to be solved. \n
													 							 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const real_t* const ubA_new,					/**< Upper constraints' bounds of neighbouring QP to be solved. \n
													 							 	 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,									/**< Input: Maximum number of working set recalculations; \n
																			 		 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP solution. \n
																				 	 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of bounds is kept!) */
								const Constraints* const guessedConstraints = 0	/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of constraints is kept!) */
								);

		/** Solves an initialised QP sequence (without matrix shift) using
		 *  the online active set strategy, where QP data is read from files.
		 *	By default, QP solution is started from previous solution. If a guess
		 *	for the working set is provided, an initialised homotopy is performed.
		 *
		 *  Note: This functions just forwards to the corresponding
		 *  	  QProblem::hotstart member function.
		 *
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MAX_NWSR_REACHED \n
		 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
					RET_HOTSTART_FAILED \n
					RET_SHIFT_DETERMINATION_FAILED \n
					RET_STEPDIRECTION_DETERMINATION_FAILED \n
					RET_STEPLENGTH_DETERMINATION_FAILED \n
					RET_HOMOTOPY_STEP_FAILED \n
					RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS */
		returnValue hotstart(	const char* const g_file,						/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
								const char* const lb_file,					 	/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no lower bounds exist, a NULL pointer can be passed. */
								const char* const ub_file,						/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no upper bounds exist, a NULL pointer can be passed. */
								const char* const lbA_file,						/**< Name of file where lower constraints' bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const char* const ubA_file,						/**< Name of file where upper constraints' bounds, of neighbouring QP to be solved, is stored. \n
													 								 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR, 									/**< Input: Maximum number of working set recalculations; \n
																					 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP solution. \n
																				 	 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of bounds is kept!) */
								const Constraints* const guessedConstraints = 0	/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																					 (If a null pointer is passed, the previous working set of constraints is kept!) */
								);


		#ifdef __MATLAB__
		/** Sets pointer of Hessian and constraint matrix to zero.
		 *  QUICK HACK FOR MAKING THE MATLAB INTERFACE RUN! TO BE REMOVED!
		 *	\return SUCCESSFUL_RETURN */
		returnValue resetMatrixPointers( );
		#endif


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:

		/** Sets new matrices and calculates their factorisations. If the
		 *  current Hessian is trivial (i.e. HST_ZERO or HST_IDENTITY) but a
		 *  non-trivial one is given, memory for Hessian is allocated and
		 *  it is set to the given one. Afterwards, all QP vectors are
		 *  transformed in order to start from an optimal solution.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_MATRIX_FACTORISATION_FAILED \n
		 * 			RET_NO_HESSIAN_SPECIFIED */
		virtual returnValue setupNewAuxiliaryQP(	SymmetricMatrix *H_new,		/**< New Hessian matrix. \n
																					 If Hessian matrix is trivial, a NULL pointer can be passed. */
													Matrix *A_new,				/**< New constraint matrix. \n
																					 If QP sequence does not involve constraints, a NULL pointer can be passed. */
													const real_t *lb_new,		/**< New lower bounds. \n
														 						 	 If no lower bounds exist, a NULL pointer can be passed. */
													const real_t *ub_new,		/**< New upper bounds. \n
														 						 	 If no lower bounds exist, a NULL pointer can be passed. */
													const real_t *lbA_new,		/**< New lower constraints' bounds. \n
														 						 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
													const real_t *ubA_new		/**< New lower constraints' bounds. \n
														 						 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
													);

		/** Sets new matrices and calculates their factorisations. If the
		 *  current Hessian is trivial (i.e. HST_ZERO or HST_IDENTITY) but a
		 *  non-trivial one is given, memory for Hessian is allocated and
		 *  it is set to the given one. Afterwards, all QP vectors are
		 *  transformed in order to start from an optimal solution.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_MATRIX_FACTORISATION_FAILED \n
		 * 			RET_NO_HESSIAN_SPECIFIED */
		virtual returnValue setupNewAuxiliaryQP(	const real_t* const H_new,	/**< New Hessian matrix. \n
																	     		 	 If Hessian matrix is trivial, a NULL pointer can be passed. */
													const real_t* const A_new,	/**< New constraint matrix. \n
																	     		 	 If QP sequence does not involve constraints, a NULL pointer can be passed. */
													const real_t *lb_new,		/**< New lower bounds. \n
														 						 	 If no lower bounds exist, a NULL pointer can be passed. */
													const real_t *ub_new,		/**< New upper bounds. \n
														 						 	 If no lower bounds exist, a NULL pointer can be passed. */
													const real_t *lbA_new,		/**< New lower constraints' bounds. \n
														 						 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
													const real_t *ubA_new		/**< New lower constraints' bounds. \n
														 						 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
													);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:

};


END_NAMESPACE_QPOASES

#include <qpOASES/SQProblem.ipp>

#endif	/* QPOASES_SQPROBLEM_HPP */


/*
 *	end of file
 */
