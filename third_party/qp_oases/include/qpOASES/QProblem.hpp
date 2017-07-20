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
 *	\file include/qpOASES/QProblem.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the QProblem class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming.
 */



#ifndef QPOASES_QPROBLEM_HPP
#define QPOASES_QPROBLEM_HPP


#include <qpOASES/QProblemB.hpp>
#include <qpOASES/Constraints.hpp>
#include <qpOASES/ConstraintProduct.hpp>
#include <qpOASES/Matrices.hpp>


BEGIN_NAMESPACE_QPOASES


/**
 *	\brief Implements the online active set strategy for QPs with general constraints.
 *
 *	A class for setting up and solving quadratic programs. The main feature is
 *	the possibily to use the newly developed online active set strategy for
 * 	parametric quadratic programming.
 *
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 */
class QProblem : public QProblemB
{
	/* allow SolutionAnalysis class to access private members */
	friend class SolutionAnalysis;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		QProblem( );

		/** Constructor which takes the QP dimension and Hessian type
		 *  information. If the Hessian is the zero (i.e. HST_ZERO) or the
		 *  identity matrix (i.e. HST_IDENTITY), respectively, no memory
		 *  is allocated for it and a NULL pointer can be passed for it
		 *  to the init() functions. */
		QProblem(	int_t _nV,	  							/**< Number of variables. */
					int_t _nC,		  						/**< Number of constraints. */
					HessianType _hessianType = HST_UNKNOWN,	/**< Type of Hessian matrix. */
					BooleanType allocDenseMats = BT_TRUE	/**< Enable allocation of dense matrices. */
					);

		/** Copy constructor (deep copy). */
		QProblem(	const QProblem& rhs		/**< Rhs object. */
					);

		/** Destructor. */
		virtual ~QProblem( );

		/** Assignment operator (deep copy). */
		virtual QProblem& operator=(	const QProblem& rhs	/**< Rhs object. */
								);


		/** Clears all data structures of QProblemB except for QP data.
		 *	\return SUCCESSFUL_RETURN \n
					RET_RESET_FAILED */
		virtual returnValue reset( );


		/** Initialises a QP problem with given QP data and tries to solve it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0    : starts with xOpt = 0, yOpt = 0 and gB/gC empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0    : starts with xOpt, yOpt = 0 and obtain gB/gC by "clipping", \n
		 *	3. 0,    yOpt, 0    : starts with xOpt = 0, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	4. 0,    0,    gB/gC: starts with xOpt = 0, yOpt = 0 and gB/gC, \n
		 *	5. xOpt, yOpt, 0    : starts with xOpt, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	6. xOpt, 0,    gB/gC: starts with xOpt, yOpt = 0 and gB/gC, \n
		 *	7. xOpt, yOpt, gB/gC: starts with xOpt, yOpt and gB/gC (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		  *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_TQ \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_INVALID_ARGUMENTS */
		returnValue init(	SymmetricMatrix *_H,							/**< Hessian matrix (a shallow copy is made). */
							const real_t* const _g, 						/**< Gradient vector. */
							Matrix *_A,  									/**< Constraint matrix (a shallow copy is made). */
							const real_t* const _lb,						/**< Lower bound vector (on variables). \n
																				 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const _ub,						/**< Upper bound vector (on variables). \n
																				 If no upper bounds exist, a NULL pointer can be passed. */
							const real_t* const _lbA,						/**< Lower constraints' bound vector. \n
																				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							const real_t* const _ubA,						/**< Upper constraints' bound vector. \n
																				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							int_t& nWSR,									/**< Input: Maximum number of working set recalculations when using initial homotopy.
																				 Output: Number of performed working set recalculations. */
							real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP initialisation. \n
																				 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,					/**< Optimal primal solution vector. \n
																				 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,					/**< Optimal dual solution vector. \n
																				 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all bounds are assumed inactive!) */
							const Constraints* const guessedConstraints = 0,/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all constraints are assumed inactive!) */
							const real_t* const _R = 0						/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix.
																			 	 The Cholesky factor must be stored in a real_t array of size nV*nV
																				 in row-major format. Note: Only used if xOpt/yOpt and gB are NULL! \n
																				 (If a null pointer is passed, Cholesky decomposition is computed internally!) */
							);


		/** Initialises a QP problem with given QP data and tries to solve it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0    : starts with xOpt = 0, yOpt = 0 and gB/gC empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0    : starts with xOpt, yOpt = 0 and obtain gB/gC by "clipping", \n
		 *	3. 0,    yOpt, 0    : starts with xOpt = 0, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	4. 0,    0,    gB/gC: starts with xOpt = 0, yOpt = 0 and gB/gC, \n
		 *	5. xOpt, yOpt, 0    : starts with xOpt, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	6. xOpt, 0,    gB/gC: starts with xOpt, yOpt = 0 and gB/gC, \n
		 *	7. xOpt, yOpt, gB/gC: starts with xOpt, yOpt and gB/gC (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_TQ \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_INVALID_ARGUMENTS */
		returnValue init(	const real_t* const _H,							/**< Hessian matrix (a shallow copy is made). \n
																				 If Hessian matrix is trivial, a NULL pointer can be passed. */
							const real_t* const _g,							/**< Gradient vector. */
							const real_t* const _A,							/**< Constraint matrix (a shallow copy is made). */
							const real_t* const _lb,						/**< Lower bound vector (on variables). \n
																				 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const _ub,						/**< Upper bound vector (on variables). \n
																				 If no upper bounds exist, a NULL pointer can be passed. */
							const real_t* const _lbA,						/**< Lower constraints' bound vector. \n
																				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							const real_t* const _ubA,						/**< Upper constraints' bound vector. \n
																				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							int_t& nWSR,									/**< Input: Maximum number of working set recalculations when using initial homotopy.
																				 Output: Number of performed working set recalculations. */
							real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP initialisation. \n
																				 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,					/**< Optimal primal solution vector. \n
																				 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,					/**< Optimal dual solution vector. \n
																				 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all bounds are assumed inactive!) */
							const Constraints* const guessedConstraints = 0,/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all constraints are assumed inactive!) */
							const real_t* const _R = 0						/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix.
																			 	 The Cholesky factor must be stored in a real_t array of size nV*nV
																				 in row-major format. Note: Only used if xOpt/yOpt and gB are NULL! \n
																				 (If a null pointer is passed, Cholesky decomposition is computed internally!) */
							);

		/** Initialises a QP problem with given data to be read from files and solves it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0    : starts with xOpt = 0, yOpt = 0 and gB/gC empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0    : starts with xOpt, yOpt = 0 and obtain gB/gC by "clipping", \n
		 *	3. 0,    yOpt, 0    : starts with xOpt = 0, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	4. 0,    0,    gB/gC: starts with xOpt = 0, yOpt = 0 and gB/gC, \n
		 *	5. xOpt, yOpt, 0    : starts with xOpt, yOpt and obtain gB/gC from yOpt != 0, \n
		 *	6. xOpt, 0,    gB/gC: starts with xOpt, yOpt = 0 and gB/gC, \n
		 *	7. xOpt, yOpt, gB/gC: starts with xOpt, yOpt and gB/gC (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_TQ \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS */
		returnValue init(	const char* const H_file,						/**< Name of file where Hessian matrix is stored. \n
																				 If Hessian matrix is trivial, a NULL pointer can be passed. */
							const char* const g_file,						/**< Name of file where gradient vector is stored. */
							const char* const A_file,						/**< Name of file where constraint matrix is stored. */
							const char* const lb_file,						/**< Name of file where lower bound vector. \n
																				 If no lower bounds exist, a NULL pointer can be passed. */
							const char* const ub_file,						/**< Name of file where upper bound vector. \n
																				 If no upper bounds exist, a NULL pointer can be passed. */
							const char* const lbA_file,						/**< Name of file where lower constraints' bound vector. \n
																				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							const char* const ubA_file,						/**< Name of file where upper constraints' bound vector. \n
																				 If no upper constraints' bounds exist, a NULL pointer can be passed. */
							int_t& nWSR,									/**< Input: Maximum number of working set recalculations when using initial homotopy.
																				 Output: Number of performed working set recalculations. */
							real_t* const cputime = 0,						/**< Input: Maximum CPU time allowed for QP initialisation. \n
																				 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,					/**< Optimal primal solution vector. \n
																				 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,					/**< Optimal dual solution vector. \n
																				 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,			/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all bounds are assumed inactive!) */
							const Constraints* const guessedConstraints = 0,/**< Optimal working set of constraints for solution (xOpt,yOpt). \n
																				 (If a null pointer is passed, all constraints are assumed inactive!) */
							const char* const R_file = 0					/**< Name of the file where a pre-computed (upper triangular) Cholesky factor
																			 	 of the Hessian matrix is stored. \n
																				 (If a null pointer is passed, Cholesky decomposition is computed internally!) */
							);


		/** Solves an initialised QP sequence using the online active set strategy.
		 *	By default, QP solution is started from previous solution. If a guess
		 *	for the working set is provided, an initialised homotopy is performed.
		 *
		 *  Note: This function internally calls solveQP/solveRegularisedQP
		 *        for solving an initialised QP!
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

		/** Solves an initialised QP sequence using the online active set strategy,
		 *  where QP data is read from files.
		 *	By default, QP solution is started from previous solution. If a guess
		 *	for the working set is provided, an initialised homotopy is performed.
		 *
		 *  Note: This function internally calls solveQP/solveRegularisedQP
		 *        for solving an initialised QP!
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


        /** Solves an equality-constrained QP problem resulting from the current working set.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_STEPDIRECTION_FAILED_TQ \n
		 *			RET_STEPDIRECTION_FAILED_CHOLESKY \n
		 *			RET_INVALID_ARGUMENTS */
        returnValue solveCurrentEQP (	const int_t n_rhs,			/**< Number of consecutive right hand sides */
										const real_t* g_in,			/**< Gradient of neighbouring QP to be solved. */
										const real_t* lb_in,		/**< Lower bounds of neighbouring QP to be solved. \n
																		 If no lower bounds exist, a NULL pointer can be passed. */
										const real_t* ub_in,		/**< Upper bounds of neighbouring QP to be solved. \n
																		 If no upper bounds exist, a NULL pointer can be passed. */
										const real_t* lbA_in,		/**< Lower constraints' bounds of neighbouring QP to be solved. \n
																		 If no lower constraints' bounds exist, a NULL pointer can be passed. */
										const real_t* ubA_in,		/**< Upper constraints' bounds of neighbouring QP to be solved. \n */
										real_t* x_out,				/**< Output: Primal solution */
										real_t* y_out				/**< Output: Dual solution */
										);

		/** Writes a vector with the state of the working set
		 *	\return SUCCESSFUL_RETURN \n
		 *	        RET_INVALID_ARGUMENTS */
		virtual returnValue getWorkingSet(	real_t* workingSet		/** Output: array containing state of the working set. */
											);

		/** Writes a vector with the state of the working set of bounds
		 *	\return SUCCESSFUL_RETURN \n
		 *	        RET_INVALID_ARGUMENTS */
		virtual returnValue getWorkingSetBounds(	real_t* workingSetB		/** Output: array containing state of the working set of bounds. */
													);

		/** Writes a vector with the state of the working set of constraints
		 *	\return SUCCESSFUL_RETURN \n
		 *	        RET_INVALID_ARGUMENTS */
		virtual returnValue getWorkingSetConstraints(	real_t* workingSetC	/** Output: array containing state of the working set of constraints. */
														);


		/** Returns current constraints object of the QP (deep copy).
		  *	\return SUCCESSFUL_RETURN \n
		  			RET_QPOBJECT_NOT_SETUP */
		inline returnValue getConstraints(	Constraints& _constraints	/** Output: Constraints object. */
											) const;


		/** Returns the number of constraints.
		 *	\return Number of constraints. */
		inline int_t getNC( ) const;

		/** Returns the number of (implicitly defined) equality constraints.
		 *	\return Number of (implicitly defined) equality constraints. */
		inline int_t getNEC( ) const;

		/** Returns the number of active constraints.
		 *	\return Number of active constraints. */
		inline int_t getNAC( ) const;

		/** Returns the number of inactive constraints.
		 *	\return Number of inactive constraints. */
		inline int_t getNIAC( ) const;

		/** Returns the dimension of null space.
		 *	\return Dimension of null space. */
		virtual int_t getNZ( ) const;


		/** Returns the dual solution vector (deep copy).
		 *	\return SUCCESSFUL_RETURN \n
					RET_QP_NOT_SOLVED */
		virtual returnValue getDualSolution(	real_t* const yOpt	/**< Output: Dual solution vector (if QP has been solved). */
												) const;


		/** Defines user-defined routine for calculating the constraint product A*x
		 *	\return  SUCCESSFUL_RETURN \n */
		returnValue setConstraintProduct(	ConstraintProduct* const _constraintProduct
											);


		/** Prints concise list of properties of the current QP.
		 *	\return  SUCCESSFUL_RETURN \n */
		virtual returnValue printProperties( );

		/** Set the incoming array to true for each variable entry that is
			in the set of free variables */
		returnValue getFreeVariablesFlags( BooleanType* varIsFree );


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );

		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const QProblem& rhs	/**< Rhs object. */
							);

		/** Solves a QProblem whose QP data is assumed to be stored in the member variables.
		 *  A guess for its primal/dual optimal solution vectors and the corresponding
		 *  working sets of bounds and constraints can be provided.
		 *  Note: This function is internally called by all init functions!
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_TQ \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED */
		returnValue solveInitialQP(	const real_t* const xOpt,						/**< Optimal primal solution vector.*/
									const real_t* const yOpt,						/**< Optimal dual solution vector. */
									const Bounds* const guessedBounds,				/**< Optimal working set of bounds for solution (xOpt,yOpt). */
									const Constraints* const guessedConstraints,	/**< Optimal working set of constraints for solution (xOpt,yOpt). */
									const real_t* const _R,							/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix. */
									int_t& nWSR, 									/**< Input: Maximum number of working set recalculations; \n
																 						 Output: Number of performed working set recalculations. */
									real_t* const cputime							/**< Input: Maximum CPU time allowed for QP solution. \n
																			 			 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
									);

		/** Solves QProblem using online active set strategy.
		 *  Note: This function is internally called by all hotstart functions!
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
		returnValue solveQP(	const real_t* const g_new,			/**< Gradient of neighbouring QP to be solved. */
								const real_t* const lb_new,			/**< Lower bounds of neighbouring QP to be solved. \n
													 			 		 If no lower bounds exist, a NULL pointer can be passed. */
								const real_t* const ub_new,			/**< Upper bounds of neighbouring QP to be solved. \n
													 				 	 If no upper bounds exist, a NULL pointer can be passed. */
								const real_t* const lbA_new,		/**< Lower constraints' bounds of neighbouring QP to be solved. \n
													 			 		 If no lower constraints' bounds exist, a NULL pointer can be passed. */
								const real_t* const ubA_new,		/**< Upper constraints' bounds of neighbouring QP to be solved. \n
													 					 If no upper constraints' bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,						/**< Input: Maximum number of working set recalculations; \n
																 		 Output: Number of performed working set recalculations. */
								real_t* const cputime,				/**< Input: Maximum CPU time allowed for QP solution. \n
																	 	 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								int_t nWSRperformed = 0,			/**< Number of working set recalculations already performed to solve
																		 this QP within previous solveQP() calls. This number is
																		 always zero, except for successive calls from solveRegularisedQP()
																		 or when using the far bound strategy. */
								BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
								);


		/** Solves QProblem using online active set strategy.
		 *  Note: This function is internally called by all hotstart functions!
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
		returnValue solveRegularisedQP(	const real_t* const g_new,			/**< Gradient of neighbouring QP to be solved. */
										const real_t* const lb_new,			/**< Lower bounds of neighbouring QP to be solved. \n
															 				 	 If no lower bounds exist, a NULL pointer can be passed. */
										const real_t* const ub_new,			/**< Upper bounds of neighbouring QP to be solved. \n
															 				 	 If no upper bounds exist, a NULL pointer can be passed. */
										const real_t* const lbA_new,		/**< Lower constraints' bounds of neighbouring QP to be solved. \n
															 				 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
										const real_t* const ubA_new,		/**< Upper constraints' bounds of neighbouring QP to be solved. \n
															 				 	 If no upper constraints' bounds exist, a NULL pointer can be passed. */
										int_t& nWSR,						/**< Input: Maximum number of working set recalculations; \n
																		 		 Output: Number of performed working set recalculations. */
										real_t* const cputime,				/**< Input: Maximum CPU time allowed for QP solution. \n
																			 	 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
										int_t nWSRperformed = 0,			/**< Number of working set recalculations already performed to solve
																				 this QP within previous solveRegularisedQP() calls. This number is
																				 always zero, except for successive calls when using the far bound strategy. */
										BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
										);


		/** Update activities in a hot start if some of the bounds have
			become infinity or if variables have become fixed.  */
		/*	\return SUCCESSFUL_RETURN \n
					RET_HOTSTART_FAILED */
		virtual returnValue updateActivitiesForHotstart( const real_t* const lb_new,	/**< New lower bounds. */
														 const real_t* const ub_new,	/**< New upper bounds. */
														 const real_t* const lbA_new,	/**< New lower constraints' bounds. */
														 const real_t* const ubA_new	/**< New upper constraints' bounds. */
														 );


		/** Determines type of existing constraints and bounds (i.e. implicitly fixed, unbounded etc.).
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUPSUBJECTTOTYPE_FAILED */
		virtual returnValue setupSubjectToType( );

		/** Determines type of new constraints and bounds (i.e. implicitly fixed, unbounded etc.).
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUPSUBJECTTOTYPE_FAILED */
		using QProblemB::setupSubjectToType;
		virtual returnValue setupSubjectToType(	const real_t* const lb_new,		/**< New lower bounds. */
												const real_t* const ub_new,		/**< New upper bounds. */
												const real_t* const lbA_new,	/**< New lower constraints' bounds. */
												const real_t* const ubA_new		/**< New upper constraints' bounds. */
												);

		/** Computes the Cholesky decomposition of the projected Hessian (i.e. R^T*R = Z^T*H*Z).
		 *  Note: If Hessian turns out not to be positive definite, the Hessian type
		 *		  is set to HST_SEMIDEF accordingly.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_HESSIAN_NOT_SPD \n
		 *			RET_INDEXLIST_CORRUPTED */
		virtual returnValue computeProjectedCholesky( );

		/** Computes initial Cholesky decomposition of the projected Hessian making
		 *  use of the function computeCholesky() or computeProjectedCholesky().
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_HESSIAN_NOT_SPD \n
		 *			RET_INDEXLIST_CORRUPTED */
		virtual returnValue setupInitialCholesky( );

		/** Initialises TQ factorisation of A (i.e. A*Q = [0 T]) if NO constraint is active.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEXLIST_CORRUPTED */
		virtual returnValue setupTQfactorisation( );


		/** Obtains the desired working set for the auxiliary initial QP in
		 *  accordance with the user specifications
		 *  (assumes that member AX has already been initialised!)
		 *	\return SUCCESSFUL_RETURN \n
					RET_OBTAINING_WORKINGSET_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue obtainAuxiliaryWorkingSet(	const real_t* const xOpt,						/**< Optimal primal solution vector.
																								 *	 If a NULL pointer is passed, all entries are assumed to be zero. */
												const real_t* const yOpt,						/**< Optimal dual solution vector.
																								 *	 If a NULL pointer is passed, all entries are assumed to be zero. */
												const Bounds* const guessedBounds,				/**< Guessed working set of bounds for solution (xOpt,yOpt). */
												const Constraints* const guessedConstraints,	/**< Guessed working set for solution (xOpt,yOpt). */
												Bounds* auxiliaryBounds,						/**< Input: Allocated bound object. \n
																								 *	 Ouput: Working set of constraints for auxiliary QP. */
												Constraints* auxiliaryConstraints				/**< Input: Allocated bound object. \n
																								 *	 Ouput: Working set for auxiliary QP. */
												) const;

		/** Sets up bound and constraints data structures according to auxiliaryBounds/Constraints.
		 *  (If the working set shall be setup afresh, make sure that
		 *  bounds and constraints data structure have been resetted
		 *  and the TQ factorisation has been initialised!)
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUP_WORKINGSET_FAILED \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNOWN_BUG */
		virtual returnValue setupAuxiliaryWorkingSet(	const Bounds* const auxiliaryBounds,			/**< Working set of bounds for auxiliary QP. */
												const Constraints* const auxiliaryConstraints,	/**< Working set of constraints for auxiliary QP. */
												BooleanType setupAfresh							/**< Flag indicating if given working set shall be
																								 *    setup afresh or by updating the current one. */
												);

		/** Sets up the optimal primal/dual solution of the auxiliary initial QP.
		 *	\return SUCCESSFUL_RETURN */
		returnValue setupAuxiliaryQPsolution(	const real_t* const xOpt,			/**< Optimal primal solution vector.
																				 	*	 If a NULL pointer is passed, all entries are set to zero. */
												const real_t* const yOpt			/**< Optimal dual solution vector.
																					 *	 If a NULL pointer is passed, all entries are set to zero. */
												);

		/** Sets up gradient of the auxiliary initial QP for given
		 *  optimal primal/dual solution and given initial working set
		 *  (assumes that members X, Y and BOUNDS, CONSTRAINTS have already been initialised!).
		 *	\return SUCCESSFUL_RETURN */
		returnValue setupAuxiliaryQPgradient( );

		/** Sets up (constraints') bounds of the auxiliary initial QP for given
		 *  optimal primal/dual solution and given initial working set
		 *  (assumes that members X, Y and BOUNDS, CONSTRAINTS have already been initialised!).
		 *	\return SUCCESSFUL_RETURN \n
					RET_UNKNOWN_BUG */
		returnValue setupAuxiliaryQPbounds(	const Bounds* const auxiliaryBounds,			/**< Working set of bounds for auxiliary QP. */
											const Constraints* const auxiliaryConstraints,	/**< Working set of constraints for auxiliary QP. */
											BooleanType useRelaxation						/**< Flag indicating if inactive (constraints') bounds shall be relaxed. */
											);


		/** Adds a constraint to active set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_ADDCONSTRAINT_FAILED \n
					RET_ADDCONSTRAINT_FAILED_INFEASIBILITY \n
					RET_ENSURELI_FAILED */
		virtual returnValue addConstraint(	int_t number,					/**< Number of constraint to be added to active set. */
											SubjectToStatus C_status,		/**< Status of new active constraint. */
											BooleanType updateCholesky,		/**< Flag indicating if Cholesky decomposition shall be updated. */
											BooleanType ensureLI = BT_TRUE	/**< Ensure linear independence by exchange rules by default. */
											);

		/** Checks if new active constraint to be added is linearly dependent from
		 *	from row of the active constraints matrix.
		 *	\return	 RET_LINEARLY_DEPENDENT \n
		 			 RET_LINEARLY_INDEPENDENT \n
					 RET_INDEXLIST_CORRUPTED */
		virtual returnValue addConstraint_checkLI(	int_t number	/**< Number of constraint to be added to active set. */
													);

		/** Ensures linear independence of constraint matrix when a new constraint is added.
		 * 	To this end a bound or constraint is removed simultaneously if necessary.
		 *	\return	 SUCCESSFUL_RETURN \n
		 			 RET_LI_RESOLVED \n
					 RET_ENSURELI_FAILED \n
					 RET_ENSURELI_FAILED_TQ \n
					 RET_ENSURELI_FAILED_NOINDEX \n
					 RET_REMOVE_FROM_ACTIVESET */
		virtual returnValue addConstraint_ensureLI(	int_t number,				/**< Number of constraint to be added to active set. */
													SubjectToStatus C_status	/**< Status of new active bound. */
													);

		/** Adds a bound to active set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_ADDBOUND_FAILED \n
					RET_ADDBOUND_FAILED_INFEASIBILITY \n
					RET_ENSURELI_FAILED */
		virtual returnValue addBound(	int_t number,					/**< Number of bound to be added to active set. */
										SubjectToStatus B_status,		/**< Status of new active bound. */
										BooleanType updateCholesky,		/**< Flag indicating if Cholesky decomposition shall be updated. */
										BooleanType ensureLI = BT_TRUE	/**< Ensure linear independence by exchange rules by default. */
										);

		/** Checks if new active bound to be added is linearly dependent from
		 *	from row of the active constraints matrix.
		 *	\return	 RET_LINEARLY_DEPENDENT \n
		 			 RET_LINEARLY_INDEPENDENT */
		virtual returnValue addBound_checkLI(	int_t number	/**< Number of bound to be added to active set. */
												);

		/** Ensures linear independence of constraint matrix when a new bound is added.
		 *	To this end a bound or constraint is removed simultaneously if necessary.
		 *	\return	 SUCCESSFUL_RETURN \n
		 			 RET_LI_RESOLVED \n
					 RET_ENSURELI_FAILED \n
					 RET_ENSURELI_FAILED_TQ \n
					 RET_ENSURELI_FAILED_NOINDEX \n
					 RET_REMOVE_FROM_ACTIVESET */
		virtual returnValue addBound_ensureLI(	int_t number,				/**< Number of bound to be added to active set. */
												SubjectToStatus B_status	/**< Status of new active bound. */
												);

		/** Removes a constraint from active set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_CONSTRAINT_NOT_ACTIVE \n
					RET_REMOVECONSTRAINT_FAILED \n
					RET_HESSIAN_NOT_SPD */
		virtual returnValue removeConstraint(	int_t number,							/**< Number of constraint to be removed from active set. */
												BooleanType updateCholesky,				/**< Flag indicating if Cholesky decomposition shall be updated. */
												BooleanType allowFlipping = BT_FALSE,	/**< Flag indicating if flipping bounds are allowed. */
												BooleanType ensureNZC = BT_FALSE		/**< Flag indicating if non-zero curvature is ensured by exchange rules. */
												);

		/** Removes a bounds from active set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_BOUND_NOT_ACTIVE \n
					RET_HESSIAN_NOT_SPD \n
					RET_REMOVEBOUND_FAILED */
		virtual returnValue removeBound(	int_t number,							/**< Number of bound to be removed from active set. */
											BooleanType updateCholesky,				/**< Flag indicating if Cholesky decomposition shall be updated. */
											BooleanType allowFlipping = BT_FALSE,	/**< Flag indicating if flipping bounds are allowed. */
											BooleanType ensureNZC = BT_FALSE		/**< Flag indicating if non-zero curvature is ensured by exchange rules. */
											);


		/** Performs robustified ratio test yield the maximum possible step length
		 *  along the homotopy path.
		 *	\return  SUCCESSFUL_RETURN */
		returnValue performPlainRatioTest(	int_t nIdx, 						/**< Number of ratios to be checked. */
											const int_t* const idxList, 		/**< Array containing the indices of all ratios to be checked. */
											const real_t* const num,	 		/**< Array containing all numerators for performing the ratio test. */
											const real_t* const den,		 	/**< Array containing all denominators for performing the ratio test. */
											real_t epsNum,						/**< Numerator tolerance. */
											real_t epsDen,						/**< Denominator tolerance. */
											real_t& t,							/**< Output: Maximum possible step length along the homotopy path. */
											int_t& BC_idx 						/**< Output: Index of blocking constraint. */
											) const;


		/** Ensure non-zero curvature by primal jump.
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_HOTSTART_STOPPED_UNBOUNDEDNESS */
		returnValue ensureNonzeroCurvature(
				BooleanType removeBoundNotConstraint,	/**< SubjectTo to be removed is a bound. */
				int_t remIdx,							/**< Index of bound/constraint to be removed. */
				BooleanType &exchangeHappened,			/**< Output: Exchange was necessary to ensure. */
				BooleanType &addBoundNotConstraint,		/**< SubjectTo to be added is a bound. */
				int_t &addIdx,							/**< Index of bound/constraint to be added. */
				SubjectToStatus &addStatus				/**< Status of bound/constraint to be added. */
				);


		/** Solves the system Ta = b or T^Ta = b where T is a reverse upper triangular matrix.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_DIV_BY_ZERO */
		virtual returnValue backsolveT(	const real_t* const b,	/**< Right hand side vector. */
								BooleanType transposed,	/**< Indicates if the transposed system shall be solved. */
								real_t* const a 		/**< Output: Solution vector */
								) const;


		/** Determines step direction of the shift of the QP data.
		 *	\return SUCCESSFUL_RETURN */
		returnValue determineDataShift(	const real_t* const g_new,	/**< New gradient vector. */
										const real_t* const lbA_new,/**< New lower constraints' bounds. */
										const real_t* const ubA_new,/**< New upper constraints' bounds. */
										const real_t* const lb_new,	/**< New lower bounds. */
										const real_t* const ub_new,	/**< New upper bounds. */
										real_t* const delta_g,	 	/**< Output: Step direction of gradient vector. */
										real_t* const delta_lbA,	/**< Output: Step direction of lower constraints' bounds. */
										real_t* const delta_ubA,	/**< Output: Step direction of upper constraints' bounds. */
										real_t* const delta_lb,	 	/**< Output: Step direction of lower bounds. */
										real_t* const delta_ub,	 	/**< Output: Step direction of upper bounds. */
										BooleanType& Delta_bC_isZero,/**< Output: Indicates if active constraints' bounds are to be shifted. */
										BooleanType& Delta_bB_isZero/**< Output: Indicates if active bounds are to be shifted. */
										);

		/** Determines step direction of the homotopy path.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_STEPDIRECTION_FAILED_TQ \n
					RET_STEPDIRECTION_FAILED_CHOLESKY */
		virtual returnValue determineStepDirection(	const real_t* const delta_g,	/**< Step direction of gradient vector. */
											const real_t* const delta_lbA,	/**< Step direction of lower constraints' bounds. */
											const real_t* const delta_ubA,	/**< Step direction of upper constraints' bounds. */
											const real_t* const delta_lb,	/**< Step direction of lower bounds. */
											const real_t* const delta_ub,	/**< Step direction of upper bounds. */
											BooleanType Delta_bC_isZero, 	/**< Indicates if active constraints' bounds are to be shifted. */
											BooleanType Delta_bB_isZero,	/**< Indicates if active bounds are to be shifted. */
											real_t* const delta_xFX, 		/**< Output: Primal homotopy step direction of fixed variables. */
											real_t* const delta_xFR,	 	/**< Output: Primal homotopy step direction of free variables. */
											real_t* const delta_yAC, 		/**< Output: Dual homotopy step direction of active constraints' multiplier. */
											real_t* const delta_yFX 		/**< Output: Dual homotopy step direction of fixed variables' multiplier. */
											);

		/** Determines the maximum possible step length along the homotopy path
		 *  and performs this step (without changing working set).
		 *	\return SUCCESSFUL_RETURN \n
		 * 			RET_ERROR_IN_CONSTRAINTPRODUCT \n
		 * 			RET_QP_INFEASIBLE */
		returnValue performStep(	const real_t* const delta_g,		/**< Step direction of gradient. */
									const real_t* const delta_lbA,		/**< Step direction of lower constraints' bounds. */
									const real_t* const delta_ubA,		/**< Step direction of upper constraints' bounds. */
									const real_t* const delta_lb,	 	/**< Step direction of lower bounds. */
									const real_t* const delta_ub,	 	/**< Step direction of upper bounds. */
									const real_t* const delta_xFX, 		/**< Primal homotopy step direction of fixed variables. */
									const real_t* const delta_xFR,		/**< Primal homotopy step direction of free variables. */
									const real_t* const delta_yAC,		/**< Dual homotopy step direction of active constraints' multiplier. */
									const real_t* const delta_yFX,		/**< Dual homotopy step direction of fixed variables' multiplier. */
									int_t& BC_idx, 						/**< Output: Index of blocking constraint. */
									SubjectToStatus& BC_status,			/**< Output: Status of blocking constraint. */
									BooleanType& BC_isBound 			/**< Output: Indicates if blocking constraint is a bound. */
									);

		/** Updates the active set.
		 *	\return  SUCCESSFUL_RETURN \n
		 			 RET_REMOVE_FROM_ACTIVESET_FAILED \n
					 RET_ADD_TO_ACTIVESET_FAILED */
		returnValue changeActiveSet(	int_t BC_idx, 						/**< Index of blocking constraint. */
										SubjectToStatus BC_status,			/**< Status of blocking constraint. */
										BooleanType BC_isBound 				/**< Indicates if blocking constraint is a bound. */
										);


		/** Compute relative length of homotopy in data space for termination
		 *  criterion.
		 *  \return Relative length in data space. */
		real_t getRelativeHomotopyLength(	const real_t* const g_new,		/**< Final gradient. */
											const real_t* const lb_new,		/**< Final lower variable bounds. */
											const real_t* const ub_new,		/**< Final upper variable bounds. */
											const real_t* const lbA_new,	/**< Final lower constraint bounds. */
											const real_t* const ubA_new		/**< Final upper constraint bounds. */
											);


		/** Ramping Strategy to avoid ties. Modifies homotopy start without
		 *  changing current active set.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue performRamping( );


		/** ... */
		returnValue updateFarBounds(	real_t curFarBound,					/**< ... */
                                        int_t nRamp,						/**< ... */
                                        const real_t* const lb_new,			/**< ... */
                                        real_t* const lb_new_far,			/**< ... */
                                        const real_t* const ub_new,			/**< ... */
                                        real_t* const ub_new_far,			/**< ... */
                                        const real_t* const lbA_new,		/**< ... */
                                        real_t* const lbA_new_far,			/**< ... */
                                        const real_t* const ubA_new,		/**< ... */
                                        real_t* const ubA_new_far			/**< ... */
                                        ) const;


		/** Drift correction at end of each active set iteration
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue performDriftCorrection( );


		/** Updates QP vectors, working sets and internal data structures in order to
			start from an optimal solution corresponding to initial guesses of the working
			set for bounds and constraints.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_SETUP_AUXILIARYQP_FAILED \n
					RET_INVALID_ARGUMENTS */
		using QProblemB::setupAuxiliaryQP;
		virtual returnValue setupAuxiliaryQP(	const Bounds* const guessedBounds,			/**< Initial guess for working set of bounds. */
												const Constraints* const guessedConstraints	/**< Initial guess for working set of constraints. */
												);

		/** Determines if it is more efficient to refactorise the matrices when
		 *  hotstarting or not (i.e. better to update the existing factorisations).
		 *	\return BT_TRUE iff matrices shall be refactorised afresh
		 */
		BooleanType shallRefactorise(	const Bounds* const guessedBounds,			/**< Guessed new working set of bounds. */
										const Constraints* const guessedConstraints	/**< Guessed new working set of constraints. */
										) const;

		/** Sets up internal QP data.
		 *	\return SUCCESSFUL_RETURN \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNONW_BUG */
		returnValue setupQPdata(	SymmetricMatrix *_H, 		/**< Hessian matrix. \n
																     If Hessian matrix is trivial,a NULL pointer can be passed. */
									const real_t* const _g, 	/**< Gradient vector. */
									Matrix *_A, 			 	/**< Constraint matrix. */
									const real_t* const _lb,	/**< Lower bound vector (on variables). \n
																	 If no lower bounds exist, a NULL pointer can be passed. */
									const real_t* const _ub,	/**< Upper bound vector (on variables). \n
																	 If no upper bounds exist, a NULL pointer can be passed. */
									const real_t* const _lbA,	/**< Lower constraints' bound vector. \n
																	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
									const real_t* const _ubA	/**< Upper constraints' bound vector. \n
																	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
									);


		/** Sets up dense internal QP data. If the current Hessian is trivial
		 *  (i.e. HST_ZERO or HST_IDENTITY) but a non-trivial one is given,
		 *  memory for Hessian is allocated and it is set to the given one.
		 *	\return SUCCESSFUL_RETURN \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNONW_BUG */
		returnValue setupQPdata(	const real_t* const _H, 	/**< Hessian matrix. \n
																     If Hessian matrix is trivial,a NULL pointer can be passed. */
									const real_t* const _g, 	/**< Gradient vector. */
									const real_t* const _A,  	/**< Constraint matrix. */
									const real_t* const _lb,	/**< Lower bound vector (on variables). \n
																	 If no lower bounds exist, a NULL pointer can be passed. */
									const real_t* const _ub,	/**< Upper bound vector (on variables). \n
																	 If no upper bounds exist, a NULL pointer can be passed. */
									const real_t* const _lbA,	/**< Lower constraints' bound vector. \n
																	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
									const real_t* const _ubA	/**< Upper constraints' bound vector. \n
																	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
									);

		/** Sets up internal QP data by loading it from files. If the current Hessian
		 *  is trivial (i.e. HST_ZERO or HST_IDENTITY) but a non-trivial one is given,
		 *  memory for Hessian is allocated and it is set to the given one.
		 *	\return SUCCESSFUL_RETURN \n
					RET_UNABLE_TO_OPEN_FILE \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNONW_BUG */
		returnValue setupQPdataFromFile(	const char* const H_file, 	/**< Name of file where Hessian matrix, of neighbouring QP to be solved, is stored. \n
																     		 If Hessian matrix is trivial,a NULL pointer can be passed. */
											const char* const g_file, 	/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
											const char* const A_file,	/**< Name of file where constraint matrix, of neighbouring QP to be solved, is stored. */
											const char* const lb_file, 	/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no lower bounds exist, a NULL pointer can be passed. */
											const char* const ub_file, 	/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no upper bounds exist, a NULL pointer can be passed. */
											const char* const lbA_file, /**< Name of file where lower constraints' bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
											const char* const ubA_file	/**< Name of file where upper constraints' bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no upper constraints' bounds exist, a NULL pointer can be passed. */
											);

		/** Loads new QP vectors from files (internal members are not affected!).
		 *	\return SUCCESSFUL_RETURN \n
					RET_UNABLE_TO_OPEN_FILE \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS */
		returnValue loadQPvectorsFromFile(	const char* const g_file, 	/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
											const char* const lb_file, 	/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no lower bounds exist, a NULL pointer can be passed. */
											const char* const ub_file, 	/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no upper bounds exist, a NULL pointer can be passed. */
											const char* const lbA_file, /**< Name of file where lower constraints' bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no lower constraints' bounds exist, a NULL pointer can be passed. */
											const char* const ubA_file, /**< Name of file where upper constraints' bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no upper constraints' bounds exist, a NULL pointer can be passed. */
											real_t* const g_new,		/**< Output: Gradient of neighbouring QP to be solved. */
											real_t* const lb_new,		/**< Output: Lower bounds of neighbouring QP to be solved */
											real_t* const ub_new,		/**< Output: Upper bounds of neighbouring QP to be solved */
											real_t* const lbA_new,		/**< Output: Lower constraints' bounds of neighbouring QP to be solved */
											real_t* const ubA_new		/**< Output: Upper constraints' bounds of neighbouring QP to be solved */
											) const;


		/** Prints concise information on the current iteration.
		 *	\return  SUCCESSFUL_RETURN \n */
		returnValue printIteration(	int_t iter,							/**< Number of current iteration. */
									int_t BC_idx, 						/**< Index of blocking constraint. */
									SubjectToStatus BC_status,			/**< Status of blocking constraint. */
									BooleanType BC_isBound,				/**< Indicates if blocking constraint is a bound. */
									real_t homotopyLength,				/**< Current homotopy distance. */
									BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
		 							);


		/** Sets constraint matrix of the QP. \n
			Note: Also internal vector Ax is recomputed!
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_INVALID_ARGUMENTS */
		inline returnValue setA(	Matrix *A_new	/**< New constraint matrix (a shallow copy is made). */
									);

		/** Sets dense constraint matrix of the QP. \n
			Note: Also internal vector Ax is recomputed!
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_INVALID_ARGUMENTS */
		inline returnValue setA(	const real_t* const A_new	/**< New dense constraint matrix (with correct dimension!), a shallow copy is made. */
									);


		/** Sets constraints' lower bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP */
		inline returnValue setLBA(	const real_t* const lbA_new	/**< New constraints' lower bound vector (with correct dimension!). */
									);

		/** Changes single entry of lower constraints' bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP \n
		 *			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setLBA(	int_t number,	/**< Number of entry to be changed. */
									real_t value	/**< New value for entry of lower constraints' bound vector (with correct dimension!). */
									);

		/** Sets constraints' upper bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP */
		inline returnValue setUBA(	const real_t* const ubA_new	/**< New constraints' upper bound vector (with correct dimension!). */
									);

		/** Changes single entry of upper constraints' bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP \n
		 *			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setUBA(	int_t number,	/**< Number of entry to be changed. */
									real_t value	/**< New value for entry of upper constraints' bound vector (with correct dimension!). */
									);


		/** Drops the blocking bound/constraint that led to infeasibility, or finds another
		 *  bound/constraint to drop according to drop priorities.
		 *  \return SUCCESSFUL_RETURN \n
		 */
		returnValue dropInfeasibles ( 	int_t BC_number,			/**< Number of the bound or constraint to be added. */
										SubjectToStatus BC_status, 	/**< New status of the bound or constraint to be added. */
										BooleanType BC_isBound,		/**< Whether a bound or a constraint is to be added. */
										real_t *xiB,				/**< (not yet documented) */
										real_t *xiC					/**< (not yet documented) */
										);

		/** Decides if lower bounds are smaller than upper bounds
		 *
		 * \return SUCCESSFUL_RETURN \n
		 * 		   RET_QP_INFEASIBLE */

				returnValue areBoundsConsistent(const real_t* const lb,  /**< Vector of lower bounds*/
												const real_t* const ub,  /**< Vector of upper bounds*/
												const real_t* const lbA, /**< Vector of lower constraints*/
												const real_t* const ubA  /**< Vector of upper constraints*/
												) const;


	public:
		/** ...
		 *	\return SUCCESSFUL_RETURN  \n
					RET_UNABLE_TO_OPEN_FILE */
		returnValue writeQpDataIntoMatFile(	const char* const filename	/**< Mat file name. */
											) const;

		/** ...
		 *	\return SUCCESSFUL_RETURN  \n
					RET_UNABLE_TO_OPEN_FILE */
		returnValue writeQpWorkspaceIntoMatFile(	const char* const filename	/**< Mat file name. */
													);



	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		BooleanType freeConstraintMatrix; 		/**< Flag indicating whether the constraint matrix needs to be de-allocated. */
		Matrix* A;								/**< Constraint matrix. */

		real_t* lbA;							/**< Lower constraints' bound vector. */
		real_t* ubA;							/**< Upper constraints' bound vector. */

		Constraints constraints;				/**< Data structure for problem's constraints. */

		real_t* T;								/**< Reverse triangular matrix, A = [0 T]*Q'. */
		real_t* Q;								/**< Orthonormal quadratic matrix, A = [0 T]*Q'. */
		int_t sizeT;							/**< Matrix T is stored in a (sizeT x sizeT) array. */

		real_t* Ax;								/**< Stores the current A*x \n
												 *	 (for increased efficiency only). */
		real_t* Ax_l;							/**< Stores the current distance to lower constraints' bounds A*x-lbA \n
												 *	 (for increased efficiency only). */
		real_t* Ax_u;							/**< Stores the current distance to lower constraints' bounds ubA-A*x \n
												 *	 (for increased efficiency only). */

		ConstraintProduct* constraintProduct;	/**< Pointer to user-defined constraint product function. */

		real_t* tempA;							/**< Temporary for determineStepDirection. */
		real_t* tempB;							/**< Temporary for determineStepDirection. */
		real_t* ZFR_delta_xFRz;					/**< Temporary for determineStepDirection. */
		real_t* delta_xFRy;						/**< Temporary for determineStepDirection. */
		real_t* delta_xFRz;						/**< Temporary for determineStepDirection. */
		real_t* delta_yAC_TMP;					/**< Temporary for determineStepDirection. */

		real_t* tempC;                          /**< Temporary for constraint types. */
};


END_NAMESPACE_QPOASES

#include <qpOASES/QProblem.ipp>

#endif	/* QPOASES_QPROBLEM_HPP */


/*
 *	end of file
 */
