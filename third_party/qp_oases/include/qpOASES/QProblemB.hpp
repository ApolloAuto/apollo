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
 *	\file include/qpOASES/QProblemB.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the QProblemB class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming
 *	for problems with (simple) bounds only.
 */



#ifndef QPOASES_QPROBLEMB_HPP
#define QPOASES_QPROBLEMB_HPP


#include <qpOASES/Flipper.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/Matrices.hpp>


BEGIN_NAMESPACE_QPOASES


class SolutionAnalysis;

/**
 *	\brief Implements the online active set strategy for box-constrained QPs.
 *
 *	Class for setting up and solving quadratic programs with bounds (= box constraints) only.
 *	The main feature is the possibily to use the newly developed online active set strategy
 *	for parametric quadratic programming.
 *
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 */
class QProblemB
{
	/* allow SolutionAnalysis class to access private members */
	friend class SolutionAnalysis;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		QProblemB( );

		/** Constructor which takes the QP dimension and Hessian type
		 *  information. If the Hessian is the zero (i.e. HST_ZERO) or the
		 *  identity matrix (i.e. HST_IDENTITY), respectively, no memory
		 *  is allocated for it and a NULL pointer can be passed for it
		 *  to the init() functions. */
		QProblemB(	int_t _nV,								/**< Number of variables. */
					HessianType _hessianType = HST_UNKNOWN,	/**< Type of Hessian matrix. */
					BooleanType allocDenseMats = BT_TRUE	/**< Enable allocation of dense matrices. */
					);

		/** Copy constructor (deep copy). */
		QProblemB(	const QProblemB& rhs	/**< Rhs object. */
					);

		/** Destructor. */
		virtual ~QProblemB( );

		/** Assignment operator (deep copy). */
		virtual QProblemB& operator=(	const QProblemB& rhs	/**< Rhs object. */
								);


		/** Clears all data structures of QProblemB except for QP data.
		 *	\return SUCCESSFUL_RETURN \n
					RET_RESET_FAILED */
		virtual returnValue reset( );


		/** Initialises a simply bounded QP problem with given QP data and tries to solve it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0 : starts with xOpt = 0, yOpt = 0 and gB empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0 : starts with xOpt, yOpt = 0 and obtain gB by "clipping", \n
		 *	3. 0,    yOpt, 0 : starts with xOpt = 0, yOpt and obtain gB from yOpt != 0, \n
		 *	4. 0,    0,    gB: starts with xOpt = 0, yOpt = 0 and gB, \n
		 *	5. xOpt, yOpt, 0 : starts with xOpt, yOpt and obtain gB from yOpt != 0, \n
		 *	6. xOpt, 0,    gB: starts with xOpt, yOpt = 0 and gB, \n
		 *	7. xOpt, yOpt, gB: starts with xOpt, yOpt and gB (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_INVALID_ARGUMENTS */
		returnValue init(	SymmetricMatrix *_H,					/**< Hessian matrix (a shallow copy is made). */
							const real_t* const _g,					/**< Gradient vector. */
							const real_t* const _lb,				/**< Lower bounds (on variables). \n
																		 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const _ub,				/**< Upper bounds (on variables). \n
																		 If no upper bounds exist, a NULL pointer can be passed. */
							int_t& nWSR, 							/**< Input: Maximum number of working set recalculations when using initial homotopy. \n
																		 Output: Number of performed working set recalculations. */
				 			real_t* const cputime = 0,				/**< Input: Maximum CPU time allowed for QP initialisation. \n
																		 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,			/**< Optimal primal solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,			/**< Optimal dual solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,	/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																		 (If a null pointer is passed, all bounds are assumed inactive!) */
							const real_t* const _R = 0				/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix.
																	 	 The Cholesky factor must be stored in a real_t array of size nV*nV
																		 in row-major format. Note: Only used if xOpt/yOpt and gB are NULL! \n
																		 (If a null pointer is passed, Cholesky decomposition is computed internally!) */
							);

		/** Initialises a simply bounded QP problem with given QP data and tries to solve it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0 : starts with xOpt = 0, yOpt = 0 and gB empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0 : starts with xOpt, yOpt = 0 and obtain gB by "clipping", \n
		 *	3. 0,    yOpt, 0 : starts with xOpt = 0, yOpt and obtain gB from yOpt != 0, \n
		 *	4. 0,    0,    gB: starts with xOpt = 0, yOpt = 0 and gB, \n
		 *	5. xOpt, yOpt, 0 : starts with xOpt, yOpt and obtain gB from yOpt != 0, \n
		 *	6. xOpt, 0,    gB: starts with xOpt, yOpt = 0 and gB, \n
		 *	7. xOpt, yOpt, gB: starts with xOpt, yOpt and gB (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_INVALID_ARGUMENTS */
		returnValue init(	const real_t* const _H, 				/**< Hessian matrix (a shallow copy is made). \n
																		 If Hessian matrix is trivial, a NULL pointer can be passed. */
							const real_t* const _g,					/**< Gradient vector. */
							const real_t* const _lb,				/**< Lower bounds (on variables). \n
																		 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const _ub,				/**< Upper bounds (on variables). \n
																		 If no upper bounds exist, a NULL pointer can be passed. */
							int_t& nWSR, 							/**< Input: Maximum number of working set recalculations when using initial homotopy. \n
																		 Output: Number of performed working set recalculations. */
				 			real_t* const cputime = 0,				/**< Input: Maximum CPU time allowed for QP initialisation. \n
																		 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,			/**< Optimal primal solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,			/**< Optimal dual solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,	/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																		 (If a null pointer is passed, all bounds are assumed inactive!) */
							const real_t* const _R = 0				/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix.
																	 	 The Cholesky factor must be stored in a real_t array of size nV*nV
																		 in row-major format. Note: Only used if xOpt/yOpt and gB are NULL! \n
																		 (If a null pointer is passed, Cholesky decomposition is computed internally!) */
							);

		/** Initialises a simply bounded QP problem with given QP data to be read from files and solves it
		 *	using at most nWSR iterations. Depending on the parameter constellation it: \n
		 *	1. 0,    0,    0 : starts with xOpt = 0, yOpt = 0 and gB empty (or all implicit equality bounds), \n
		 *	2. xOpt, 0,    0 : starts with xOpt, yOpt = 0 and obtain gB by "clipping", \n
		 *	3. 0,    yOpt, 0 : starts with xOpt = 0, yOpt and obtain gB from yOpt != 0, \n
		 *	4. 0,    0,    gB: starts with xOpt = 0, yOpt = 0 and gB, \n
		 *	5. xOpt, yOpt, 0 : starts with xOpt, yOpt and obtain gB from yOpt != 0, \n
		 *	6. xOpt, 0,    gB: starts with xOpt, yOpt = 0 and gB, \n
		 *	7. xOpt, yOpt, gB: starts with xOpt, yOpt and gB (assume them to be consistent!)
		 *
		 *  Note: This function internally calls solveInitialQP for initialisation!
		 *
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED \n
					RET_UNABLE_TO_READ_FILE */
		returnValue init(	const char* const H_file,				/**< Name of file where Hessian matrix is stored. \n
																		 If Hessian matrix is trivial, a NULL pointer can be passed. */
							const char* const g_file,				/**< Name of file where gradient vector is stored. */
							const char* const lb_file,				/**< Name of file where lower bound vector. \n
																		 If no lower bounds exist, a NULL pointer can be passed. */
							const char* const ub_file,				/**< Name of file where upper bound vector. \n
																		 If no upper bounds exist, a NULL pointer can be passed. */
							int_t& nWSR, 							/**< Input: Maximum number of working set recalculations when using initial homotopy. \n
																		 Output: Number of performed working set recalculations. */
				 			real_t* const cputime = 0,				/**< Input: Maximum CPU time allowed for QP initialisation. \n
																		 Output: CPU time spent for QP initialisation (if pointer passed). */
							const real_t* const xOpt = 0,			/**< Optimal primal solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old primal solution is kept!) */
							const real_t* const yOpt = 0,			/**< Optimal dual solution vector. A NULL pointer can be passed. \n
																		 (If a null pointer is passed, the old dual solution is kept!) */
							const Bounds* const guessedBounds = 0,	/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																		 (If a null pointer is passed, all bounds are assumed inactive!) */
							const char* const R_file = 0			/**< Name of the file where a pre-computed (upper triangular) Cholesky factor
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
					RET_HOTSTART_STOPPED_UNBOUNDEDNESS \n
					RET_SETUP_AUXILIARYQP_FAILED */
		returnValue hotstart(	const real_t* const g_new,				/**< Gradient of neighbouring QP to be solved. */
								const real_t* const lb_new,				/**< Lower bounds of neighbouring QP to be solved. \n
													 						 If no lower bounds exist, a NULL pointer can be passed. */
								const real_t* const ub_new,				/**< Upper bounds of neighbouring QP to be solved. \n
													 						 If no upper bounds exist, a NULL pointer can be passed. */
								int_t& nWSR,							/**< Input: Maximum number of working set recalculations; \n
																			 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,				/**< Input: Maximum CPU time allowed for QP solution. \n
																			 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0	/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																			 (If a null pointer is passed, the previous working set is kept!) */
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
					RET_SETUP_AUXILIARYQP_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue hotstart(	const char* const g_file, 				/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
								const char* const lb_file,				/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
													 						 If no lower bounds exist, a NULL pointer can be passed. */
								const char* const ub_file,				/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
													 						 If no upper bounds exist, a NULL pointer can be passed. */
								int_t& nWSR, 							/**< Input: Maximum number of working set recalculations; \n
																			 Output: Number of performed working set recalculations. */
								real_t* const cputime = 0,			 	/**< Input: Maximum CPU time allowed for QP solution. \n
																			 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								const Bounds* const guessedBounds = 0	/**< Optimal working set of bounds for solution (xOpt,yOpt). \n
																			 (If a null pointer is passed, the previous working set is kept!) */
								);


		/** Writes a vector with the state of the working set
		 *	\return SUCCESSFUL_RETURN \n
		 *	        RET_INVALID_ARGUMENTS */
		virtual returnValue getWorkingSet(	real_t* workingSet				/** Output: array containing state of the working set. */
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


		/** Returns current bounds object of the QP (deep copy).
		  *	\return SUCCESSFUL_RETURN \n
		  			RET_QPOBJECT_NOT_SETUP */
		inline returnValue getBounds(	Bounds& _bounds	/** Output: Bounds object. */
										) const;


		/** Returns the number of variables.
		 *	\return Number of variables. */
		inline int_t getNV( ) const;

		/** Returns the number of free variables.
		 *	\return Number of free variables. */
		inline int_t getNFR( ) const;

		/** Returns the number of fixed variables.
		 *	\return Number of fixed variables. */
		inline int_t getNFX( ) const;

		/** Returns the number of implicitly fixed variables.
		 *	\return Number of implicitly fixed variables. */
		inline int_t getNFV( ) const;

		/** Returns the dimension of null space.
		 *	\return Dimension of null space. */
		virtual int_t getNZ( ) const;


		/** Returns the optimal objective function value.
		 *	\return finite value: Optimal objective function value (QP was solved) \n
		 			+infinity:	  QP was not yet solved */
		real_t getObjVal( ) const;

		/** Returns the objective function value at an arbitrary point x.
		 *	\return Objective function value at point x */
		real_t getObjVal(	const real_t* const _x	/**< Point at which the objective function shall be evaluated. */
							) const;

		/** Returns the primal solution vector.
		 *	\return SUCCESSFUL_RETURN \n
					RET_QP_NOT_SOLVED */
		returnValue getPrimalSolution(	real_t* const xOpt			/**< Output: Primal solution vector (if QP has been solved). */
										) const;

		/** Returns the dual solution vector.
		 *	\return SUCCESSFUL_RETURN \n
					RET_QP_NOT_SOLVED */
		virtual returnValue getDualSolution(	real_t* const yOpt	/**< Output: Dual solution vector (if QP has been solved). */
												) const;


		/** Returns status of the solution process.
		 *	\return Status of solution process. */
		inline QProblemStatus getStatus( ) const;


		/** Returns if the QProblem object is initialised.
		 *	\return BT_TRUE:  QProblemB initialised \n
		 			BT_FALSE: QProblemB not initialised */
		inline BooleanType isInitialised( ) const;

		/** Returns if the QP has been solved.
		 *	\return BT_TRUE:  QProblemB solved \n
		 			BT_FALSE: QProblemB not solved */
		inline BooleanType isSolved( ) const;

		/** Returns if the QP is infeasible.
		 *	\return BT_TRUE:  QP infeasible \n
		 			BT_FALSE: QP feasible (or not known to be infeasible!) */
		inline BooleanType isInfeasible( ) const;

		/** Returns if the QP is unbounded.
		 *	\return BT_TRUE:  QP unbounded \n
		 			BT_FALSE: QP unbounded (or not known to be unbounded!) */
		inline BooleanType isUnbounded( ) const;


		/** Returns Hessian type flag (type is not determined due to this call!).
		 *	\return Hessian type. */
		inline HessianType getHessianType( ) const;

		/** Changes the print level.
 		 *	\return SUCCESSFUL_RETURN */
		inline returnValue setHessianType(	HessianType _hessianType /**< New Hessian type. */
											);

		/** Returns if the QP has been internally regularised.
		 *	\return BT_TRUE:  Hessian is internally regularised for QP solution \n
		 			BT_FALSE: No internal Hessian regularisation is used for QP solution */
		inline BooleanType usingRegularisation( ) const;

		/** Returns current options struct.
		 *	\return Current options struct. */
		inline Options getOptions( ) const;

		/** Overrides current options with given ones.
 		 *	\return SUCCESSFUL_RETURN */
		inline returnValue setOptions(	const Options& _options	/**< New options. */
										);

		/** Returns the print level.
		 *	\return Print level. */
		inline PrintLevel getPrintLevel( ) const;

		/** Changes the print level.
 		 *	\return SUCCESSFUL_RETURN */
		returnValue setPrintLevel(	PrintLevel _printlevel	/**< New print level. */
									);


		/** Returns the current number of QP problems solved.
		 *	\return Number of QP problems solved. */
		inline uint_t getCount( ) const;

		/** Resets QP problem counter (to zero).
		 *	\return SUCCESSFUL_RETURN. */
		inline returnValue resetCounter( );


		/** Prints concise list of properties of the current QP.
		 *	\return  SUCCESSFUL_RETURN \n */
		virtual returnValue printProperties( );

		/** Prints a list of all options and their current values.
		 *	\return  SUCCESSFUL_RETURN \n */
		returnValue printOptions( ) const;


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );

		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const QProblemB& rhs	/**< Rhs object. */
							);

		/** If Hessian type has been set by the user, nothing is done.
		 *  Otherwise the Hessian type is set to HST_IDENTITY, HST_ZERO, or
		 *  HST_POSDEF (default), respectively.
		 *	\return SUCCESSFUL_RETURN \n
					RET_HESSIAN_INDEFINITE */
		returnValue determineHessianType( );

		/** Determines type of existing constraints and bounds (i.e. implicitly fixed, unbounded etc.).
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUPSUBJECTTOTYPE_FAILED */
		virtual returnValue setupSubjectToType( );

		/** Determines type of new constraints and bounds (i.e. implicitly fixed, unbounded etc.).
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUPSUBJECTTOTYPE_FAILED */
		virtual returnValue setupSubjectToType(	const real_t* const lb_new,	/**< New lower bounds. */
												const real_t* const ub_new	/**< New upper bounds. */
												);

		/** Computes the Cholesky decomposition of the (simply projected) Hessian
		 *  (i.e. R^T*R = Z^T*H*Z). It only works in the case where Z is a simple
		 *  projection matrix!
		 *  Note: If Hessian turns out not to be positive definite, the Hessian type
		 *		  is set to HST_SEMIDEF accordingly.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_HESSIAN_NOT_SPD \n
		 *			RET_INDEXLIST_CORRUPTED */
		virtual returnValue computeCholesky( );


		/** Computes initial Cholesky decomposition of the (simply projected) Hessian
		 *  making use of the function computeCholesky().
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_HESSIAN_NOT_SPD \n
		 *			RET_INDEXLIST_CORRUPTED */
		virtual returnValue setupInitialCholesky( );

		/** Obtains the desired working set for the auxiliary initial QP in
		 *  accordance with the user specifications
		 *	\return SUCCESSFUL_RETURN \n
					RET_OBTAINING_WORKINGSET_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue obtainAuxiliaryWorkingSet(	const real_t* const xOpt,			/**< Optimal primal solution vector.
																					 *	 If a NULL pointer is passed, all entries are assumed to be zero. */
												const real_t* const yOpt,			/**< Optimal dual solution vector.
																					 *	 If a NULL pointer is passed, all entries are assumed to be zero. */
												const Bounds* const guessedBounds,	/**< Guessed working set for solution (xOpt,yOpt). */
												Bounds* auxiliaryBounds				/**< Input: Allocated bound object. \n
																					 *	 Output: Working set for auxiliary QP. */
												) const;

		/** Decides if lower bounds are smaller than upper bounds
		 *
		 * \return SUCCESSFUL_RETURN \n
		 * 		   RET_QP_INFEASIBLE */

		returnValue areBoundsConsistent(const real_t* const lb, /**< Vector of lower bounds*/
										const real_t* const ub  /**< Vector of upper bounds*/
										) const;

		/** Solves the system Ra = b or R^Ta = b where R is an upper triangular matrix.
		 *	\return SUCCESSFUL_RETURN \n
					RET_DIV_BY_ZERO */
		virtual returnValue backsolveR(	const real_t* const b,	/**< Right hand side vector. */
								BooleanType transposed,	/**< Indicates if the transposed system shall be solved. */
								real_t* const a 		/**< Output: Solution vector */
								) const;

		/** Solves the system Ra = b or R^Ta = b where R is an upper triangular matrix. \n
		 *  Special variant for the case that this function is called from within "removeBound()".
		 *	\return SUCCESSFUL_RETURN \n
					RET_DIV_BY_ZERO */
		virtual returnValue backsolveR(	const real_t* const b,		/**< Right hand side vector. */
								BooleanType transposed,		/**< Indicates if the transposed system shall be solved. */
								BooleanType removingBound,	/**< Indicates if function is called from "removeBound()". */
								real_t* const a 			/**< Output: Solution vector */
								) const;


		/** Determines step direction of the shift of the QP data.
		 *	\return SUCCESSFUL_RETURN */
		returnValue determineDataShift(	const real_t* const g_new,	/**< New gradient vector. */
										const real_t* const lb_new,	/**< New lower bounds. */
										const real_t* const ub_new,	/**< New upper bounds. */
										real_t* const delta_g,	 	/**< Output: Step direction of gradient vector. */
										real_t* const delta_lb,	 	/**< Output: Step direction of lower bounds. */
										real_t* const delta_ub,	 	/**< Output: Step direction of upper bounds. */
										BooleanType& Delta_bB_isZero/**< Output: Indicates if active bounds are to be shifted. */
										);


		/** Sets up internal QP data.
		 *	\return SUCCESSFUL_RETURN \n
					RET_INVALID_ARGUMENTS */
		returnValue setupQPdata(	SymmetricMatrix *_H,	 	/**< Hessian matrix.*/
									const real_t* const _g,		/**< Gradient vector. */
									const real_t* const _lb,	/**< Lower bounds (on variables). \n
																	 If no lower bounds exist, a NULL pointer can be passed. */
									const real_t* const _ub		/**< Upper bounds (on variables). \n
																	 If no upper bounds exist, a NULL pointer can be passed. */
									);

		/** Sets up internal QP data. If the current Hessian is trivial
		 *  (i.e. HST_ZERO or HST_IDENTITY) but a non-trivial one is given,
		 *  memory for Hessian is allocated and it is set to the given one.
		 *	\return SUCCESSFUL_RETURN \n
					RET_INVALID_ARGUMENTS \n
					RET_NO_HESSIAN_SPECIFIED */
		returnValue setupQPdata(	const real_t* const _H, 	/**< Hessian matrix. \n
																     If Hessian matrix is trivial,a NULL pointer can be passed. */
									const real_t* const _g,		/**< Gradient vector. */
									const real_t* const _lb,	/**< Lower bounds (on variables). \n
																	 If no lower bounds exist, a NULL pointer can be passed. */
									const real_t* const _ub		/**< Upper bounds (on variables). \n
																	 If no upper bounds exist, a NULL pointer can be passed. */
									);

		/** Sets up internal QP data by loading it from files. If the current Hessian
		 *  is trivial (i.e. HST_ZERO or HST_IDENTITY) but a non-trivial one is given,
		 *  memory for Hessian is allocated and it is set to the given one.
		 *	\return SUCCESSFUL_RETURN \n
					RET_UNABLE_TO_OPEN_FILE \n
					RET_UNABLE_TO_READ_FILE \n
					RET_INVALID_ARGUMENTS \n
					RET_NO_HESSIAN_SPECIFIED */
		returnValue setupQPdataFromFile(	const char* const H_file, 	/**< Name of file where Hessian matrix, of neighbouring QP to be solved, is stored. \n
																     		 If Hessian matrix is trivial,a NULL pointer can be passed. */
											const char* const g_file, 	/**< Name of file where gradient, of neighbouring QP to be solved, is stored. */
											const char* const lb_file, 	/**< Name of file where lower bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no lower bounds exist, a NULL pointer can be passed. */
											const char* const ub_file 	/**< Name of file where upper bounds, of neighbouring QP to be solved, is stored. \n
												 			 				 If no upper bounds exist, a NULL pointer can be passed. */
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
											real_t* const g_new,		/**< Output: Gradient of neighbouring QP to be solved. */
											real_t* const lb_new,		/**< Output: Lower bounds of neighbouring QP to be solved */
											real_t* const ub_new		/**< Output: Upper bounds of neighbouring QP to be solved */
											) const;


		/** Sets internal infeasibility flag and throws given error in case the far bound
		 *	strategy is not enabled (as QP might actually not be infeasible in this case).
		 *	\return RET_HOTSTART_STOPPED_INFEASIBILITY \n
					RET_ENSURELI_FAILED_CYCLING \n
					RET_ENSURELI_FAILED_NOINDEX */
		returnValue setInfeasibilityFlag(	returnValue returnvalue,			/**< Returnvalue to be tunneled. */
											BooleanType doThrowError = BT_FALSE	/**< Flag forcing to throw an error. */
											);


		/** Determines if next QP iteration can be performed within given CPU time limit.
		 *	\return BT_TRUE: CPU time limit is exceeded, stop QP solution. \n
					BT_FALSE: Sufficient CPU time for next QP iteration. */
		BooleanType isCPUtimeLimitExceeded(	const real_t* const cputime,	/**< Maximum CPU time allowed for QP solution. */
											real_t starttime,				/**< Start time of current QP solution. */
											int_t nWSR						/**< Number of working set recalculations performed so far. */
											) const;


		/** Regularise Hessian matrix by adding a scaled identity matrix to it.
		 *	\return SUCCESSFUL_RETURN \n
					RET_HESSIAN_ALREADY_REGULARISED */
		returnValue regulariseHessian( );


		/** Sets Hessian matrix of the QP.
		 *	\return SUCCESSFUL_RETURN */
		inline returnValue setH(	SymmetricMatrix* H_new	/**< New Hessian matrix (a shallow copy is made). */
									);

		/** Sets dense Hessian matrix of the QP.
		 *  If a null pointer is passed and
		 *  a) hessianType is HST_IDENTITY, nothing is done,
		 *  b) hessianType is not HST_IDENTITY, Hessian matrix is set to zero.
		 *	\return SUCCESSFUL_RETURN */
		inline returnValue setH(	const real_t* const H_new	/**< New dense Hessian matrix (with correct dimension!), a shallow copy is made. */
									);

		/** Changes gradient vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_INVALID_ARGUMENTS */
		inline returnValue setG(	const real_t* const g_new	/**< New gradient vector (with correct dimension!). */
									);

		/** Changes lower bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP */
		inline returnValue setLB(	const real_t* const lb_new	/**< New lower bound vector (with correct dimension!). */
									);

		/** Changes single entry of lower bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP \n
		 *			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setLB(	int_t number,	/**< Number of entry to be changed. */
									real_t value	/**< New value for entry of lower bound vector. */
									);

		/** Changes upper bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP */
		inline returnValue setUB(	const real_t* const ub_new	/**< New upper bound vector (with correct dimension!). */
									);

		/** Changes single entry of upper bound vector of the QP.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QPOBJECT_NOT_SETUP \n
		 *			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setUB(	int_t number,	/**< Number of entry to be changed. */
									real_t value	/**< New value for entry of upper bound vector. */
									);


		/** Computes parameters for the Givens matrix G for which [x,y]*G = [z,0]
		 *	\return SUCCESSFUL_RETURN */
		inline void computeGivens(	real_t xold,	/**< Matrix entry to be normalised. */
									real_t yold,	/**< Matrix entry to be annihilated. */
									real_t& xnew,	/**< Output: Normalised matrix entry. */
									real_t& ynew,	/**< Output: Annihilated matrix entry. */
									real_t& c,		/**< Output: Cosine entry of Givens matrix. */
									real_t& s 		/**< Output: Sine entry of Givens matrix. */
									) const;

		/** Applies Givens matrix determined by c and s (cf. computeGivens).
		 *	\return SUCCESSFUL_RETURN */
		inline void applyGivens(	real_t c,		/**< Cosine entry of Givens matrix. */
									real_t s,		/**< Sine entry of Givens matrix. */
									real_t nu, 		/**< Further factor: s/(1+c). */
									real_t xold,	/**< Matrix entry to be transformed corresponding to
													 *	 the normalised entry of the original matrix. */
									real_t yold, 	/**< Matrix entry to be transformed corresponding to
													 *	 the annihilated entry of the original matrix. */
									real_t& xnew,	/**< Output: Transformed matrix entry corresponding to
													 *	 the normalised entry of the original matrix. */
									real_t& ynew	/**< Output: Transformed matrix entry corresponding to
													 *	 the annihilated entry of the original matrix. */
									) const;



		/** Compute relative length of homotopy in data space for termination
		 *  criterion.
		 *  \return Relative length in data space. */
		real_t getRelativeHomotopyLength(	const real_t* const g_new,		/**< Final gradient. */
											const real_t* const lb_new,		/**< Final lower variable bounds. */
											const real_t* const ub_new		/**< Final upper variable bounds. */
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
                                        real_t* const ub_new_far			/**< ... */
                                        ) const;


		/** Performs robustified ratio test yield the maximum possible step length
		 *  along the homotopy path.
		 *	\return  SUCCESSFUL_RETURN */
		returnValue performRatioTest(	int_t nIdx, 						/**< Number of ratios to be checked. */
										const int_t* const idxList, 		/**< Array containing the indices of all ratios to be checked. */
										const SubjectTo* const subjectTo,	/**< Bound/Constraint object corresponding to ratios to be checked. */
										const real_t* const num,	 		/**< Array containing all numerators for performing the ratio test. */
										const real_t* const den,		 	/**< Array containing all denominators for performing the ratio test. */
										real_t epsNum,						/**< Numerator tolerance. */
										real_t epsDen,						/**< Denominator tolerance. */
										real_t& t,							/**< Output: Maximum possible step length along the homotopy path. */
										int_t& BC_idx 						/**< Output: Index of blocking constraint. */
										) const;

		/** Checks whether given ratio is blocking, i.e. limits the maximum step length
		 *  along the homotopy path to a value lower than given one.
		 *	\return  SUCCESSFUL_RETURN */
		inline BooleanType isBlocking(	real_t num,					 		/**< Numerator for performing the ratio test. */
										real_t den,		 					/**< Denominator for performing the ratio test. */
										real_t epsNum,						/**< Numerator tolerance. */
										real_t epsDen,						/**< Denominator tolerance. */
										real_t& t							/**< Input: Current maximum step length along the homotopy path,
																			 *   Output: Updated maximum possible step length along the homotopy path. */
										) const;


		/** Creates a sparse diagonal (square-)matrix which is a given
		 *  multiple of the identity matrix.
		 *  \return Diagonal matrix \n
		 */
		SymSparseMat* createDiagSparseMat(	int_t n,				/**< Row/column dimension of matrix to be created. */
											real_t diagVal = 1.0	/**< Value of all diagonal entries. */
											);


	/*
	 *	PRIVATE MEMBER FUNCTIONS
	 */
	private:
		/** Solves a QProblemB whose QP data is assumed to be stored in the member variables.
		 *  A guess for its primal/dual optimal solution vectors and the corresponding
		 *  optimal working set can be provided.
		 *  Note: This function is internally called by all init functions!
		 *	\return SUCCESSFUL_RETURN \n
					RET_INIT_FAILED \n
					RET_INIT_FAILED_CHOLESKY \n
					RET_INIT_FAILED_HOTSTART \n
					RET_INIT_FAILED_INFEASIBILITY \n
					RET_INIT_FAILED_UNBOUNDEDNESS \n
					RET_MAX_NWSR_REACHED */
		returnValue solveInitialQP(	const real_t* const xOpt,			/**< Optimal primal solution vector.*/
									const real_t* const yOpt,			/**< Optimal dual solution vector. */
									const Bounds* const guessedBounds,	/**< Optimal working set of bounds for solution (xOpt,yOpt). */
									const real_t* const _R,				/**< Pre-computed (upper triangular) Cholesky factor of Hessian matrix. */
									int_t& nWSR, 						/**< Input: Maximum number of working set recalculations; \n
																 		 *	 Output: Number of performed working set recalculations. */
									real_t* const cputime				/**< Input: Maximum CPU time allowed for QP solution. \n
																 			 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
									);

		/** Solves an initialised QProblemB using online active set strategy.
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
								int_t& nWSR,						/**< Input: Maximum number of working set recalculations; \n
																		 Output: Number of performed working set recalculations. */
								real_t* const cputime,				/**< Input: Maximum CPU time allowed for QP solution. \n
																		 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
								int_t  nWSRperformed = 0,			/**< Number of working set recalculations already performed to solve
																		 this QP within previous solveQP() calls. This number is
																		 always zero, except for successive calls from solveRegularisedQP()
																		 or when using the far bound strategy. */
								BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
								);


		/** Solves an initialised QProblemB using online active set strategy.
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
										int_t& nWSR,						/**< Input: Maximum number of working set recalculations; \n
																				 Output: Number of performed working set recalculations. */
										real_t* const cputime,				/**< Input: Maximum CPU time allowed for QP solution. \n
																				 Output: CPU time spent for QP solution (or to perform nWSR iterations). */
										int_t nWSRperformed = 0,			/**< Number of working set recalculations already performed to solve
																				 this QP within previous solveRegularisedQP() calls. This number is
																				 always zero, except for successive calls when using the far bound strategy. */
										BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
										);


		/** Sets up bound data structure according to auxiliaryBounds.
		 *  (If the working set shall be setup afresh, make sure that
		 *  bounds data structure has been resetted!)
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUP_WORKINGSET_FAILED \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNOWN_BUG */
		returnValue setupAuxiliaryWorkingSet(	const Bounds* const auxiliaryBounds,	/**< Working set for auxiliary QP. */
												BooleanType setupAfresh					/**< Flag indicating if given working set shall be
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
		 *  (assumes that members X, Y and BOUNDS have already been initialised!).
		 *	\return SUCCESSFUL_RETURN */
		returnValue setupAuxiliaryQPgradient( );

		/** Sets up bounds of the auxiliary initial QP for given
		 *  optimal primal/dual solution and given initial working set
		 *  (assumes that members X, Y and BOUNDS have already been initialised!).
		 *	\return SUCCESSFUL_RETURN \n
					RET_UNKNOWN_BUG */
		returnValue setupAuxiliaryQPbounds( BooleanType useRelaxation	/**< Flag indicating if inactive bounds shall be relaxed. */
											);


	protected:
		/** Updates QP vectors, working sets and internal data structures in order to
			start from an optimal solution corresponding to initial guesses of the working
			set for bounds
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_SETUP_AUXILIARYQP_FAILED */
		virtual returnValue setupAuxiliaryQP(	const Bounds* const guessedBounds	/**< Initial guess for working set of bounds. */
												);

	private:
		/** Determines step direction of the homotopy path.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_STEPDIRECTION_FAILED_CHOLESKY */
		returnValue determineStepDirection(	const real_t* const delta_g,	/**< Step direction of gradient vector. */
											const real_t* const delta_lb,	/**< Step direction of lower bounds. */
											const real_t* const delta_ub,	/**< Step direction of upper bounds. */
											BooleanType Delta_bB_isZero,	/**< Indicates if active bounds are to be shifted. */
											real_t* const delta_xFX, 		/**< Output: Primal homotopy step direction of fixed variables. */
											real_t* const delta_xFR,	 	/**< Output: Primal homotopy step direction of free variables. */
											real_t* const delta_yFX 		/**< Output: Dual homotopy step direction of fixed variables' multiplier. */
											);

		/** Determines the maximum possible step length along the homotopy path
		 *  and performs this step (without changing working set).
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_QP_INFEASIBLE \n
		 */
		returnValue performStep(	const real_t* const delta_g,	/**< Step direction of gradient. */
									const real_t* const delta_lb,	/**< Step direction of lower bounds. */
									const real_t* const delta_ub,	/**< Step direction of upper bounds. */
									const real_t* const delta_xFX, 	/**< Primal homotopy step direction of fixed variables. */
									const real_t* const delta_xFR,	/**< Primal homotopy step direction of free variables. */
									const real_t* const delta_yFX,	/**< Dual homotopy step direction of fixed variables' multiplier. */
									int_t& BC_idx, 					/**< Output: Index of blocking constraint. */
									SubjectToStatus& BC_status		/**< Output: Status of blocking constraint. */
									);

		/** Updates active set.
		 *	\return  SUCCESSFUL_RETURN \n
		 			 RET_REMOVE_FROM_ACTIVESET_FAILED \n
					 RET_ADD_TO_ACTIVESET_FAILED */
		returnValue changeActiveSet(	int_t BC_idx, 						/**< Index of blocking constraint. */
										SubjectToStatus BC_status 			/**< Status of blocking constraint. */
										);

		/** Drift correction at end of each active set iteration
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue performDriftCorrection( );

		/** Determines if it is more efficient to refactorise the matrices when
		 *  hotstarting or not (i.e. better to update the existing factorisations).
		 *	\return BT_TRUE iff matrices shall be refactorised afresh
		 */
		BooleanType shallRefactorise(	const Bounds* const guessedBounds	/**< Guessed new working set. */
										) const;


		/** Adds a bound to active set (specialised version for the case where no constraints exist).
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_ADDBOUND_FAILED */
		returnValue addBound(	int_t number,				/**< Number of bound to be added to active set. */
								SubjectToStatus B_status,	/**< Status of new active bound. */
								BooleanType updateCholesky	/**< Flag indicating if Cholesky decomposition shall be updated. */
								);

		/** Removes a bounds from active set (specialised version for the case where no constraints exist).
		 *	\return SUCCESSFUL_RETURN \n
					RET_HESSIAN_NOT_SPD \n
					RET_REMOVEBOUND_FAILED */
		returnValue removeBound(	int_t number,				/**< Number of bound to be removed from active set. */
									BooleanType updateCholesky	/**< Flag indicating if Cholesky decomposition shall be updated. */
									);


		/** Prints concise information on the current iteration.
		 *	\return  SUCCESSFUL_RETURN \n */
		returnValue printIteration(	int_t iter,							/**< Number of current iteration. */
									int_t BC_idx, 						/**< Index of blocking bound. */
									SubjectToStatus BC_status,			/**< Status of blocking bound. */
									real_t homotopyLength,				/**< Current homotopy distance. */
									BooleanType isFirstCall = BT_TRUE	/**< Indicating whether this is the first call for current QP. */
									);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		BooleanType freeHessian;	/**< Flag indicating whether the Hessian matrix needs to be de-allocated. */
		SymmetricMatrix* H;			/**< Hessian matrix. */

		real_t* g;					/**< Gradient. */
		real_t* lb;					/**< Lower bound vector (on variables). */
		real_t* ub;					/**< Upper bound vector (on variables). */

		Bounds bounds;				/**< Data structure for problem's bounds. */

		real_t* R;					/**< Cholesky factor of H (i.e. H = R^T*R). */
		BooleanType haveCholesky;	/**< Flag indicating whether Cholesky decomposition has already been setup. */

		real_t* x;					/**< Primal solution vector. */
		real_t* y;					/**< Dual solution vector. */

		real_t tau;					/**< Last homotopy step length. */

		QProblemStatus status;		/**< Current status of the solution process. */

		BooleanType infeasible;		/**< QP infeasible? */
		BooleanType unbounded;		/**< QP unbounded? */

		HessianType hessianType;	/**< Type of Hessian matrix. */
		real_t regVal;				/**< Holds the offset used to regularise Hessian matrix (zero by default). */

		uint_t count;				/**< Counts the number of hotstart function calls. */

		real_t *delta_xFR_TMP;		/**< Temporary for determineStepDirection */

		real_t ramp0;				/**< Start value for Ramping Strategy. */
		real_t ramp1;				/**< Final value for Ramping Strategy. */
		int_t rampOffset;			/**< Offset index for Ramping. */

		Options options;			/**< Struct containing all user-defined options for solving QPs. */

		Flipper flipper;			/**< Struct for making a temporary copy of the matrix factorisations. */

		TabularOutput tabularOutput;	/**< Struct storing information for tabular output (printLevel == PL_TABULAR). */
};


END_NAMESPACE_QPOASES

#include <qpOASES/QProblemB.ipp>

#endif	/* QPOASES_QPROBLEMB_HPP */


/*
 *	end of file
 */
