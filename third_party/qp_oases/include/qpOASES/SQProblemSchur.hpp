/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
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
 *	\file include/qpOASES/SQProblemSchur.hpp
 *	\author Andreas Waechter and Dennis Janka, based on QProblem.hpp by Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2012-2017
 *
 *	Declaration of the SQProblemSchur class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming
 *	with varying matrices and uses a Schur Complement approach to solve
 *	the linear systems.
 */


#ifndef QPOASES_SQPROBLEMSCHUR_HPP
#define QPOASES_SQPROBLEMSCHUR_HPP


#include <qpOASES/SQProblem.hpp>
#include <qpOASES/SparseSolver.hpp>
#include <qpOASES/LapackBlasReplacement.hpp>


BEGIN_NAMESPACE_QPOASES


/**
 *	\brief Implements the online active set strategy for QPs with varying, sparse matrices.
 *
 *	A class for setting up and solving quadratic programs with varying,
 *	sparse QP matrices. Here, sparsity is exploited by means of a
 *	Schur complement approach to solve the linear systems.
 *
 *	\author Andreas Waechter, Dennis Janka
 *	\version 3.2
 *	\date 2012-2017
 */
class SQProblemSchur : public SQProblem
{
	/* allow SolutionAnalysis class to access private members */
	friend class SolutionAnalysis;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		SQProblemSchur( );

		/** Constructor which takes the QP dimension and Hessian type
		 *  information. If the Hessian is the zero (i.e. HST_ZERO) or the
		 *  identity matrix (i.e. HST_IDENTITY), respectively, no memory
		 *  is allocated for it and a NULL pointer can be passed for it
		 *  to the init() functions. */
		SQProblemSchur(	int_t _nV,	  							/**< Number of variables. */
						int_t _nC,		  						/**< Number of constraints. */
						HessianType _hessianType = HST_UNKNOWN,	/**< Type of Hessian matrix. */
						int_t maxSchurUpdates = 75				/**< Maximal number of Schur updates */
						);

		/** Copy constructor (deep copy). */
		SQProblemSchur(	const SQProblemSchur& rhs		/**< Rhs object. */
						);

		/** Destructor. */
		virtual ~SQProblemSchur( );

		/** Assignment operator (deep copy). */
		virtual SQProblemSchur& operator=(	const SQProblemSchur& rhs	/**< Rhs object. */
											);

		/** Clears all data structures of QProblem except for QP data.
		 *	\return SUCCESSFUL_RETURN \n
					RET_RESET_FAILED */
		virtual returnValue reset( );

		/** Resets Schur complement.  This sets up the KKT matrix for the
			current activities, copies the activities, etc. TODO: Return values */
		returnValue resetSchurComplement( BooleanType allowInertiaCorrection );

		/** Return the total number of sparse matrix factorizations performed so far. */
		inline int_t getNumFactorizations( ) const;

	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );

		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const SQProblemSchur& rhs	/**< Rhs object. */
							);

		/** Computes the Cholesky decomposition of the projected Hessian (i.e. R^T*R = Z^T*H*Z).
		 *  For the Schur complement version, this function only returns SUCCESSFUL_RETURN. */
		virtual returnValue computeProjectedCholesky( );

		/** Computes initial Cholesky decomposition of the projected Hessian making
		 *  use of the function setupCholeskyDecomposition() or setupCholeskyDecompositionProjected().
		 *  For the Schur complement version, this function only returns SUCCESSFUL_RETURN. */
		virtual returnValue computeInitialCholesky( );

		/** Initialises TQ factorisation of A (i.e. A*Q = [0 T]) if NO constraint is active.
		 *  For the Schur complement version, this function only returns SUCCESSFUL_RETURN. */
		virtual returnValue setupTQfactorisation( );

		/** This method is overloaded from SQProblem.
		 *  Sets new matrices and calculates their factorisations. If the
		 *  current Hessian is trivial (i.e. HST_ZERO or HST_IDENTITY) but a
		 *  non-trivial one is given, memory for Hessian is allocated and
		 *  it is set to the given one. Afterwards, all QP vectors are
		 *  transformed in order to start from an optimal solution.
		 *	\return SUCCESSFUL_RETURN \n
		 *			RET_MATRIX_FACTORISATION_FAILED \n
		 * 			RET_NO_HESSIAN_SPECIFIED */
		virtual returnValue setupAuxiliaryQP(	SymmetricMatrix *H_new,		/**< New Hessian matrix. \n
																				 If Hessian matrix is trivial, a NULL pointer can be passed. */
												Matrix *A_new,				/**< New constraint matrix. \n
																				 If QP sequence does not involve constraints, a NULL pointer can be passed. */
												const real_t *lb_new,
												const real_t *ub_new,
												const real_t *lbA_new,
												const real_t *ubA_new
												);

		/** Setup bounds and constraints data structures according to auxiliaryBounds/Constraints.
		 *  Calls the sparse solver to obtain the factorization for the initial active set.
		 *	\return SUCCESSFUL_RETURN \n
					RET_SETUP_WORKINGSET_FAILED \n
					RET_INVALID_ARGUMENTS \n
					RET_UNKNOWN_BUG */
		virtual returnValue setupAuxiliaryWorkingSet(	const Bounds* const auxiliaryBounds,
														const Constraints* const auxiliaryConstraints,
														BooleanType setupAfresh
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

		/** Solves the system Ta = b or T^Ta = b where T is a reverse upper triangular matrix.
		 *	 This must not be called for the Schur complement version. */
		virtual returnValue backsolveT( 	const real_t* const b,	/**< Right hand side vector. */
											BooleanType transposed,	/**< Indicates if the transposed system shall be solved. */
											real_t* const a 		/**< Output: Solution vector */
											) const;

		/** Solves the system Ra = b or R^Ta = b where R is an upper triangular matrix.
		 *	This must not be called for the Schur complement version. */
		virtual returnValue backsolveR( 	const real_t* const b,	/**< Right hand side vector. */
											BooleanType transposed,	/**< Indicates if the transposed system shall be solved. */
											real_t* const a 		/**< Output: Solution vector */
											) const;

		/** Solves the system Ra = b or R^Ta = b where R is an upper triangular matrix. \n
		 *	This must not be called for the Schur complement version. */
		virtual returnValue backsolveR( 	const real_t* const b,		/**< Right hand side vector. */
											BooleanType transposed,		/**< Indicates if the transposed system shall be solved. */
											BooleanType removingBound,	/**< Indicates if function is called from "removeBound()". */
											real_t* const a 			/**< Output: Solution vector */
											) const;


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

		virtual returnValue determineStepDirection2(	const real_t* const delta_g,	/**< Step direction of gradient vector. */
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

	/*
	 *	PRIVATE MEMBER FUNCTION
	 */
	private:
		/** Checks if new active bound to be added is linearly dependent from
		 *	from row of the active constraints matrix.  This version computes
		 *	the multipliers in the (full) test.
		 *	\return	 RET_LINEARLY_DEPENDENT \n
		 			 RET_LINEARLY_INDEPENDENT */
		returnValue addBound_checkLISchur(	int_t number,			/**< Number of bound to be added to active set. */
											real_t* const xiC, 		/**< Output: Multipliers in linear independence test for active constraints. */
											real_t* const xiX 		/**< Output: Multipliers in linear independence test for fixed variables. */
											);

		/** Checks if new active bound to be added is linearly dependent from
		 *	from row of the active constraints matrix.  This version computes
		 *	the multipliers in the (full) test.
		 *	\return	 RET_LINEARLY_DEPENDENT \n
		 			 RET_LINEARLY_INDEPENDENT */
		returnValue addConstraint_checkLISchur(	int_t number,		/**< Number of bound to be added to active set. */
													real_t* const xiC, 	/**< Output: Multipliers in linear independence test for active constraints. */
													real_t* const xiX 	/**< Output: Multipliers in linear independence test for fixed variables. */
													);

		/** Compute product of "M" matrix (additional columns in KKT
			matrix) with vector.  y = alpha * M * x + beta * y */
		returnValue computeMTimes( real_t alpha, const real_t* const x, real_t beta, real_t* const y );

		/** Compute product of transpose of "M" matrix (additional columns in KKT
			matrix) with vector.  y = alpha * M^T * x + beta * y */
		returnValue computeMTransTimes( real_t alpha, const real_t* const x, real_t beta, real_t* const y );

		/** Add a row/column to the Schur complement. */
		returnValue addToSchurComplement( int_t number, SchurUpdateType update, int_t numNonzerosM, const sparse_int_t* M_pos, const real_t* const M_vals, int_t numNonzerosN, const sparse_int_t* Npos, const real_t* const Nvals, real_t N_diag );

		/** Remove a row/column from the Schur complement. */
		returnValue deleteFromSchurComplement( int_t idx, BooleanType allowUndo = BT_FALSE );

		/** Undo the last deletion from the Schur complement by moving the nS+1th row/column to position idx. */
		returnValue undoDeleteFromSchurComplement( int_t idx );

		/** Compute determinant of new nS*nS Schur complement from old factorization */
		real_t calcDetSchur( int_t idxDel );

		/** Update QR factorization and determinant of Schur complement after a row and column have been added or removed */
		returnValue updateSchurQR( int_t idxDel );

		/** Compute the solution to QRx = rhs and store it in sol */
		returnValue backsolveSchurQR( int_t dimS, const real_t* const rhs, int_t dimRhs, real_t* const sol );

		/** If negative curvature is discovered in the reduced Hessian, add bounds until all eigenvalues are positive */
		returnValue correctInertia();

		/** If the KKT matrix is declared singular during refactorization, remove linearly dependent constraints or add bounds */
		returnValue repairSingularWorkingSet( );

		returnValue stepCalcRhs(	int_t nFR, int_t nFX, int_t nAC, int_t* FR_idx, int_t* FX_idx, int_t* AC_idx, real_t& rhs_max, const real_t* const delta_g,
									const real_t* const delta_lbA, const real_t* const delta_ubA,
									const real_t* const delta_lb, const real_t* const delta_ub,
									BooleanType Delta_bC_isZero, BooleanType Delta_bB_isZero,
									real_t* const delta_xFX, real_t* const delta_xFR,
									real_t* const delta_yAC, real_t* const delta_yFX
									);

		returnValue stepCalcReorder(int_t nFR, int_t nAC, int_t* FR_idx, int_t* AC_idx, int_t nFRStart, int_t nACStart,
									int_t* FR_idxStart, int_t* AC_idxStart, int_t* FR_iSort, int_t* FR_iSortStart,
									int_t* AC_iSort, int_t* AC_iSortStart, real_t* rhs
									);

		returnValue stepCalcBacksolveSchur( int_t nFR, int_t nFX, int_t nAC, int_t* FR_idx, int_t* FX_idx, int_t* AC_idx,
											int_t dim, real_t* rhs, real_t* sol
											);

		returnValue stepCalcReorder2(	int_t nFR, int_t nAC, int_t* FR_idx, int_t* AC_idx, int_t nFRStart, int_t nACStart,
										int_t* FR_idxStart, int_t* AC_idxStart, int_t* FR_iSort, int_t* FR_iSortStart,
										int_t* AC_iSort, int_t* AC_iSortStart, real_t* sol, real_t* const delta_xFR, real_t* const delta_yAC
										);

		returnValue stepCalcResid(	int_t nFR, int_t nFX, int_t nAC, int_t* FR_idx, int_t* FX_idx, int_t* AC_idx,
									BooleanType Delta_bC_isZero, real_t* const delta_xFX, real_t* const delta_xFR,
									real_t* const delta_yAC, const real_t* const delta_g,
									const real_t* const delta_lbA, const real_t* const delta_ubA, real_t& rnrm
									);

		returnValue stepCalcDeltayFx(	int_t nFR, int_t nFX, int_t nAC, int_t* FX_idx, const real_t* const delta_g,
										real_t* const delta_xFX, real_t* const delta_xFR, real_t* const delta_yAC, real_t* const delta_yFX
										);

	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		SparseSolver* sparseSolver;			/**< Interface to the sparse linear solver. */

		real_t* S;							/**< Schur complement matrix. (This is actually the negative of the Schur complement!) */
		int_t nS;							/**< Current size of Schur complement matrix. -1 means that the Schur complement has not yet been initialized. */
		int_t nSmax;						/**< Maximum size of Schur complement matrix. */

		real_t* Q_;							/**< QR factorization of S: orthogonal matrix Q */
		real_t* R_;							/**< QR factorization of S: upper triangular matrix R */
		real_t detS;						/**< Determinant of Schur complement */
		real_t rcondS;						/**< Reciprocal of condition number of S (estimate) */
		int_t numFactorizations;			/**< Total number of factorizations performed */

		int_t* schurUpdateIndex;			/**< Indices of variables or constraints for each update in Schur complement. */
		SchurUpdateType* schurUpdate;		/**< Type of update for each update in Schur complement. */

		int_t M_physicallength;				/**< Allocated size of the M_vals and M_ir arrays. */
		real_t* M_vals;						/**< Values of the sparse M matrix containing the vectors with the additional rows defining the Schur complement (length). */
		sparse_int_t* M_ir;					/**< Row indices (length). */
		sparse_int_t* M_jc;					/**< Indices in M to first entry of columns (nS+1). */

		Indexlist boundsFreeStart;			/**< Index list for free bounds when major iteration started. */
		Indexlist constraintsActiveStart;	/**< Index list for active constraints when major iteration started. */
};


END_NAMESPACE_QPOASES

#include <qpOASES/SQProblemSchur.ipp>

#endif	/* QPOASES_QPROBLEMSCHUR_HPP */


/*
 *	end of file
 */
