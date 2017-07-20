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
 *	\file include/qpOASES/extras/SolutionAnalysis.hpp
 *	\author Hans Joachim Ferreau (thanks to Boris Houska)
 *	\version 3.2
 *	\date 2008-2017
 *
 *	Declaration of the SolutionAnalysis class designed to perform
 *	additional analysis after solving a QP with qpOASES.
 */


#ifndef QPOASES_SOLUTIONANALYSIS_HPP
#define QPOASES_SOLUTIONANALYSIS_HPP


#include <qpOASES/SQProblem.hpp>
#include <qpOASES/SQProblemSchur.hpp>


BEGIN_NAMESPACE_QPOASES


/**
 *	\brief Provides additional tools for analysing QP solutions.
 *
 *	This class is intended to provide additional tools for analysing
 *  a QP solution obtained with qpOASES.
 *
 *	\author Hans Joachim Ferreau (thanks to Boris Houska)
 *	\version 3.2
 *	\date 2008-2017
 */
class SolutionAnalysis
{
	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		SolutionAnalysis( );

		/** Copy constructor (deep copy). */
		SolutionAnalysis(	const SolutionAnalysis& rhs		/**< Rhs object. */
							);

		/** Destructor. */
		~SolutionAnalysis( );

		/** Assignment operator (deep copy). */
		SolutionAnalysis& operator=(	const SolutionAnalysis& rhs		/**< Rhs object. */
										);


		/** Computes the maximum violation of the KKT optimality conditions
		 *  of the current iterate within the QProblemB object.
		 *	\return Maximum violation of the KKT conditions (or INFTY on error). */
		real_t getKktViolation(	QProblemB* const qp,		/**< QProblemB to be analysed. */
								real_t* const maxStat = 0,	/**< Output: maximum value of stationarity condition residual. */
								real_t* const maxFeas = 0,	/**< Output: maximum value of primal feasibility violation. */
								real_t* const maxCmpl = 0	/**< Output: maximum value of complementarity residual. */
								) const;

		/** Computes the maximum violation of the KKT optimality conditions
		 *  of the current iterate within the QProblem object.
		 *	\return Maximum violation of the KKT conditions (or INFTY on error). */
		real_t getKktViolation(	QProblem* const qp,			/**< QProblem to be analysed. */
								real_t* const maxStat = 0,	/**< Output: maximum value of stationarity condition residual. */
								real_t* const maxFeas = 0,	/**< Output: maximum value of primal feasibility violation. */
								real_t* const maxCmpl = 0	/**< Output: maximum value of complementarity residual. */
								) const;

		/** Computes the maximum violation of the KKT optimality conditions
		 *  of the current iterate within the SQProblem object.
		 *	\return Maximum violation of the KKT conditions (or INFTY on error). */
		real_t getKktViolation(	SQProblem* const qp,		/**< SQProblem to be analysed. */
								real_t* const maxStat = 0,	/**< Output: maximum value of stationarity condition residual. */
								real_t* const maxFeas = 0,	/**< Output: maximum value of primal feasibility violation. */
								real_t* const maxCmpl = 0	/**< Output: maximum value of complementarity residual. */
								) const;


		/** Computes the variance-covariance matrix of the QP output for uncertain
			inputs.
		 *	\return SUCCESSFUL_RETURN \n
					RET_HOTSTART_FAILED \n
		 			RET_STEPDIRECTION_FAILED_TQ \n
					RET_STEPDIRECTION_FAILED_CHOLESKY */
		returnValue getVarianceCovariance(	QProblemB* const qp,			/**< QProblemB to be analysed. */
											const real_t* const g_b_bA_VAR,	/**< Input:  Variance-covariance of g, the bounds lb and ub, 
																			 *			 and lbA and ubA respectively. Dimension: 2nV x 2nV */
											real_t* const Primal_Dual_VAR	/**< Output: The result for the variance-covariance of the primal 
																			 *			 and dual variables. Dimension: 2nV x 2nV */
											) const;

		/** Computes the variance-covariance matrix of the QP output for uncertain
			inputs.
		 *	\return SUCCESSFUL_RETURN \n
					RET_HOTSTART_FAILED \n
		 			RET_STEPDIRECTION_FAILED_TQ \n
					RET_STEPDIRECTION_FAILED_CHOLESKY */
		returnValue getVarianceCovariance(	QProblem* const qp,				/**< QProblem to be analysed. */
											const real_t* const g_b_bA_VAR,	/**< Input:  Variance-covariance of g, the bounds lb and ub, 
																			 *			 and lbA and ubA respectively. Dimension:  (2nV+nC) x (2nV+nC) */
											real_t* const Primal_Dual_VAR	/**< Output: The result for the variance-covariance of the primal 
																			 *			 and dual variables. Dimension:  (2nV+nC) x (2nV+nC) */
											) const;

		/** Computes the variance-covariance matrix of the QP output for uncertain
			inputs.
		 *	\return SUCCESSFUL_RETURN \n
					RET_HOTSTART_FAILED \n
		 			RET_STEPDIRECTION_FAILED_TQ \n
					RET_STEPDIRECTION_FAILED_CHOLESKY */
		returnValue getVarianceCovariance(	SQProblem* const qp,			/**< SQProblem to be analysed. */
											const real_t* const g_b_bA_VAR,	/**< Input:  Variance-covariance of g, the bounds lb and ub, 
																			 *			 and lbA and ubA respectively. Dimension:  (2nV+nC) x (2nV+nC) */
											real_t* const Primal_Dual_VAR	/**< Output: The result for the variance-covariance of the primal 
																			 *			 and dual variables. Dimension:  (2nV+nC) x (2nV+nC) */
											) const;

		/** Checks if a direction of negative curvature shows up if we remove all bounds that just recently became active */
		returnValue checkCurvatureOnStronglyActiveConstraints(	SQProblemSchur* qp );
		returnValue checkCurvatureOnStronglyActiveConstraints(	SQProblem* qp );

	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:

};


END_NAMESPACE_QPOASES

#include <qpOASES/extras/SolutionAnalysis.ipp>

#endif	/* QPOASES_SOLUTIONANALYSIS_HPP */


/*
 *	end of file
 */
