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
 *	\file include/qpOASES/extras/OQPinterface.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of an interface comprising several utility functions
 *	for solving test problems from the Online QP Benchmark Collection
 *	(This collection is no longer maintained, see 
 *	http://www.qpOASES.org/onlineQP for a backup).
 */


#ifndef QPOASES_OQPINTERFACE_HPP
#define QPOASES_OQPINTERFACE_HPP


#include <qpOASES/Options.hpp>


BEGIN_NAMESPACE_QPOASES


/** Reads dimensions of an Online QP Benchmark problem from file.
 *
 * \return SUCCESSFUL_RETURN \n
		   RET_UNABLE_TO_READ_FILE \n
		   RET_FILEDATA_INCONSISTENT */
returnValue readOqpDimensions(	const char* path,	/**< Full path of the data files (without trailing slash!). */
								int_t& nQP,			/**< Output: Number of QPs. */
								int_t& nV,			/**< Output: Number of variables. */
								int_t& nC,			/**< Output: Number of constraints. */
								int_t& nEC			/**< Output: Number of equality constraints. */
								);

/** Reads data of an Online QP Benchmark problem from file.
 *  This function allocates the required memory for all data; after successfully calling it,
 *  you have to free this memory yourself!
 *
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS \n
		   RET_UNABLE_TO_READ_FILE \n
		   RET_FILEDATA_INCONSISTENT */
returnValue readOqpData(	const char* path,	/**< Full path of the data files (without trailing slash!). */
							int_t& nQP,			/**< Output: Number of QPs. */
							int_t& nV,			/**< Output: Number of variables. */
							int_t& nC,			/**< Output: Number of constraints. */
							int_t& nEC,			/**< Output: Number of equality constraints. */
							real_t** H,		 	/**< Output: Hessian matrix. */
							real_t** g,		 	/**< Output: Sequence of gradient vectors. */
							real_t** A,		 	/**< Output: Constraint matrix. */
							real_t** lb,		/**< Output: Sequence of lower bound vectors (on variables). */
							real_t** ub,		/**< Output: Sequence of upper bound vectors (on variables). */
							real_t** lbA,		/**< Output: Sequence of lower constraints' bound vectors. */
							real_t** ubA,		/**< Output: Sequence of upper constraints' bound vectors. */
							real_t** xOpt,		/**< Output: Sequence of primal solution vectors
												 *           (not read if a null pointer is passed). */
							real_t** yOpt,		/**< Output: Sequence of dual solution vectors
												 *           (not read if a null pointer is passed). */
							real_t** objOpt		/**< Output: Sequence of optimal objective function values
												 *           (not read if a null pointer is passed). */
							);


/** Solves an Online QP Benchmark problem as specified by the arguments.
 *  The maximum deviations from the given optimal solution as well as the
 *  maximum CPU time to solve each QP are determined.
 *
 *	Note: This variant is outdated and only kept to ensure 
 *		  backwards-compatibility!
 *
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOqpBenchmark(	int_t nQP,					/**< Number of QPs. */
								int_t nV,					/**< Number of variables. */
								int_t nC,					/**< Number of constraints. */
								int_t nEC,					/**< Number of equality constraints. */
								const real_t* const _H,		/**< Hessian matrix. */
								const real_t* const g,		/**< Sequence of gradient vectors. */
								const real_t* const _A,		/**< Constraint matrix. */
								const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
								const real_t* const lbA,	/**< Sequence of lower constraints' bound vectors. */
								const real_t* const ubA,	/**< Sequence of upper constraints' bound vectors. */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t& nWSR, 				/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);

/** Solves an Online QP Benchmark problem as specified by the arguments.
 *  The maximum deviations from the given optimal solution as well as the
 *  maximum CPU time to solve each QP are determined.
 *
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOqpBenchmark(	int_t nQP,					/**< Number of QPs. */
								int_t nV,					/**< Number of variables. */
								int_t nC,					/**< Number of constraints. */
								int_t nEC,					/**< Number of equality constraints. */
								const real_t* const _H,		/**< Hessian matrix. */
								const real_t* const g,		/**< Sequence of gradient vectors. */
								const real_t* const _A,		/**< Constraint matrix. */
								const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
								const real_t* const lbA,	/**< Sequence of lower constraints' bound vectors. */
								const real_t* const ubA,	/**< Sequence of upper constraints' bound vectors. */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								BooleanType useHotstarts,	/**< Shall QP solution be hotstarted? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t maxAllowedNWSR, 		/**< Maximum number of working set recalculations to be performed. */
								real_t& maxNWSR,			/**< Output: Maximum number of performed working set recalculations. */
								real_t& avgNWSR,			/**< Output: Average number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& avgCPUtime,			/**< Output: Average CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);


/** Solves an Online QP Benchmark problem (without constraints) as specified
 *  by the arguments. The maximum deviations from the given optimal solution
 *  as well as the maximum CPU time to solve each QP are determined.
 *
 *	Note: This variant is outdated and only kept to ensure 
 *		  backwards-compatibility!
 *
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOqpBenchmark(	int_t nQP,					/**< Number of QPs. */
								int_t nV,					/**< Number of variables. */
								const real_t* const _H,		/**< Hessian matrix. */
								const real_t* const g,		/**< Sequence of gradient vectors. */
								const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t& nWSR, 				/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);

/** Solves an Online QP Benchmark problem (without constraints) as specified
 *  by the arguments. The maximum deviations from the given optimal solution
 *  as well as the maximum CPU time to solve each QP are determined.
 *
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOqpBenchmark(	int_t nQP,					/**< Number of QPs. */
								int_t nV,					/**< Number of variables. */
								const real_t* const _H,		/**< Hessian matrix. */
								const real_t* const g,		/**< Sequence of gradient vectors. */
								const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								BooleanType useHotstarts,	/**< Shall QP solution be hotstarted? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t maxAllowedNWSR, 		/**< Maximum number of working set recalculations to be performed. */
								real_t& maxNWSR,			/**< Output: Maximum number of performed working set recalculations. */
								real_t& avgNWSR,			/**< Output: Average number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& avgCPUtime,			/**< Output: Average CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);


/** Runs an Online QP Benchmark problem and determines the maximum
 *  violation of the KKT optimality conditions as well as the 
 *  maximum CPU time to solve each QP.
 *
 * \return SUCCESSFUL_RETURN \n
		   RET_UNABLE_TO_READ_BENCHMARK \n
 		   RET_BENCHMARK_ABORTED */
returnValue runOqpBenchmark(	const char* path,			/**< Full path of the benchmark files (without trailing slash!). */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t& nWSR, 				/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);


/** Runs an Online QP Benchmark problem and determines the maximum
 *  violation of the KKT optimality conditions as well as the 
 *  maximum and average number of iterations and CPU time to solve 
 *	each QP.
 *
 * \return SUCCESSFUL_RETURN \n
		   RET_UNABLE_TO_READ_BENCHMARK \n
 		   RET_BENCHMARK_ABORTED */
returnValue runOqpBenchmark(	const char* path,			/**< Full path of the benchmark files (without trailing slash!). */
								BooleanType isSparse,		/**< Shall convert matrices to sparse format before solution? */
								BooleanType useHotstarts,	/**< Shall QP solution be hotstarted? */
								const Options& options,		/**< QP solver options to be used while solving benchmark problems. */
								int_t maxAllowedNWSR, 		/**< Maximum number of working set recalculations to be performed. */
								real_t& maxNWSR,			/**< Output: Maximum number of performed working set recalculations. */
								real_t& avgNWSR,			/**< Output: Average number of performed working set recalculations. */
								real_t& maxCPUtime,			/**< Output: Maximum CPU time required for solving each QP. */
								real_t& avgCPUtime,			/**< Output: Average CPU time required for solving each QP. */
								real_t& maxStationarity,	/**< Output: Maximum residual of stationarity condition. */
								real_t& maxFeasibility,		/**< Output: Maximum residual of primal feasibility condition. */
								real_t& maxComplementarity	/**< Output: Maximum residual of complementarity condition. */
								);

END_NAMESPACE_QPOASES


#endif	/* QPOASES_OQPINTERFACE_HPP */


/*
 *	end of file
 */
