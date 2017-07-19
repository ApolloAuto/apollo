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
 *	\file include/qpOASES/Utils.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of some utility functions for working with qpOASES.
 */


#ifndef QPOASES_UTILS_HPP
#define QPOASES_UTILS_HPP


#include <qpOASES/MessageHandling.hpp>


BEGIN_NAMESPACE_QPOASES


/** Prints a (possibly named) vector.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const v,	/**< Vector to be printed. */
					int_t n,				/**< Length of vector. */
					const char* name = 0	/**< Name of vector. */
					);

/** Prints a (possibly named) permuted vector.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const v,		/**< Vector to be printed. */
					int_t n,					/**< Length of vector. */
					const int_t* const V_idx,	/**< Pemutation vector. */
					const char* name = 0		/**< Name of vector. */
					);

/** Prints a (possibly named) matrix.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const M,	/**< Matrix to be printed. */
					int_t nrow,				/**< Row number of matrix. */
					int_t ncol,				/**< Column number of matrix. */
					const char* name = 0	/**< Name of matrix. */
					);

/** Prints a (possibly named) permuted matrix.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const M,		/**< Matrix to be printed. */
					int_t nrow,					/**< Row number of matrix. */
					int_t ncol	,				/**< Column number of matrix. */
					const int_t* const ROW_idx,	/**< Row pemutation vector. */
					const int_t* const COL_idx,	/**< Column pemutation vector. */
					const char* name = 0		/**< Name of matrix. */
					);

/** Prints a (possibly named) index array.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const int_t* const index,	/**< Index array to be printed. */
					int_t n,					/**< Length of index array. */
					const char* name = 0		/**< Name of index array. */
					);


/** Prints a string to desired output target (useful also for MATLAB output!).
 * \return SUCCESSFUL_RETURN */
returnValue myPrintf(	const char* s	/**< String to be written. */
						);


/** Prints qpOASES copyright notice.
 * \return SUCCESSFUL_RETURN */
returnValue printCopyrightNotice( );


/** Reads a real_t matrix from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	real_t* data,				/**< Matrix to be read from file. */
							int_t nrow,					/**< Row number of matrix. */
							int_t ncol,					/**< Column number of matrix. */
							const char* datafilename	/**< Data file name. */
							);

/** Reads a real_t vector from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	real_t* data,				/**< Vector to be read from file. */
							int_t n,					/**< Length of vector. */
							const char* datafilename	/**< Data file name. */
							);

/** Reads an integer (column) vector from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	int_t* data,				/**< Vector to be read from file. */
							int_t n,					/**< Length of vector. */
							const char* datafilename	/**< Data file name. */
							);


/** Writes a real_t matrix into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE  */
returnValue writeIntoFile(	const real_t* const data,		/**< Matrix to be written into file. */
							int_t nrow,						/**< Row number of matrix. */
							int_t ncol,						/**< Column number of matrix. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if data shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes a real_t vector into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE  */
returnValue writeIntoFile(	const real_t* const data,		/**< Vector to be written into file. */
							int_t n,						/**< Length of vector. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if data shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes an integer (column) vector into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE */
returnValue writeIntoFile(	const int_t* const integer,		/**< Integer vector to be written into file. */
							int_t n,						/**< Length of vector. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if integer shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes a real_t matrix/vector into a Matlab binary file.
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS
 		   RET_UNABLE_TO_WRITE_FILE */
returnValue writeIntoMatFile(	FILE* const matFile,		/**< Pointer to Matlab binary file. */
								const real_t* const data,	/**< Data to be written into file. */
								int_t nRows,				/**< Row number of matrix. */
								int_t nCols, 				/**< Column number of matrix. */
								const char* name			/**< Matlab name of matrix/vector to be stored. */
								);

/** Writes in integer matrix/vector into a Matlab binary file.
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS
 		   RET_UNABLE_TO_WRITE_FILE */
returnValue writeIntoMatFile(	FILE* const matFile,		/**< Pointer to Matlab binary file. */
								const int_t* const data,	/**< Data to be written into file. */
								int_t nRows,				/**< Row number of matrix. */
								int_t nCols,				/**< Column number of matrix. */
								const char* name			/**< Matlab name of matrix/vector to be stored. */
								);


/** Returns the current system time.
 * \return current system time */
real_t getCPUtime( );


/** Returns the N-norm of a vector.
 * \return >= 0.0: successful */
real_t getNorm(	const real_t* const v,	/**< Vector. */
				int_t n,				/**< Vector's dimension. */
				int_t type = 2			/**< Norm type, 1: one-norm, 2: Euclidean norm. */
				);


/** Tests whether two real_t-valued arguments are (numerically) equal.
 * \return	BT_TRUE:  arguments differ not more than TOL \n
		 	BT_FALSE: arguments differ more than TOL */
inline BooleanType isEqual(	real_t x,			/**< First real number. */
							real_t y,			/**< Second real number. */
							real_t TOL = ZERO	/**< Tolerance for comparison. */
							);


/** Tests whether a real-valued argument is (numerically) zero.
 * \return	BT_TRUE:  argument differs from 0.0 not more than TOL \n
		 	BT_FALSE: argument differs from 0.0 more than TOL */
inline BooleanType isZero(	real_t x,			/**< Real number. */
							real_t TOL = ZERO	/**< Tolerance for comparison. */
							);


/** Returns sign of a real-valued argument.
 * \return	 1.0: argument is non-negative \n
		 	-1.0: argument is negative */
inline real_t getSign(	real_t arg	/**< real-valued argument whose sign is to be determined. */
						);


/** Returns maximum of two integers.
 * \return	Maximum of two integers */
inline int_t getMax(	int_t x,	/**< First integer. */
						int_t y		/**< Second integer. */
						);
					
/** Returns minimum of two integers.
 * \return	Minimum of two integers */
inline int_t getMin(	int_t x,	/**< First integer. */
						int_t y		/**< Second integer. */
						);

	
/** Returns maximum of two reals.
 * \return	Maximum of two reals */
inline real_t getMax(	real_t x,	/**< First real number. */
						real_t y	/**< Second real number. */
						);

/** Returns minimum of two reals.
 * \return	Minimum of two reals */
inline real_t getMin(	real_t x,	/**< First real number. */
						real_t y	/**< Second real number. */
						);

/** Returns the absolute value of a real number.
 * \return	Absolute value of a real number */
inline real_t getAbs(	real_t x	/**< Real number. */
						);

/** Returns the square-root of a real number.
 * \return	Square-root of a real number */
inline real_t getSqrt(	real_t x	/**< Non-negative real number. */
						);


/** Computes the maximum violation of the KKT optimality conditions
 *	of given iterate for given QP data. */
returnValue getKktViolation(	int_t nV,									/**< Number of variables. */
								int_t nC,									/**< Number of constraints. */
								const real_t* const H,						/**< Hessian matrix (may be NULL if Hessian is zero or identity matrix). */
								const real_t* const g,						/**< Gradient vector. */
								const real_t* const A,						/**< Constraint matrix. */
								const real_t* const lb,						/**< Lower bound vector (on variables). */
								const real_t* const ub,						/**< Upper bound vector (on variables). */
								const real_t* const lbA,					/**< Lower constraints' bound vector. */
								const real_t* const ubA,					/**< Upper constraints' bound vector. */
								const real_t* const x,						/**< Primal trial vector. */
								const real_t* const y,						/**< Dual trial vector. */
								real_t& stat,								/**< Output: maximum value of stationarity condition residual. */
								real_t& feas,								/**< Output: maximum value of primal feasibility violation. */
								real_t& cmpl,								/**< Output: maximum value of complementarity residual. */
								const real_t* const workingSetB = 0,		/**< Working set of bounds (used to determine active bounds). */
								const real_t* const workingSetC = 0,		/**< Working set of constraints (used to determine active constraints). */
								BooleanType hasIdentityHessian = BT_FALSE	/**< Indicating whether Hessian matrix is identity matrix or not if NULL pointer is passed. */
								);

/** Computes the maximum violation of the KKT optimality conditions
 *	of given iterate for given QP data. */
returnValue getKktViolation(	int_t nV,									/**< Number of variables. */
								const real_t* const H,						/**< Hessian matrix (may be NULL if Hessian is zero or identity matrix). */
								const real_t* const g,						/**< Gradient vector. */
								const real_t* const lb,						/**< Lower bound vector (on variables). */
								const real_t* const ub,						/**< Upper bound vector (on variables). */
								const real_t* const x,						/**< Primal trial vector. */
								const real_t* const y,						/**< Dual trial vector. */
								real_t& stat,								/**< Output: maximum value of stationarity condition residual. */
								real_t& feas,								/**< Output: maximum value of primal feasibility violation. */
								real_t& cmpl,								/**< Output: maximum value of complementarity residual. */
								const real_t* const workingSetB = 0,		/**< Working set of bounds (used to determine active bounds). */
								BooleanType hasIdentityHessian = BT_FALSE	/**< Indicating whether Hessian matrix is identity matrix or not if NULL pointer is passed */
								);


/** Writes a value of BooleanType into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertBooleanTypeToString(	BooleanType value, 		/**< Value to be written. */
										char* const string		/**< Input: String of sufficient size, \n
																	 Output: String containing value. */
										);

/** Writes a value of SubjectToStatus into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertSubjectToStatusToString(	SubjectToStatus value,	/**< Value to be written. */
											char* const string		/**< Input: String of sufficient size, \n
																		 Output: String containing value. */
											);

/** Writes a value of PrintLevel into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertPrintLevelToString(	PrintLevel value, 		/**< Value to be written. */
										char* const string		/**< Input: String of sufficient size, \n
																	 Output: String containing value. */
										);


/** Converts a returnValue from an (S)QProblem(B) object into a more 
 *	simple status flag.
 *
 * \return  0: QP problem solved
 *          1: QP could not be solved within given number of iterations
 *         -1: QP could not be solved due to an internal error
 *         -2: QP is infeasible (and thus could not be solved)
 *         -3: QP is unbounded (and thus could not be solved)
 */
int_t getSimpleStatus(	returnValue returnvalue, 				/**< ReturnValue to be analysed. */
						BooleanType doPrintStatus = BT_FALSE	/**< Flag indicating whether simple status shall be printed to screen. */
						);


/** Normalises QP constraints.
 * \return SUCCESSFUL_RETURN \n
 *		   RET_INVALID_ARGUMENTS */
returnValue normaliseConstraints(	int_t nV,		/**< Number of variables. */
									int_t nC, 		/**< Number of constraints. */
									real_t* A,		/**< Input:  Constraint matrix, \n
														 Output: Normalised constraint matrix. */
									real_t* lbA,	/**< Input:  Constraints' lower bound vector, \n
														 Output: Normalised constraints' lower bound vector. */
									real_t* ubA,	/**< Input:  Constraints' upper bound vector, \n
														 Output: Normalised constraints' upper bound vector. */
									int_t type = 1	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
									);


#ifdef __DEBUG__
/** Writes matrix with given dimension into specified file. */
extern "C" void gdb_printmat(	const char *fname,			/**< File name. */
								real_t *M,					/**< Matrix to be written. */
								int_t n,					/**< Number of rows. */
								int_t m,					/**< Number of columns. */
								int_t ldim					/**< Leading dimension. */
								);
#endif /* __DEBUG__ */


#if defined(__DSPACE__) || defined(__XPCTARGET__) || defined(__C_WRAPPER__)
extern "C" void __cxa_pure_virtual( void );
#endif /* __DSPACE__ || __XPCTARGET__*/ 



END_NAMESPACE_QPOASES


#include <qpOASES/Utils.ipp>

#endif	/* QPOASES_UTILS_HPP */


/*
 *	end of file
 */
