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
 *	\file include/qpOASES/Matrices.hpp
 *	\author Andreas Potschka, Hans Joachim Ferreau, Christian Kirches
 *	\version 3.2
 *	\date 2009-2017
 *
 *  Various matrix classes: Abstract base matrix class, dense and sparse matrices,
 *  including symmetry exploiting specializations.
 */



#ifndef QPOASES_MATRICES_HPP
#define QPOASES_MATRICES_HPP


#include <qpOASES/Utils.hpp>
#include <qpOASES/Indexlist.hpp>


BEGIN_NAMESPACE_QPOASES


	/**
 *	\brief Abstract base class for interfacing tailored matrix-vector operations.
 *
 *	Abstract base matrix class. Supplies interface to matrix vector
 *  products, including products with submatrices given by (ordered) working set
 *  index lists (see \a SubjectTo).
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class Matrix
{
	public:
		/** Default constructor. */
		Matrix( ) { doNotFreeMemory(); };

		/** Destructor. */
		virtual ~Matrix( ) { };

		/** Frees all internal memory. */
		virtual void free( ) = 0;

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix* duplicate( ) const = 0;

		/** Returns i-th diagonal entry.
		 *	\return i-th diagonal entry */
		virtual real_t diag(	int_t i			/**< Index. */
								) const = 0;

		/** Checks whether matrix is square and diagonal.
		 *	\return BT_TRUE  iff matrix is square and diagonal; \n
		 *	        BT_FALSE otherwise. */
		virtual BooleanType isDiag( ) const = 0;

		/** Get the N-norm of the matrix
		 *  \return N-norm of the matrix
		 */
		virtual real_t getNorm(	int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
								) const = 0;

		/** Get the N-norm of a row
		 *  \return N-norm of row \a rNum
		 */
		virtual real_t getRowNorm(	int_t rNum,		/**< Row number. */
									int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
									) const = 0;

		/** Get the N-norm of all rows
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getRowNorm(  real_t* norm,   /**< Norm of each row. */
										 int_t type = 2  /**< Norm type, 1: one-norm, 2: Euclidean norm. */
										 ) const = 0;

		/** Retrieve indexed entries of matrix row multiplied by alpha.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue getRow(	int_t rNum,						/**< Row number. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									real_t alpha,					/**< Scalar factor. */
									real_t* row						/**< Output row vector. */
									) const = 0;

		/** Retrieve indexed entries of matrix column multiplied by alpha.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue getCol(	int_t cNum,						/**< Column number. */
									const Indexlist* const irows,	/**< Index list specifying rows. */
									real_t alpha,					/**< Scalar factor. */
									real_t* col						/**< Output column vector. */
									) const = 0;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				const Indexlist* const irows,	/**< Index list specifying rows. */
				const Indexlist* const icols,	/**< Index list specifying columns. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.  This version retrieves one
		 *  column.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				const Indexlist* const irows,	/**< Index list specifying rows. */
				int_t idx_icol,					/**< Index list specifying columns. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.  This version retrieves one row.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				int_t idx_row,					/**< Row number. */
				const Indexlist* const icols,	/**< Index list specifying columns. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				int_t irowsLength,				/**< Number of rows. */
				const int_t* const irowsNumber, /**< Array with row numbers. */
				int_t icolsLength,				/**< Number of columns. */
				const int_t* const icolsNumber, /**< Array with column numbers. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const = 0;

		/** Evaluate Y=alpha*A*X + beta*Y.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue times (	int_t xN,				/**< Number of vectors to multiply. */
									real_t alpha,			/**< Scalar factor for matrix vector product. */
									const real_t* x,		/**< Input vector to be multiplied. */
									int_t xLD,				/**< Leading dimension of input x. */
									real_t beta,			/**< Scalar factor for y. */
									real_t* y,				/**< Output vector of results. */
									int_t yLD				/**< Leading dimension of output y. */
									) const = 0;

		/** Evaluate Y=alpha*A'*X + beta*Y.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue transTimes (	int_t xN,			/**< Number of vectors to multiply. */
											real_t alpha,		/**< Scalar factor for matrix vector product. */
											const real_t* x,	/**< Input vector to be multiplied. */
											int_t xLD,			/**< Leading dimension of input x. */
											real_t beta,		/**< Scalar factor for y. */
											real_t* y,			/**< Output vector of results. */
											int_t yLD			/**< Leading dimension of output y. */
											) const = 0;

		/** Evaluate matrix vector product with submatrix given by Indexlist.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue times (	const Indexlist* const irows,	/**< Index list specifying rows. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									int_t xN,						/**< Number of vectors to multiply. */
									real_t alpha,					/**< Scalar factor for matrix vector product. */
									const real_t* x,				/**< Input vector to be multiplied. */
									int_t xLD,						/**< Leading dimension of input x. */
									real_t beta,					/**< Scalar factor for y. */
									real_t* y,						/**< Output vector of results. */
									int_t yLD,						/**< Leading dimension of output y. */
									BooleanType yCompr = BT_TRUE	/**< Compressed storage for y. */
									) const = 0;

		/** Evaluate matrix transpose vector product.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue transTimes (	const Indexlist* const irows,	/**< Index list specifying rows. */
											const Indexlist* const icols,	/**< Index list specifying columns. */
											int_t xN,						/**< Number of vectors to multiply. */
											real_t alpha,					/**< Scalar factor for matrix vector product. */
											const real_t* x,				/**< Input vector to be multiplied. */
											int_t xLD,						/**< Leading dimension of input x. */
											real_t beta,					/**< Scalar factor for y. */
											real_t* y,						/**< Output vector of results. */
											int_t yLD						/**< Leading dimension of output y. */
											) const = 0;

		/** Adds given offset to diagonal of matrix.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_NO_DIAGONAL_AVAILABLE */
		virtual returnValue addToDiag(	real_t alpha		/**< Diagonal offset. */
										) = 0;

		/** Allocates and creates dense matrix array in row major format.
		 *
		 *  Note: Calling function has to free allocated memory!
		 *
		 *  \return Pointer to matrix array. 
		 */
		virtual real_t* full() const = 0;


		/** Prints matrix to screen.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue print(	const char* name = 0	/** Name of matrix. */
									) const = 0;

		/** Write matrix to file.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue writeToFile( FILE* output_file, const char* prefix ) const = 0;

		/** Returns whether internal memory needs to be de-allocated.
		 *	\return BT_TRUE  iff internal memory needs to be de-allocated, \n
		 			BT_FALSE otherwise */
		BooleanType needToFreeMemory( ) const { return freeMemory; };

		/** Enables de-allocation of internal memory. */
		void doFreeMemory( ) { freeMemory = BT_TRUE; };

		/** Disables de-allocation of internal memory. */
		void doNotFreeMemory( ) { freeMemory = BT_FALSE; };


	protected:
			BooleanType freeMemory;				/**< Indicating whether internal memory needs to be de-allocated. */

};


/**
 *	\brief Abstract base class for interfacing matrix-vector operations tailored to symmetric matrices.
 *
 *	Abstract base class for symmetric matrices. Extends Matrix interface with
 *  bilinear form evaluation.
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class SymmetricMatrix : public virtual Matrix
{
	public:
		/** Default constructor. */
		SymmetricMatrix( ) { };

		/** Destructor. */
		virtual ~SymmetricMatrix( ) { };

		/** Returns a deep-copy of the SymmetricMatrix object.
		 *	\return Deep-copy of SymmetricMatrix object */
		virtual SymmetricMatrix* duplicateSym( ) const = 0;


		/** Compute bilinear form y = x'*H*x using submatrix given by index list.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue bilinear(	const Indexlist* const icols,	/**< Index list specifying columns of x. */
										int_t xN,						/**< Number of vectors to multiply. */
										const real_t* x,				/**< Input vector to be multiplied (uncompressed). */
										int_t xLD,						/**< Leading dimension of input x. */
										real_t* y,						/**< Output vector of results (compressed). */
										int_t yLD						/**< Leading dimension of output y. */
										) const = 0;

};


/**
 *	\brief Interfaces matrix-vector operations tailored to general dense matrices.
 *
 *	Dense matrix class (row major format).
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class DenseMatrix : public virtual Matrix
{
	public:
		/** Default constructor. */
		DenseMatrix( ) : nRows(0), nCols(0), leaDim(0), val(0) { };

		/** Constructor from vector of values.
		 *  Caution: Data pointer must be valid throughout lifetime
		 */
		DenseMatrix(	int_t m,		/**< Number of rows. */
						int_t n,		/**< Number of columns. */
						int_t lD,		/**< Leading dimension. */
						real_t* v		/**< Values. */
						) : nRows(m), nCols(n), leaDim(lD), val(v) {}


		/** Destructor. */
		virtual ~DenseMatrix( );

		/** Frees all internal memory. */
		virtual void free( );

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix *duplicate( ) const;

		/** Returns i-th diagonal entry.
		 *	\return i-th diagonal entry */
		virtual real_t diag(	int_t i			/**< Index. */
								) const;

		/** Checks whether matrix is square and diagonal.
		 *	\return BT_TRUE  iff matrix is square and diagonal; \n
		 *		    BT_FALSE otherwise. */
		virtual BooleanType isDiag( ) const;

        /** Get the N-norm of the matrix
         *  \return N-norm of the matrix
         */
        virtual real_t getNorm(	int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
								) const;
		
        /** Get the N-norm of a row
         *  \return N-norm of row \a rNum
         */
        virtual real_t getRowNorm(	int_t rNum,		/**< Row number. */
									int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
									) const;

        /** Get the N-norm of all rows
         *  \return SUCCESSFUL_RETURN */
        virtual returnValue getRowNorm(  real_t* norm,   /**< Norm of each row. */
                                         int_t type = 2  /**< Norm type, 1: one-norm, 2: Euclidean norm. */
                                         ) const;

        /** Retrieve indexed entries of matrix row multiplied by alpha.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getRow(	int_t rNum,						/**< Row number. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									real_t alpha,					/**< Scalar factor. */
									real_t* row						/**< Output row vector. */
									) const;

		/** Retrieve indexed entries of matrix column multiplied by alpha.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getCol(
				int_t cNum,						/**< Column number. */
				const Indexlist* const irows,	/**< Index list specifying rows. */
				real_t alpha,					/**< Scalar factor. */
				real_t* col						/**< Output column vector. */
				) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				int_t irowsLength,				/**< Number of rows. */
				const int_t* const irowsNumber, /**< Array with row numbers. */
				int_t icolsLength,				/**< Number of columns. */
				const int_t* const icolsNumber, /**< Array with column numbers. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;


		/** Evaluate Y=alpha*A*X + beta*Y.
		 *  \return SUCCESSFUL_RETURN. */
		virtual returnValue times(	int_t xN,				/**< Number of vectors to multiply. */
									real_t alpha,			/**< Scalar factor for matrix vector product. */
									const real_t* x,		/**< Input vector to be multiplied. */
									int_t xLD,				/**< Leading dimension of input x. */
									real_t beta,			/**< Scalar factor for y. */
									real_t* y,				/**< Output vector of results. */
									int_t yLD				/**< Leading dimension of output y. */
									) const;

		/** Evaluate Y=alpha*A'*X + beta*Y.
		 *  \return SUCCESSFUL_RETURN. */
		virtual returnValue transTimes(	int_t xN,			/**< Number of vectors to multiply. */
										real_t alpha,		/**< Scalar factor for matrix vector product. */
										const real_t* x,	/**< Input vector to be multiplied. */
										int_t xLD,			/**< Leading dimension of input x. */
										real_t beta,		/**< Scalar factor for y. */
										real_t* y,			/**< Output vector of results. */
										int_t yLD			/**< Leading dimension of output y. */
										) const;

		/** Evaluate matrix vector product with submatrix given by Indexlist.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue times(	const Indexlist* const irows,	/**< Index list specifying rows. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									int_t xN,						/**< Number of vectors to multiply. */
									real_t alpha,					/**< Scalar factor for matrix vector product. */
									const real_t* x,				/**< Input vector to be multiplied. */
									int_t xLD,						/**< Leading dimension of input x. */
									real_t beta,					/**< Scalar factor for y. */
									real_t* y,						/**< Output vector of results. */
									int_t yLD,						/**< Leading dimension of output y. */
									BooleanType yCompr = BT_TRUE	/**< Compressed storage for y. */
									) const;

		/** Evaluate matrix transpose vector product.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue transTimes(	const Indexlist* const irows,	/**< Index list specifying rows. */
										const Indexlist* const icols,	/**< Index list specifying columns. */
										int_t xN,						/**< Number of vectors to multiply. */
										real_t alpha,					/**< Scalar factor for matrix vector product. */
										const real_t* x,				/**< Input vector to be multiplied. */
										int_t xLD,						/**< Leading dimension of input x. */
										real_t beta,					/**< Scalar factor for y. */
										real_t* y,						/**< Output vector of results. */
										int_t yLD						/**< Leading dimension of output y. */
										) const;

		/** Adds given offset to diagonal of matrix.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_NO_DIAGONAL_AVAILABLE */
		virtual returnValue addToDiag(	real_t alpha		/**< Diagonal offset. */
										);


		/** Allocates and creates dense matrix array in row major format.
		 *
		 *  Note: Calling function has to free allocated memory!
		 *
		 *  \return Pointer to matrix array. 
		 */
		virtual real_t* full() const;


		/** Prints matrix to screen.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue print( 	const char* name = 0	/** Name of matrix. */
									) const;

		/** Write matrix to file.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue writeToFile( FILE* output_file, const char* prefix ) const;

	protected:
		int_t nRows;		/**< Number of rows. */
		int_t nCols;		/**< Number of columns. */
		int_t leaDim;		/**< Leading dimension. */
		real_t* val;		/**< Vector of entries. */
};


/**
 *	\brief Interfaces matrix-vector operations tailored to symmetric dense matrices.
 *
 *	Symmetric dense matrix class.
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class SymDenseMat : public DenseMatrix, public SymmetricMatrix
{
	public:
		/** Default constructor. */
		SymDenseMat() : DenseMatrix() { };

		/** Constructor from vector of values. */
		SymDenseMat(	int_t m,		/**< Number of rows. */
						int_t n,		/**< Number of columns. */
						int_t lD,		/**< Leading dimension. */
						real_t* v		/**< Values. */
						) : DenseMatrix(m, n, lD, v) { };

		/** Destructor. */
		virtual ~SymDenseMat() { };

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix *duplicate( ) const;

		/** Returns a deep-copy of the SymmetricMatrix object.
		 *	\return Deep-copy of SymmetricMatrix object */
		virtual SymmetricMatrix* duplicateSym( ) const;


		/** Compute bilinear form y = x'*H*x using submatrix given by index list.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue bilinear(	const Indexlist* const icols,	/**< Index list specifying columns of x. */
										int_t xN,						/**< Number of vectors to multiply. */
										const real_t* x,				/**< Input vector to be multiplied (uncompressed). */
										int_t xLD,						/**< Leading dimension of input x. */
										real_t* y,						/**< Output vector of results (compressed). */
										int_t yLD						/**< Leading dimension of output y. */
										) const;
};


/**
 *	\brief Interfaces matrix-vector operations tailored to general sparse matrices.
 *
 *	Sparse matrix class (col compressed format).
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class SparseMatrix : public virtual Matrix
{
	public:
		/** Default constructor. */
		SparseMatrix( );

		/** Constructor with arguments. */
		SparseMatrix(	int_t nr, 			/**< Number of rows. */
						int_t nc, 			/**< Number of columns. */
						sparse_int_t* r, 	/**< Row indices (length). */
						sparse_int_t* c, 	/**< Indices to first entry of columns (nCols+1). */
						real_t* v			/**< Vector of entries (length). */
						);

		/** Constructor from dense matrix. */
		SparseMatrix(	int_t nr, 				/**< Number of rows. */
						int_t nc,		 		/**< Number of columns. */
						int_t ld,				/**< Leading dimension. */
						const real_t*  const v	/**< Row major stored matrix elements. */
						);

		/** Destructor. */
		virtual ~SparseMatrix( );

		/** Frees all internal memory. */
		virtual void free( );

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix *duplicate( ) const;


		/** Sets value array. 
		 *	
		 *	Thanks to Frank Chuang.
		 */
		virtual void setVal(	const real_t* newVal	/**< ... */
								);

		/** Returns i-th diagonal entry.
		 *	\return i-th diagonal entry (or INFTY if diagonal does not exist)*/
		virtual real_t diag(	int_t i			/**< Index. */
								) const;

		/** Checks whether matrix is square and diagonal.
		 *	\return BT_TRUE  iff matrix is square and diagonal; \n
		 *	        BT_FALSE otherwise. */
		virtual BooleanType isDiag( ) const;


        /** Get the N-norm of the matrix
         *  \return N-norm of the matrix
         */
        virtual real_t getNorm(	int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
								) const;

        /** Get the N-norm of a row
         *  \return N-norm of row \a rNum
         */
        virtual real_t getRowNorm(	int_t rNum,		/**< Row number. */
									int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
									) const;

        /** Get the N-norm of all rows
         *  \return SUCCESSFUL_RETURN */
        virtual returnValue getRowNorm(  real_t* norm,   /**< Norm of each row. */
                                         int_t type = 2  /**< Norm type, 1: one-norm, 2: Euclidean norm. */
                                         ) const;

		/** Retrieve indexed entries of matrix row multiplied by alpha. */
		virtual returnValue getRow(	int_t rNum,						/**< Row number. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									real_t alpha,					/**< Scalar factor. */
									real_t* row						/**< Output row vector. */
									) const;

		/** Retrieve indexed entries of matrix column multiplied by alpha. */
		virtual returnValue getCol(	int_t cNum,						/**< Column number. */
									const Indexlist* const irows,	/**< Index list specifying rows. */
									real_t alpha,					/**< Scalar factor. */
									real_t* col						/**< Output column vector. */
									) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				int_t irowsLength,				/**< Number of rows. */
				const int_t* const irowsNumber, /**< Array with row numbers. */
				int_t icolsLength,				/**< Number of columns. */
				const int_t* const icolsNumber, /**< Array with column numbers. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;

		/** Evaluate Y=alpha*A*X + beta*Y. */
		virtual returnValue times (	int_t xN,				/**< Number of vectors to multiply. */
									real_t alpha,			/**< Scalar factor for matrix vector product. */
									const real_t* x,		/**< Input vector to be multiplied. */
									int_t xLD,				/**< Leading dimension of input x. */
									real_t beta,			/**< Scalar factor for y. */
									real_t* y,				/**< Output vector of results. */
									int_t yLD				/**< Leading dimension of output y. */
									) const;

		/** Evaluate Y=alpha*A'*X + beta*Y. */
		virtual returnValue transTimes (	int_t xN,			/**< Number of vectors to multiply. */
											real_t alpha,		/**< Scalar factor for matrix vector product. */
											const real_t* x,	/**< Input vector to be multiplied. */
											int_t xLD,			/**< Leading dimension of input x. */
											real_t beta,		/**< Scalar factor for y. */
											real_t* y,			/**< Output vector of results. */
											int_t yLD			/**< Leading dimension of output y. */
											) const;

		/** Evaluate matrix vector product with submatrix given by Indexlist. */
		virtual returnValue times (	const Indexlist* const irows,	/**< Index list specifying rows. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									int_t xN,						/**< Number of vectors to multiply. */
									real_t alpha,					/**< Scalar factor for matrix vector product. */
									const real_t* x,				/**< Input vector to be multiplied. */
									int_t xLD,						/**< Leading dimension of input x. */
									real_t beta,					/**< Scalar factor for y. */
									real_t* y,						/**< Output vector of results. */
									int_t yLD,						/**< Leading dimension of output y. */
									BooleanType yCompr = BT_TRUE	/**< Compressed storage for y. */
									) const;

		/** Evaluate matrix transpose vector product. */
		virtual returnValue transTimes (	const Indexlist* const irows,	/**< Index list specifying rows. */
											const Indexlist* const icols,	/**< Index list specifying columns. */
											int_t xN,						/**< Number of vectors to multiply. */
											real_t alpha,					/**< Scalar factor for matrix vector product. */
											const real_t* x,				/**< Input vector to be multiplied. */
											int_t xLD,						/**< Leading dimension of input x. */
											real_t beta,					/**< Scalar factor for y. */
											real_t* y,						/**< Output vector of results. */
											int_t yLD						/**< Leading dimension of output y. */
											) const;

		/** Adds given offset to diagonal of matrix.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_NO_DIAGONAL_AVAILABLE */
		virtual returnValue addToDiag(	real_t alpha		/**< Diagonal offset. */
										);

		/** Create jd field from ir and jc.
		 *  \return Pointer to jd. */
		sparse_int_t* createDiagInfo();

		/** Allocates and creates dense matrix array in row major format.
		 *
		 *  Note: Calling function has to free allocated memory!
		 *
		 *  \return Pointer to matrix array. 
		 */
		virtual real_t* full() const;

		/** Prints matrix to screen.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue print( 	const char* name = 0	/** Name of matrix. */
									) const;

		/** Write matrix to file.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue writeToFile( FILE* output_file, const char* prefix ) const;


	protected:
		int_t nRows;		/**< Number of rows. */
		int_t nCols;		/**< Number of columns. */
		sparse_int_t* ir;	/**< Row indices (length). */
		sparse_int_t* jc;	/**< Indices to first entry of columns (nCols+1). */
		sparse_int_t* jd;	/**< Indices to first entry of lower triangle (including diagonal) (nCols). */
		real_t* val;		/**< Vector of entries (length). */
};


/**
 *	\brief Interfaces matrix-vector operations tailored to general sparse matrices.
 *
 *	Sparse matrix class (row compressed format).
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class SparseMatrixRow : public virtual Matrix
{
	public:
		/** Default constructor. */
		SparseMatrixRow( );

		/** Constructor with arguments. */
		SparseMatrixRow(	int_t nr, 			/**< Number of rows. */
							int_t nc, 			/**< Number of columns. */
							sparse_int_t* r, 	/**< Indices to first entry of rows (nRows+1). */
							sparse_int_t* c, 	/**< Column indices (length). */
							real_t* v			/**< Vector of entries (length). */
							);

		/** Constructor from dense matrix. */
		SparseMatrixRow(	int_t nr, 				/**< Number of rows. */
							int_t nc,			 	/**< Number of columns. */
							int_t ld,				/**< Leading dimension. */
							const real_t*  const v	/**< Row major stored matrix elements. */
							);

		/** Destructor. */
		virtual ~SparseMatrixRow( );

		/** Frees all internal memory. */
		virtual void free( );

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix *duplicate( ) const;

		/** Returns i-th diagonal entry.
		 *	\return i-th diagonal entry (or INFTY if diagonal does not exist)*/
		virtual real_t diag(	int_t i			/**< Index. */
								) const;

		/** Checks whether matrix is square and diagonal.
		 *	\return BT_TRUE  iff matrix is square and diagonal; \n
		 *	        BT_FALSE otherwise. */
		virtual BooleanType isDiag( ) const;

		
        /** Get the N-norm of the matrix
         *  \return N-norm of the matrix
         */
        virtual real_t getNorm(	int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
								) const;

        /** Get the N-norm of a row
         *  \return N-norm of row \a rNum
         */
        virtual real_t getRowNorm(	int_t rNum,		/**< Row number. */
									int_t type = 2	/**< Norm type, 1: one-norm, 2: Euclidean norm. */
									) const;

        /** Get the N-norm of all rows
         *  \return SUCCESSFUL_RETURN */
        virtual returnValue getRowNorm(  real_t* norm,   /**< Norm of each row. */
                                         int_t type = 2  /**< Norm type, 1: one-norm, 2: Euclidean norm. */
                                         ) const;

		/** Retrieve indexed entries of matrix row multiplied by alpha. */
		virtual returnValue getRow (	int_t rNum,						/**< Row number. */
										const Indexlist* const icols,	/**< Index list specifying columns. */
										real_t alpha,					/**< Scalar factor. */
										real_t* row						/**< Output row vector. */
										) const;

		/** Retrieve indexed entries of matrix column multiplied by alpha. */
		virtual returnValue getCol (	int_t cNum,						/**< Column number. */
										const Indexlist* const irows,	/**< Index list specifying rows. */
										real_t alpha,					/**< Scalar factor. */
										real_t* col						/**< Output column vector. */
										) const;

		/** Retrieve entries of submatrix in Harwell-Boeing sparse format.
		 *  If irn, jcn, and avals are null, this only counts the number of nonzeros.
		 *  Otherwise, numNonzeros containts the size of irn, jcn, and avals on entry,
		 *  and the written number of entries on return.
		 *  \return SUCCESSFUL_RETURN */
		virtual returnValue getSparseSubmatrix(
				int_t irowsLength,				/**< Number of rows. */
				const int_t* const irowsNumber, /**< Array with row numbers. */
				int_t icolsLength,				/**< Number of columns. */
				const int_t* const icolsNumber, /**< Array with column numbers. */
				int_t rowoffset,				/**< Offset for row entries. */
				int_t coloffset,				/**< Offset for row entries. */
				int_t& numNonzeros,				/**< Number of nonzeros in submatrix. */
				int_t* irn,						/**< Row position of entries (as position in irows) plus rowoffset. */
				int_t* jcn,						/**< Column position of entries (as position in irows) plus coloffset. */
				real_t* avals,					/**< Numerical values of the entries. */
				BooleanType only_lower_triangular = BT_FALSE /**< if true, only the lower triangular portion is returned.  This can only be true for symmetric matrices and if irows==jcols. */
				) const;

		/** Evaluate Y=alpha*A*X + beta*Y. */
		virtual returnValue times(	int_t xN,				/**< Number of vectors to multiply. */
									real_t alpha,			/**< Scalar factor for matrix vector product. */
									const real_t* x,		/**< Input vector to be multiplied. */
									int_t xLD,				/**< Leading dimension of input x. */
									real_t beta,			/**< Scalar factor for y. */
									real_t* y,				/**< Output vector of results. */
									int_t yLD				/**< Leading dimension of output y. */
									) const;

		/** Evaluate Y=alpha*A'*X + beta*Y. */
		virtual returnValue transTimes(	int_t xN,			/**< Number of vectors to multiply. */
										real_t alpha,		/**< Scalar factor for matrix vector product. */
										const real_t* x,	/**< Input vector to be multiplied. */
										int_t xLD,			/**< Leading dimension of input x. */
										real_t beta,		/**< Scalar factor for y. */
										real_t* y,			/**< Output vector of results. */
										int_t yLD			/**< Leading dimension of output y. */
										) const;

		/** Evaluate matrix vector product with submatrix given by Indexlist. */
		virtual returnValue times(	const Indexlist* const irows,	/**< Index list specifying rows. */
									const Indexlist* const icols,	/**< Index list specifying columns. */
									int_t xN,						/**< Number of vectors to multiply. */
									real_t alpha,					/**< Scalar factor for matrix vector product. */
									const real_t* x,				/**< Input vector to be multiplied. */
									int_t xLD,						/**< Leading dimension of input x. */
									real_t beta,					/**< Scalar factor for y. */
									real_t* y,						/**< Output vector of results. */
									int_t yLD,						/**< Leading dimension of output y. */
									BooleanType yCompr = BT_TRUE	/**< Compressed storage for y. */
									) const;

		/** Evaluate matrix transpose vector product. */
		virtual returnValue transTimes(	const Indexlist* const irows,	/**< Index list specifying rows. */
										const Indexlist* const icols,	/**< Index list specifying columns. */
										int_t xN,						/**< Number of vectors to multiply. */
										real_t alpha,					/**< Scalar factor for matrix vector product. */
										const real_t* x,				/**< Input vector to be multiplied. */
										int_t xLD,						/**< Leading dimension of input x. */
										real_t beta,					/**< Scalar factor for y. */
										real_t* y,						/**< Output vector of results. */
										int_t yLD						/**< Leading dimension of output y. */
										) const;

		/** Adds given offset to diagonal of matrix.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_NO_DIAGONAL_AVAILABLE */
		virtual returnValue addToDiag(	real_t alpha		/**< Diagonal offset. */
										);

		/** Create jd field from ir and jc.
		 *  \return Pointer to jd. */
		sparse_int_t* createDiagInfo();

		/** Allocates and creates dense matrix array in row major format.
		 *
		 *  Note: Calling function has to free allocated memory!
		 *
		 *  \return Pointer to matrix array. 
		 */
		virtual real_t* full() const;

		/** Prints matrix to screen.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue print( 	const char* name = 0	/** Name of matrix. */
									) const;

		/** Write matrix to file.
		 *	\return SUCCESSFUL_RETURN */
		virtual returnValue writeToFile( FILE* output_file, const char* prefix ) const;

	protected:
		int_t nRows;		/**< Number of rows. */
		int_t nCols;		/**< Number of columns. */
		sparse_int_t* jr;	/**< Indices to first entry of row (nRows+1). */
		sparse_int_t* ic;	/**< Column indices (length). */
		sparse_int_t* jd;	/**< Indices to first entry of upper triangle (including diagonal) (nRows). */
		real_t* val;		/**< Vector of entries (length). */
};


/**
 *	\brief Interfaces matrix-vector operations tailored to symmetric sparse matrices.
 *
 *	Symmetric sparse matrix class (column compressed format).
 *
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2011-2017
 */
class SymSparseMat : public SymmetricMatrix, public SparseMatrix
{
	public:
		/** Default constructor. */
		SymSparseMat( ) : SparseMatrix( ) { };

		/** Constructor with arguments. */
		SymSparseMat(	int_t nr, 			/**< Number of rows. */
						int_t nc, 			/**< Number of columns. */
						sparse_int_t* r, 	/**< Row indices (length). */
						sparse_int_t* c, 	/**< Indices to first entry of columns (nCols+1). */
						real_t* v			/**< Vector of entries (length). */
						) : SparseMatrix(nr, nc, r, c, v) { };

		/** Constructor from dense matrix. */
		SymSparseMat(	int_t nr, 				/**< Number of rows. */
						int_t nc,		 		/**< Number of columns. */
						int_t ld,				/**< Leading dimension. */
						const real_t*  const v	/**< Row major stored matrix elements. */
						) : SparseMatrix(nr, nc, ld, v) { };

		/** Destructor. */
		virtual ~SymSparseMat( ) { };

		/** Returns a deep-copy of the Matrix object.
		 *	\return Deep-copy of Matrix object */
		virtual Matrix *duplicate( ) const;

		/** Returns a deep-copy of the SymmetricMatrix object.
		 *	\return Deep-copy of SymmetricMatrix object */
		virtual SymmetricMatrix* duplicateSym( ) const;


		/** Compute bilinear form y = x'*H*x using submatrix given by index list.
		*	\return SUCCESSFUL_RETURN */
		virtual returnValue bilinear(	const Indexlist* const icols,	/**< Index list specifying columns of x. */
										int_t xN,						/**< Number of vectors to multiply. */
										const real_t* x,				/**< Input vector to be multiplied (uncompressed). */
										int_t xLD,						/**< Leading dimension of input x. */
										real_t* y,						/**< Output vector of results (compressed). */
										int_t yLD						/**< Leading dimension of output y. */
										) const;
};


END_NAMESPACE_QPOASES


#endif	/* QPOASES_MATRICES_HPP */
