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
 *	\file include/qpOASES/Indexlist.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the Indexlist class designed to manage index lists of
 *	constraints and bounds within a SubjectTo object.
 */


#ifndef QPOASES_INDEXLIST_HPP
#define QPOASES_INDEXLIST_HPP


#include <qpOASES/Utils.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Stores and manages index lists.
 *
 *	This class manages index lists of active/inactive bounds/constraints.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 */
class Indexlist
{
	/*
	 *  FRIENDS
	 */
	friend class DenseMatrix;
	friend class SymDenseMat;
	friend class SparseMatrix;
	friend class SparseMatrixRow;
	friend class SymSparseMat;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		Indexlist( );

		/** Constructor which takes the desired physical length of the index list. */
		Indexlist(	int_t n	/**< Physical length of index list. */
					);

		/** Copy constructor (deep copy). */
		Indexlist(	const Indexlist& rhs	/**< Rhs object. */
					);

		/** Destructor. */
		~Indexlist( );

		/** Assingment operator (deep copy). */
		Indexlist& operator=(	const Indexlist& rhs	/**< Rhs object. */
								);


		/** Initialises index list of desired physical length.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INVALID_ARGUMENTS */
		returnValue init(	int_t n = 0	/**< Physical length of index list. */
							);


		/** Creates an array of all numbers within the index set in correct order.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEXLIST_CORRUPTED */
		returnValue	getNumberArray(	int_t** const numberarray	/**< Output: Array of numbers (NULL on error). */
									) const;

		/** Creates an array of all numbers within the index set in correct order.
		 *	\return SUCCESSFUL_RETURN \n
					RET_INDEXLIST_CORRUPTED */
		returnValue	getISortArray(	int_t** const iSortArray	/**< Output: iSort Array. */
									) const;


		/** Determines the index within the index list at which a given number is stored.
		 *	\return >= 0: Index of given number. \n
		 			-1: Number not found. */
		int_t getIndex(	int_t givennumber	/**< Number whose index shall be determined. */
						) const;

		/** Returns the number stored at a given physical index.
		 *	\return >= 0: Number stored at given physical index. \n
		 			-RET_INDEXLIST_OUTOFBOUNDS */
		int_t getNumber(	int_t physicalindex	/**< Physical index of the number to be returned. */
							) const;


		/** Returns the current length of the index list.
		 *	\return Current length of the index list. */
		inline int_t getLength( ) const;

		/** Returns last number within the index list.
		 *	\return Last number within the index list. */
		inline int_t getLastNumber( ) const;


		/** Adds number to index list.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEXLIST_MUST_BE_REORDERD \n
		 			RET_INDEXLIST_EXCEEDS_MAX_LENGTH */
		returnValue addNumber(	int_t addnumber			/**< Number to be added. */
								);

		/** Removes number from index list.
		 *	\return SUCCESSFUL_RETURN */
		returnValue removeNumber(	int_t removenumber	/**< Number to be removed. */
									);

		/** Swaps two numbers within index list.
		 *	\return SUCCESSFUL_RETURN */
		returnValue swapNumbers(	int_t number1,		/**< First number for swapping. */
									int_t number2			/**< Second number for swapping. */
									);

		/** Determines if a given number is contained in the index set.
		 *	\return BT_TRUE iff number is contain in the index set */
		inline BooleanType isMember(	int_t _number		/**< Number to be tested for membership. */
										) const;


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );

		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const Indexlist& rhs	/**< Rhs object. */
							);

		/** Find first index j between -1 and length in sorted list of indices
		 *  iSort such that numbers[iSort[j]] <= i < numbers[iSort[j+1]]. Uses
		 *  bisection.
		 *  \return j. */
		int_t findInsert(	int_t i
							) const;


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		int_t* number;			/**< Array to store numbers of constraints or bounds. */
		int_t* iSort;			/**< Index list to sort vector \a number */

		int_t	length;			/**< Length of index list. */
		int_t	first;			/**< Physical index of first element. */
		int_t	last;			/**< Physical index of last element. */
		int_t	lastusedindex;	/**< Physical index of last entry in index list. */
		int_t	physicallength;	/**< Physical length of index list. */
};

END_NAMESPACE_QPOASES

#include <qpOASES/Indexlist.ipp>

#endif	/* QPOASES_INDEXLIST_HPP */


/*
 *	end of file
 */
