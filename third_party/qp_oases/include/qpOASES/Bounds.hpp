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
 *	\file include/qpOASES/Bounds.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the Bounds class designed to manage working sets of
 *	bounds within a QProblem.
 */


#ifndef QPOASES_BOUNDS_HPP
#define QPOASES_BOUNDS_HPP


#include <qpOASES/SubjectTo.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Manages working sets of bounds (i.e. box constraints).
 *
 *	This class manages working sets of bounds (= box constraints) 
 *	by storing index sets and other status information.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 */
class Bounds : public SubjectTo
{
	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		Bounds( );

		/** Constructor which takes the number of bounds. */
		Bounds(	int_t _n								/**< Number of bounds. */
				);

		/** Copy constructor (deep copy). */
		Bounds(	const Bounds& rhs						/**< Rhs object. */
				);

		/** Destructor. */
		virtual ~Bounds( );

		/** Assignment operator (deep copy). */
		Bounds& operator=(	const Bounds& rhs			/**< Rhs object. */
							);


		/** Initialises object with given number of bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INVALID_ARGUMENTS */
		returnValue init(	int_t _n = 0				/**< Number of bounds. */
							);


		/** Initially adds number of a new (i.e. not yet in the list) bound to
		 *  given index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS \n
					RET_INVALID_ARGUMENTS */
		returnValue setupBound(	int_t number,			/**< Number of new bound. */
								SubjectToStatus _status	/**< Status of new bound. */
								);

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of free bounds; the order depends on the SujectToType
		 *  of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllFree( );

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of fixed bounds (on their lower bounds);
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllLower( );

		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set of fixed bounds (on their upper bounds);
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAllUpper( );


		/** Moves index of a bound from index list of fixed to that of free bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue moveFixedToFree(	int_t number			/**< Number of bound to be freed. */
										);

		/** Moves index of a bound from index list of free to that of fixed bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue moveFreeToFixed(	int_t number,			/**< Number of bound to be fixed. */
										SubjectToStatus _status	/**< Status of bound to be fixed. */
										);

		/** Flip fixed bound.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_BOUND_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue flipFixed( int_t number );

		/** Swaps the indices of two free bounds within the index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SWAPINDEX_FAILED */
		returnValue swapFree(	int_t number1,					/**< Number of first constraint or bound. */
								int_t number2					/**< Number of second constraint or bound. */
								);


		/** Returns number of variables.
		 *	\return Number of variables. */
		inline int_t getNV( ) const;

		/** Returns number of implicitly fixed variables.
		 *	\return Number of implicitly fixed variables. */
		inline int_t getNFV( ) const;

		/** Returns number of bounded (but possibly free) variables.
		 *	\return Number of bounded (but possibly free) variables. */
		inline int_t getNBV( ) const;

		/** Returns number of unbounded variables.
		 *	\return Number of unbounded variables. */
		inline int_t getNUV( ) const;

		/** Returns number of free variables.
		 *	\return Number of free variables. */
		inline int_t getNFR( ) const;

		/** Returns number of fixed variables.
		 *	\return Number of fixed variables. */
		inline int_t getNFX( ) const;


		/** Returns a pointer to free variables index list.
		 *	\return Pointer to free variables index list. */
		inline Indexlist* getFree( );

		/** Returns a pointer to fixed variables index list.
		 *	\return Pointer to fixed variables index list. */
		inline Indexlist* getFixed( );


		/** Shifts forward type and status of all bounds by a given
		 *  offset. This offset has to lie within the range [0,n/2] and has to
		 *  be an integer divisor of the total number of bounds n.
		 *  Type and status of the first \<offset\> bounds is thrown away,
		 *  type and status of the last \<offset\> bounds is doubled,
		 *  e.g. for offset = 2: \n
		 *  shift( {b1,b2,b3,b4,b5,b6} ) = {b3,b4,b5,b6,b5,b6}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_INVALID_ARGUMENTS \n
		 			RET_SHIFTING_FAILED */
		virtual returnValue shift(	int_t offset	/**< Shift offset within the range [0,n/2] and integer divisor of n. */
									);

		/** Rotates forward type and status of all bounds by a given
		 *  offset. This offset has to lie within the range [0,n].
		 *  Example for offset = 2: \n
		 *  rotate( {b1,b2,b3,b4,b5,b6} ) = {b3,b4,b5,b6,b1,b2}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_ROTATING_FAILED */
		virtual returnValue rotate(	int_t offset	/**< Rotation offset within the range [0,n]. */
									);


		/** Prints information on bounds object
		 *  (in particular, lists of free and fixed bounds.
		 * \return SUCCESSFUL_RETURN \n
				   RET_INDEXLIST_CORRUPTED */
		returnValue print( );


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );
		
		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const Bounds& rhs	/**< Rhs object. */
							);


		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set corresponding to the desired status;
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_BOUND_FAILED */
		returnValue setupAll(	SubjectToStatus _status	/**< Desired initial status for all bounds. */
								);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		Indexlist freee;		/**< Index list of free variables. */
		Indexlist fixed;		/**< Index list of fixed variables. */
};


END_NAMESPACE_QPOASES

#include <qpOASES/Bounds.ipp>

#endif	/* QPOASES_BOUNDS_HPP */


/*
 *	end of file
 */
