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
 *	\file include/qpOASES/SubjectTo.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the SubjectTo class designed to manage working sets of
 *	constraints and bounds within a QProblem.
 */


#ifndef QPOASES_SUBJECTTO_HPP
#define QPOASES_SUBJECTTO_HPP


#include <qpOASES/Indexlist.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Base class for managing working sets of bounds and constraints.
 *
 *	This class manages working sets of bounds and constraints by storing
 *	index sets and other status information.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 */
class SubjectTo
{
	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		SubjectTo( );

		/** Constructor which takes the number of constraints or bounds. */
		SubjectTo(	int_t _n 							/**< Number of constraints or bounds. */
					);

		/** Copy constructor (deep copy). */
		SubjectTo(	const SubjectTo& rhs				/**< Rhs object. */
					);

		/** Destructor. */
		virtual ~SubjectTo( );

		/** Assignment operator (deep copy). */
		SubjectTo& operator=(	const SubjectTo& rhs	/**< Rhs object. */
								);


		/** Initialises object with given number of constraints or bounds.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INVALID_ARGUMENTS */
		returnValue init(	int_t _n = 0				/**< Number of constraints or bounds. */
							);


		/** Returns number of constraints/bounds with given SubjectTo type.
		 *	\return Number of constraints/bounds with given type. */
		inline int_t getNumberOfType(	SubjectToType _type	/**< Type of (constraints') bound. */
										) const;


		/** Returns type of (constraints') bound.
		 *	\return Type of (constraints') bound \n
		 			RET_INDEX_OUT_OF_BOUNDS */
		inline SubjectToType getType(	int_t i			/**< Number of (constraints') bound. */
										) const;

		/** Returns status of (constraints') bound.
		 *	\return Status of (constraints') bound \n
		 			ST_UNDEFINED */
		inline SubjectToStatus getStatus(	int_t i		/**< Number of (constraints') bound. */
											) const;


		/** Sets type of (constraints') bound.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setType(	int_t i,			/**< Number of (constraints') bound. */
									SubjectToType value	/**< Type of (constraints') bound. */
									);

		/** Sets status of (constraints') bound.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS */
		inline returnValue setStatus(	int_t i,				/**< Number of (constraints') bound. */
										SubjectToStatus value	/**< Status of (constraints') bound. */
										);


		/** Sets status of lower (constraints') bounds. */
		inline void setNoLower(	BooleanType _status		/**< Status of lower (constraints') bounds. */
								);

		/** Sets status of upper (constraints') bounds. */
		inline void setNoUpper(	BooleanType _status		/**< Status of upper (constraints') bounds. */
								);


		/** Returns status of lower (constraints') bounds.
		 *	\return BT_TRUE if there is no lower (constraints') bound on any variable. */
		inline BooleanType hasNoLower( ) const;

		/** Returns status of upper bounds.
		 *	\return BT_TRUE if there is no upper (constraints') bound on any variable. */
		inline BooleanType hasNoUpper( ) const;


		/** Shifts forward type and status of all constraints/bounds by a given
		 *  offset. This offset has to lie within the range [0,n/2] and has to
		 *  be an integer divisor of the total number of constraints/bounds n.
		 *  Type and status of the first \<offset\> constraints/bounds is thrown away,
		 *  type and status of the last \<offset\> constraints/bounds is doubled,
		 *  e.g. for offset = 2: \n
		 *  shift( {c/b1,c/b2,c/b3,c/b4,c/b5,c/b6} ) = {c/b3,c/b4,c/b5,c/b6,c/b5,c/b6}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_INVALID_ARGUMENTS \n
		 			RET_SHIFTING_FAILED */
		virtual returnValue shift(	int_t offset	/**< Shift offset within the range [0,n/2] and integer divisor of n. */
									) = 0;

		/** Rotates forward type and status of all constraints/bounds by a given
		 *  offset. This offset has to lie within the range [0,n].
		 *  Example for offset = 2: \n
		 *  rotate( {c/b1,c/b2,c/b3,c/b4,c/b5,c/b6} ) = {c/b3,c/b4,c/b5,c/b6,c/b1,c/b2}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_ROTATING_FAILED */
		virtual returnValue rotate(	int_t offset	/**< Rotation offset within the range [0,n]. */
									) = 0;


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );
		
		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const SubjectTo& rhs	/**< Rhs object. */
							);


		/** Adds the index of a new constraint or bound to index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_ADDINDEX_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue addIndex(	Indexlist* const indexlist,	/**< Index list to which the new index shall be added. */
								int_t newnumber,			/**< Number of new constraint or bound. */
								SubjectToStatus newstatus	/**< Status of new constraint or bound. */
								);

		/** Removes the index of a constraint or bound from index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_REMOVEINDEX_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue removeIndex(	Indexlist* const indexlist,	/**< Index list from which the new index shall be removed. */
									int_t removenumber			/**< Number of constraint or bound to be removed. */
									);

		/** Swaps the indices of two constraints or bounds within the index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SWAPINDEX_FAILED \n
					RET_INVALID_ARGUMENTS */
		returnValue swapIndex(	Indexlist* const indexlist,	/**< Index list in which the indices shold be swapped. */
								int_t number1,				/**< Number of first constraint or bound. */
								int_t number2				/**< Number of second constraint or bound. */
								);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		int_t n;					/**< Total number of constraints/bounds. */

		SubjectToType* type; 		/**< Type of constraints/bounds. */
		SubjectToStatus* status;	/**< Status of constraints/bounds. */

		BooleanType noLower;	 	/**< This flag indicates if there is no lower bound on any variable. */
		BooleanType noUpper;	 	/**< This flag indicates if there is no upper bound on any variable. */
};


END_NAMESPACE_QPOASES

#include <qpOASES/SubjectTo.ipp>

#endif	/* QPOASES_SUBJECTTO_HPP */


/*
 *	end of file
 */
