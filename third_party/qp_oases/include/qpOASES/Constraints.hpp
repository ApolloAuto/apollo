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
 *	\file include/qpOASES/Constraints.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the Constraints class designed to manage working sets of
 *	constraints within a QProblem.
 */


#ifndef QPOASES_CONSTRAINTS_HPP
#define QPOASES_CONSTRAINTS_HPP


#include <qpOASES/SubjectTo.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Manages working sets of constraints.
 *
 *	This class manages working sets of constraints by storing
 *	index sets and other status information.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 */
class Constraints : public SubjectTo
{
	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		Constraints( );

		/** Constructor which takes the number of constraints. */
		Constraints(	int_t _n						/**< Number of constraints. */
						);

		/** Copy constructor (deep copy). */
		Constraints(	const Constraints& rhs			/**< Rhs object. */
						);

		/** Destructor. */
		virtual ~Constraints( );

		/** Assignment operator (deep copy). */
		Constraints& operator=(	const Constraints& rhs	/**< Rhs object. */
								);


		/** Initialises object with given number of constraints.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INVALID_ARGUMENTS */
		returnValue init(	int_t _n = 0				/**< Number of constraints. */
							);


		/** Initially adds number of a new (i.e. not yet in the list) constraint to
		 *  a given index set.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_CONSTRAINT_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS \n
					RET_INVALID_ARGUMENTS */
		returnValue setupConstraint(	int_t number,			/**< Number of new constraint. */
										SubjectToStatus _status	/**< Status of new constraint. */
										);

		/** Initially adds all enabled numbers of new (i.e. not yet in the list) constraints to
		 *  to the index set of inactive constraints; the order depends on the SujectToType
		 *  of each index. Only disabled constraints are added to index set of disabled constraints!
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_CONSTRAINT_FAILED */
		returnValue setupAllInactive( );

		/** Initially adds all enabled numbers of new (i.e. not yet in the list) constraints to
		 *  to the index set of active constraints (on their lower bounds); the order depends on the SujectToType
		 *  of each index. Only disabled constraints are added to index set of disabled constraints!
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_CONSTRAINT_FAILED */
		returnValue setupAllLower( );

		/** Initially adds all enabled numbers of new (i.e. not yet in the list) constraints to
		 *  to the index set of active constraints (on their upper bounds); the order depends on the SujectToType
		 *  of each index. Only disabled constraints are added to index set of disabled constraints!
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_CONSTRAINT_FAILED */
		returnValue setupAllUpper( );


		/** Moves index of a constraint from index list of active to that of inactive constraints.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_CONSTRAINT_FAILED */
		returnValue moveActiveToInactive(	int_t number			/**< Number of constraint to become inactive. */
											);

		/** Moves index of a constraint from index list of inactive to that of active constraints.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_CONSTRAINT_FAILED */
		returnValue moveInactiveToActive(	int_t number,			/**< Number of constraint to become active. */
											SubjectToStatus _status	/**< Status of constraint to become active. */
											);

		/** Flip fixed constraint.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_MOVING_CONSTRAINT_FAILED \n
					RET_INDEX_OUT_OF_BOUNDS */
		returnValue flipFixed( int_t number );


		/** Returns the number of constraints.
		 *	\return Number of constraints. */
		inline int_t getNC( ) const;

		/** Returns the number of implicit equality constraints.
		 *	\return Number of implicit equality constraints. */
		inline int_t getNEC( ) const;

		/** Returns the number of "real" inequality constraints.
		 *	\return Number of "real" inequality constraints. */
		inline int_t getNIC( ) const;

		/** Returns the number of unbounded constraints (i.e. without any bounds).
		 *	\return Number of unbounded constraints (i.e. without any bounds). */
		inline int_t getNUC( ) const;

		/** Returns the number of active constraints.
		 *	\return Number of active constraints. */
		inline int_t getNAC( ) const;

		/** Returns the number of inactive constraints.
		 *	\return Number of inactive constraints. */
		inline int_t getNIAC( ) const;


		/** Returns a pointer to active constraints index list.
		 *	\return Pointer to active constraints index list. */
		inline Indexlist* getActive( );

		/** Returns a pointer to inactive constraints index list.
		 *	\return Pointer to inactive constraints index list. */
		inline Indexlist* getInactive( );


		/** Shifts forward type and status of all constraints by a given
		 *  offset. This offset has to lie within the range [0,n/2] and has to
		 *  be an integer divisor of the total number of constraints n.
		 *  Type and status of the first \<offset\> constraints  is thrown away,
		 *  type and status of the last \<offset\> constraints is doubled,
		 *  e.g. for offset = 2: \n
		 *  shift( {c1,c2,c3,c4,c5,c6} ) = {c3,c4,c5,c6,c5,c6}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_INVALID_ARGUMENTS \n
		 			RET_SHIFTING_FAILED */
		virtual returnValue shift(	int_t offset	/**< Shift offset within the range [0,n/2] and integer divisor of n. */
									);

		/** Rotates forward type and status of all constraints by a given
		 *  offset. This offset has to lie within the range [0,n].
		 *  Example for offset = 2: \n
		 *  rotate( {c1,c2,c3,c4,c5,c6} ) = {c3,c4,c5,c6,c1,c2}
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INDEX_OUT_OF_BOUNDS \n
		 			RET_ROTATING_FAILED */
		virtual returnValue rotate(	int_t offset	/**< Rotation offset within the range [0,n]. */
									);


		/** Prints information on constraints object
		 *  (in particular, lists of inactive and active constraints.
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
		returnValue copy(	const Constraints& rhs	/**< Rhs object. */
							);


		/** Initially adds all numbers of new (i.e. not yet in the list) bounds to
		 *  to the index set corresponding to the desired status;
		 *  the order depends on the SujectToType of each index.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_SETUP_CONSTRAINT_FAILED */
		returnValue setupAll(	SubjectToStatus _status	/**< Desired initial status for all bounds. */
								);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		Indexlist active;		/**< Index list of active constraints. */
		Indexlist inactive;		/**< Index list of inactive constraints. */
};

END_NAMESPACE_QPOASES

#include <qpOASES/Constraints.ipp>

#endif	/* QPOASES_CONSTRAINTS_HPP */


/*
 *	end of file
 */
