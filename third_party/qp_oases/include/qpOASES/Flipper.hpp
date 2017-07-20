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
 *	\file include/qpOASES/Flipper.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of the Options class designed to manage user-specified
 *	options for solving a QProblem.
 */


#ifndef QPOASES_FLIPPER_HPP
#define QPOASES_FLIPPER_HPP


#include <qpOASES/Bounds.hpp>
#include <qpOASES/Constraints.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Auxiliary class for storing a copy of the current matrix factorisations.
 *
 *	This auxiliary class stores a copy of the current matrix factorisations. It
 *	is used by the classe QProblemB and QProblem in case flipping bounds are enabled.
 *
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 */
class Flipper
{
	friend class QProblemB;
	friend class QProblem;

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
		/** Default constructor. */
		Flipper( );

		/** Constructor which takes the number of bounds and constraints. */
		Flipper(	uint_t _nV,			/**< Number of bounds. */
					uint_t _nC = 0		/**< Number of constraints. */
					);

		/** Copy constructor (deep copy). */
		Flipper(	const Flipper& rhs			/**< Rhs object. */
					);

		/** Destructor. */
		~Flipper( );

		/** Assignment operator (deep copy). */
		Flipper& operator=(	const Flipper& rhs	/**< Rhs object. */
							);


		/** Initialises object with given number of bounds and constraints.
		 *	\return SUCCESSFUL_RETURN \n
		 			RET_INVALID_ARGUMENTS */
		returnValue init(	uint_t _nV = 0,		/**< Number of bounds. */
							uint_t _nC = 0		/**< Number of constraints. */
							);


		/** Copies current values to non-null arguments (assumed to be allocated with consistent size).
		 *	\return SUCCESSFUL_RETURN */
		returnValue get(	Bounds* const _bounds,					/**< Pointer to new bounds. */
							real_t* const R,						/**< New matrix R. */
							Constraints* const _constraints = 0,	/**< Pointer to new constraints. */
							real_t* const _Q = 0,					/**< New matrix Q. */
							real_t* const _T = 0					/**< New matrix T. */
							) const;

		/** Assigns new values to non-null arguments.
		 *	\return SUCCESSFUL_RETURN */
		returnValue set(	const Bounds* const _bounds,				/**< Pointer to new bounds. */
							const real_t* const _R,						/**< New matrix R. */
							const Constraints* const _constraints = 0,	/**< Pointer to new constraints. */
							const real_t* const _Q = 0,					/**< New matrix Q. */
							const real_t* const _T = 0					/**< New matrix T. */
							);


	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:
		/** Frees all allocated memory.
		 *  \return SUCCESSFUL_RETURN */
		returnValue clear( );
		
		/** Copies all members from given rhs object.
		 *  \return SUCCESSFUL_RETURN */
		returnValue copy(	const Flipper& rhs	/**< Rhs object. */
							);

		/** Returns dimension of matrix T.
		 *  \return Dimension of matrix T. */
		uint_t getDimT( ) const;


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:
		uint_t nV;						/**< Number of variables. */
		uint_t nC;						/**< Number of constraints. */

		Bounds      bounds;				/**< Data structure for problem's bounds. */
		Constraints constraints;		/**< Data structure for problem's constraints. */

		real_t* R;						/**< Cholesky factor of H (i.e. H = R^T*R). */
		real_t* Q;						/**< Orthonormal quadratic matrix, A = [0 T]*Q'. */
		real_t* T;						/**< Reverse triangular matrix, A = [0 T]*Q'. */
};


END_NAMESPACE_QPOASES


#endif	/* QPOASES_FLIPPER_HPP */


/*
 *	end of file
 */
