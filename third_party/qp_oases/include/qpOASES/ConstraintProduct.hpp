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
 *	\file include/qpOASES/ConstraintProduct.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2009-2017
 *
 *	Declaration of the ConstraintProduct class which allows to specify a
 *	user-defined function for evaluating the constraint product at the 
 *	current iterate to speed-up QP solution in case of a specially structured
 *	constraint matrix.
 */



#ifndef QPOASES_CONSTRAINT_PRODUCT_HPP
#define QPOASES_CONSTRAINT_PRODUCT_HPP


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Interface for specifying user-defined evaluations of constraint products.
 *
 *	A class which allows to specify a user-defined function for evaluating the 
 *	constraint product at the current iterate to speed-up QP solution in case 
 *	of a specially structured constraint matrix.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2009-2017
 */
class ConstraintProduct
{
	public:
		/** Default constructor. */
		ConstraintProduct( ) {};

		/** Copy constructor. */
		ConstraintProduct(	const ConstraintProduct &toCopy	/**< Rhs object. */
							) {};

		/** Destructor. */
		virtual ~ConstraintProduct( ) {};
		
		/** Assignment operator. */
		ConstraintProduct &operator=(	const ConstraintProduct &toCopy	/**< Rhs object. */
										)
		{
			return *this;
		}

		/** Evaluates the product of a given constraint with the current iterate.
		 *	This function needs to be implemented in a derived class for the 
		 *	user-defined constraint product function.
		 *	\return 0:         successful \n
					otherwise: not successful */
		virtual int_t operator() (	int_t constrIndex,			/**< Number of constraint to be evaluated. */
									const real_t* const x,		/**< Array containing current primal iterate. */
									real_t* const constrValue	/**< Output: Scalar value of the evaluated constraint. */
									) const = 0;
};

END_NAMESPACE_QPOASES


#endif	/* QPOASES_CONSTRAINT_PRODUCT_HPP */
