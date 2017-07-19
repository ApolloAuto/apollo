/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2013 by Hans Joachim Ferreau, Andreas Potschka,
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
 *	\file include/qpOASES/SQProblemSchur.ipp
 *	\author Andreas Waechter, Dennis Janka
 *	\version 3.2
 *	\date 2012-2017
 *
 *	Implementation of inlined member functions of the SQProblemSchur class which
 *	is able to use the newly developed online active set strategy for
 *	parametric quadratic programming.
 */


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/

/*
 *	g e t N u m F a c t o r i z a t i o n s
 */
inline int_t SQProblemSchur::getNumFactorizations( ) const
{
	return numFactorizations;
}

END_NAMESPACE_QPOASES


/*
 *	end of file
 */
