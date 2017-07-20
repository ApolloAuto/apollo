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
 *	\file include/qpOASES/Constraints.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Declaration of inlined member functions of the Constraints class designed
 *	to manage working sets of constraints within a QProblem.
 */


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	g e t N C
 */
inline int_t Constraints::getNC( ) const
{
 	return n;
}


/*
 *	g e t N E C
 */
inline int_t Constraints::getNEC( ) const
{
	return getNumberOfType( ST_EQUALITY );
}


/*
 *	g e t N I C
 */
inline int_t Constraints::getNIC( ) const
{
 	return getNumberOfType( ST_BOUNDED );
}


/*
 *	g e t N U C
 */
inline int_t Constraints::getNUC( ) const
{
 	return getNumberOfType( ST_UNBOUNDED );
}


/*
 *	g e t N A C
 */
inline int_t Constraints::getNAC( ) const
{
 	return active.getLength( );
}


/*
 *	g e t N I A C
 */
inline int_t Constraints::getNIAC( ) const
{
	return inactive.getLength( );
}



/*
 *	g e t A c t i v e
 */
inline Indexlist* Constraints::getActive( )
{
	return &active;
}


/*
 *	g e t I n a c t i v e
 */
inline Indexlist* Constraints::getInactive( )
{
	return &inactive;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
