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
 *	\file include/qpOASES/Bounds.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Implementation of inlined member functions of the Bounds class designed
 *	to manage working sets of bounds within a QProblem.
 */


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/

/*
 *	g e t N V
 */
inline int_t Bounds::getNV( ) const
{
 	return n;
}


/*
 *	g e t N F V
 */
inline int_t Bounds::getNFV( ) const
{
 	return getNumberOfType( ST_EQUALITY );
}


/*
 *	g e t N B V
 */
inline int_t Bounds::getNBV( ) const
{
 	return getNumberOfType( ST_BOUNDED );
}


/*
 *	g e t N U V
 */
inline int_t Bounds::getNUV( ) const
{
	return getNumberOfType( ST_UNBOUNDED );
}


/*
 *	g e t N F R
 */
inline int_t Bounds::getNFR( ) const
{
 	return freee.getLength( );
}


/*
 *	g e t N F X
 */
inline int_t Bounds::getNFX( ) const
{
	return fixed.getLength( );
}


/*
 *	g e t F r e e
 */
inline Indexlist* Bounds::getFree( )
{
	return &freee;
}


/*
 *	g e t F i x e d
 */
inline Indexlist* Bounds::getFixed( )
{
	return &fixed;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
