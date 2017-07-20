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
 *	\file include/qpOASES/SubjectTo.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Implementation of the inlined member functions of the SubjectTo class
 *	designed to manage working sets of constraints and bounds within a QProblem.
 */


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	g e t N u m b e r O f T y p e
 */
inline int_t SubjectTo::getNumberOfType( SubjectToType _type ) const
{
	int_t i;
	int_t numberOfType = 0;

	if ( type != 0 )
	{
		for( i=0; i<n; ++i )
			if ( type[i] == _type )
				++numberOfType;
	}

	return numberOfType;
}


/*
 *	g e t T y p e
 */
inline SubjectToType SubjectTo::getType( int_t i ) const
{
	if ( ( i >= 0 ) && ( i < n ) )
		return type[i];

	return ST_UNKNOWN;
}


/*
 *	g e t S t a t u s
 */
inline SubjectToStatus SubjectTo::getStatus( int_t i ) const
{
	if ( ( i >= 0 ) && ( i < n ) )
		return status[i];

	return ST_UNDEFINED;
}


/*
 *	s e t T y p e
 */
inline returnValue SubjectTo::setType( int_t i, SubjectToType value )
{
	if ( ( i >= 0 ) && ( i < n ) )
	{
		type[i] = value;
		return SUCCESSFUL_RETURN;
	}
	else
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );
}


/*
 *	s e t S t a t u s
 */
inline returnValue SubjectTo::setStatus( int_t i, SubjectToStatus value )
{
	if ( ( i >= 0 ) && ( i < n ) )
	{
		status[i] = value;
		return SUCCESSFUL_RETURN;
	}
	else
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );
}


/*
 *	s e t N o L o w e r
 */
inline void SubjectTo::setNoLower( BooleanType _status )
{
	noLower = _status;
}


/*
 *	s e t N o U p p e r
 */
inline void SubjectTo::setNoUpper( BooleanType _status )
{
	noUpper = _status;
}


/*
 *	h a s N o L o w e r
 */
inline BooleanType SubjectTo::hasNoLower( ) const
{
	return noLower;
}


/*
 *	h a s N o U p p p e r
 */
inline BooleanType SubjectTo::hasNoUpper( ) const
{
	return noUpper;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
