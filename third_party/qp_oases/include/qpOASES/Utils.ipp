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
 *	\file include/qpOASES/Utils.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Implementation of some inlined utilities for working with the different QProblem classes.
 */


#include <math.h>


BEGIN_NAMESPACE_QPOASES


/*
 *   i s E q u a l
 */
inline BooleanType isEqual(	real_t x,
							real_t y,
							real_t TOL
							)
{
    if ( getAbs(x-y) <= TOL )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *   i s Z e r o
 */
inline BooleanType isZero(	real_t x,
							real_t TOL
							)
{
    if ( getAbs(x) <= TOL )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *   g e t S i g n
 */
inline real_t getSign(	real_t arg
						)
{
	if ( arg >= 0.0 )
		return 1.0;
	else
		return -1.0;
}



/*
 *   g e t M a x
 */
inline int_t getMax(	int_t x,
						int_t y
						)
{
    return (y<x) ? x : y;
}


/*
 *   g e t M i n
 */
inline int_t getMin(	int_t x,
						int_t y
						)
{
    return (y>x) ? x : y;
}



/*
 *   g e t M a x
 */
inline real_t getMax(	real_t x,
						real_t y
						)
{
	#ifdef __NO_FMATH__
    return (y<x) ? x : y;
	#else
	return (y<x) ? x : y;
	//return fmax(x,y); /* seems to be slower */
	#endif
}


/*
 *   g e t M i n
 */
inline real_t getMin(	real_t x,
						real_t y
						)
{
	#ifdef __NO_FMATH__
    return (y>x) ? x : y;
	#else
	return (y>x) ? x : y;
	//return fmin(x,y); /* seems to be slower */
	#endif
}


/*
 *   g e t A b s
 */
inline real_t getAbs(	real_t x
						)
{
	#ifdef __NO_FMATH__
	return (x>=0.0) ? x : -x;
	#else
	return fabs(x);
	#endif
}


/*
 *   g e t S q r t
 */
inline real_t getSqrt(	real_t x
						)
{
	#ifdef __NO_FMATH__
	return sqrt(x); /* put your custom sqrt-replacement here */
	#else
	return sqrt(x);
	#endif
}



END_NAMESPACE_QPOASES


/*
 *	end of file
 */
