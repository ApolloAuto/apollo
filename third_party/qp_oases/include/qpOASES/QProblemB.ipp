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
 *	\file include/qpOASES/QProblemB.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Implementation of inlined member functions of the QProblemB class which
 *	is able to use the newly developed online active set strategy for
 *	parametric quadratic programming.
 */



BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/

/*
 *	g e t B o u n d s
 */
inline returnValue QProblemB::getBounds( Bounds& _bounds ) const
{
	int_t nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	_bounds = bounds;

	return SUCCESSFUL_RETURN;
}


/*
 *	g e t N V
 */
inline int_t QProblemB::getNV( ) const
{
	return bounds.getNV( );
}


/*
 *	g e t N F R
 */
inline int_t QProblemB::getNFR( ) const
{
	return bounds.getNFR( );
}


/*
 *	g e t N F X
 */
inline int_t QProblemB::getNFX( ) const
{
	return bounds.getNFX( );
}


/*
 *	g e t N F V
 */
inline int_t QProblemB::getNFV( ) const
{
	return bounds.getNFV( );
}


/*
 *	g e t S t a t u s
 */
inline QProblemStatus QProblemB::getStatus( ) const
{
	return status;
}


/*
 *	i s I n i t i a l i s e d
 */
inline BooleanType QProblemB::isInitialised( ) const
{
	if ( status == QPS_NOTINITIALISED )
		return BT_FALSE;
	else
		return BT_TRUE;
}


/*
 *	i s S o l v e d
 */
inline BooleanType QProblemB::isSolved( ) const
{
	if ( status == QPS_SOLVED )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *	i s I n f e a s i b l e
 */
inline BooleanType QProblemB::isInfeasible( ) const
{
	return infeasible;
}


/*
 *	i s U n b o u n d e d
 */
inline BooleanType QProblemB::isUnbounded( ) const
{
	return unbounded;
}


/*
 *	g e t H e s s i a n T y p e
 */
inline HessianType QProblemB::getHessianType( ) const
{
	return hessianType;
}


/*
 *	s e t H e s s i a n T y p e
 */
inline returnValue QProblemB::setHessianType( HessianType _hessianType )
{
	hessianType = _hessianType;
	return SUCCESSFUL_RETURN;
}


/*
 *	u s i n g R e g u l a r i s a t i o n
 */
inline BooleanType QProblemB::usingRegularisation( ) const
{
	if ( regVal > ZERO )
		return BT_TRUE;
	else
		return BT_FALSE;
}


/*
 *	g e t O p t i o n s
 */
inline Options QProblemB::getOptions( ) const
{
	return options;
}


/*
 *	s e t O p t i o n s
 */
inline returnValue QProblemB::setOptions(	const Options& _options
											)
{
	options = _options;
	options.ensureConsistency( );

	setPrintLevel( options.printLevel );

	return SUCCESSFUL_RETURN;
}


/*
 *	g e t P r i n t L e v e l
 */
inline PrintLevel QProblemB::getPrintLevel( ) const
{
	return options.printLevel;
}



/*
 *	g e t C o u n t
 */
inline uint_t QProblemB::getCount( ) const
{
	return count;
}


/*
 *	r e s e t C o u n t e r
 */
inline returnValue QProblemB::resetCounter( )
{
	count = 0;
	return SUCCESSFUL_RETURN;
}


/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	s e t H
 */
inline returnValue QProblemB::setH( SymmetricMatrix* H_new )
{
	if ( ( freeHessian == BT_TRUE ) && ( H != 0 ) )
	{
		delete H;
		H = 0;
	}

	H = H_new;
	freeHessian = BT_FALSE;
	
	return SUCCESSFUL_RETURN;
}


/*
 *	s e t H
 */
inline returnValue QProblemB::setH( const real_t* const H_new )
{
	int_t nV = getNV();
	SymDenseMat* dH;

	/* if null pointer is passed, Hessian is set to zero matrix
	 *                            (or stays identity matrix) */
	if ( H_new == 0 )
	{
		if ( hessianType == HST_IDENTITY )
			return SUCCESSFUL_RETURN;

		hessianType = HST_ZERO;

		if ( ( freeHessian == BT_TRUE ) && ( H != 0 ) )
			delete H;

		H = 0;
		freeHessian = BT_FALSE;
	}
	else
	{
		if ( ( freeHessian == BT_TRUE ) && ( H != 0 ) )
			delete H;

		H = dH = new SymDenseMat( nV, nV, nV, (real_t*) H_new );
		freeHessian = BT_TRUE;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t G
 */
inline returnValue QProblemB::setG( const real_t* const g_new )
{
	uint_t nV = (uint_t)getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	if ( g_new == 0 )
		return THROWERROR( RET_INVALID_ARGUMENTS );

	memcpy( g,g_new,nV*sizeof(real_t) );

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t L B
 */
inline returnValue QProblemB::setLB( const real_t* const lb_new )
{
	uint_t i;
	uint_t nV = (uint_t)getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	if ( lb_new != 0 )
	{
		memcpy( lb,lb_new,nV*sizeof(real_t) );
	}
	else
	{
		/* if no lower bounds are specified, set them to -infinity */
		for( i=0; i<nV; ++i )
			lb[i] = -INFTY;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t L B
 */
inline returnValue QProblemB::setLB( int_t number, real_t value )
{
	int_t nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	if ( ( number >= 0 ) && ( number < nV ) )
	{
		lb[number] = value;
		return SUCCESSFUL_RETURN;
	}
	else
	{
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );
	}
}


/*
 *	s e t U B
 */
inline returnValue QProblemB::setUB( const real_t* const ub_new )
{
	uint_t i;
	uint_t nV = (uint_t)getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	if ( ub_new != 0 )
	{
		memcpy( ub,ub_new,nV*sizeof(real_t) );
	}
	else
	{
		/* if no upper bounds are specified, set them to infinity */
		for( i=0; i<nV; ++i )
			ub[i] = INFTY;
	}

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t U B
 */
inline returnValue QProblemB::setUB( int_t number, real_t value )
{
	int_t nV = getNV( );

	if ( nV == 0 )
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	if ( ( number >= 0 ) && ( number < nV ) )
	{
		ub[number] = value;

		return SUCCESSFUL_RETURN;
	}
	else
	{
		return THROWERROR( RET_INDEX_OUT_OF_BOUNDS );
	}
}


/*
 *	c o m p u t e G i v e n s
 */
inline void QProblemB::computeGivens(	real_t xold, real_t yold, real_t& xnew, real_t& ynew,
										real_t& c, real_t& s
										) const
{
	real_t t, mu;

	if ( isZero( yold ) == BT_TRUE )
	{
		c = 1.0;
		s = 0.0;

		xnew = xold;
		ynew = yold;
	}
	else
	{
		mu = getAbs( xold );
		if ( getAbs( yold ) > mu )
			mu = getAbs( yold );

		t = mu * getSqrt( (xold/mu)*(xold/mu) + (yold/mu)*(yold/mu) );

		if ( xold < 0.0 )
		t = -t;

		c = xold/t;
		s = yold/t;
		xnew = t;
		ynew = 0.0;
	}

	return;
}


/*
 *	a p p l y G i v e n s
 */
inline void QProblemB::applyGivens(	real_t c, real_t s, real_t nu, real_t xold, real_t yold,
									real_t& xnew, real_t& ynew
									) const
{
	#ifdef __USE_THREE_MULTS_GIVENS__

	/* Givens plane rotation requiring only three multiplications,
	 * cf. Hammarling, S.: A note on modifications to the givens plane rotation.
	 * J. Inst. Maths Applics, 13:215-218, 1974. */
	xnew = xold*c + yold*s;
	ynew = (xnew+xold)*nu - yold;

	#else

	/* Usual Givens plane rotation requiring four multiplications. */
	xnew =  c*xold + s*yold;
	ynew = -s*xold + c*yold;

	#endif

	return;
}


/*
 * i s B l o c k i n g
 */
inline BooleanType QProblemB::isBlocking(	real_t num,
											real_t den,
											real_t epsNum,
											real_t epsDen,
											real_t& t
											) const
{
	if ( ( den >= epsDen ) && ( num >= epsNum ) )
	{
		if ( num < t*den )
			return BT_TRUE;
	}

	return BT_FALSE;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
