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
 *	\file include/qpOASES/MessageHandling.ipp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Implementation of inlined member functions of the MessageHandling class.
 */


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	g e t E r r o r V i s i b i l i t y S t a t u s
 */
inline VisibilityStatus MessageHandling::getErrorVisibilityStatus( ) const
{
 	return errorVisibility;
}


/*
 *	g e t W a r n i n g V i s i b i l i t y S t a t u s
 */
inline VisibilityStatus MessageHandling::getWarningVisibilityStatus( ) const
{
 	return warningVisibility;
}


/*
 *	g e t I n f o V i s i b i l i t y S t a t u s
 */
inline VisibilityStatus MessageHandling::getInfoVisibilityStatus( ) const
{
 	return infoVisibility;
}


/*
 *	g e t O u t p u t F i l e
 */
inline FILE* MessageHandling::getOutputFile( ) const
{
 	return outputFile;
}


/*
 *	g e t E r r o r C o u n t
 */
inline int_t MessageHandling::getErrorCount( ) const
{
 	return errorCount;
}


/*
 *	s e t E r r o r V i s i b i l i t y S t a t u s
 */
inline void MessageHandling::setErrorVisibilityStatus( VisibilityStatus _errorVisibility )
{
 	errorVisibility = _errorVisibility;
}


/*
 *	s e t W a r n i n g V i s i b i l i t y S t a t u s
 */
inline void MessageHandling::setWarningVisibilityStatus( VisibilityStatus _warningVisibility )
{
 	warningVisibility = _warningVisibility;
}


/*
 *	s e t I n f o V i s i b i l i t y S t a t u s
 */
inline void MessageHandling::setInfoVisibilityStatus( VisibilityStatus _infoVisibility )
{
 	infoVisibility = _infoVisibility;
}


/*
 *	s e t O u t p u t F i l e
 */
inline void MessageHandling::setOutputFile( FILE* _outputFile )
{
 	outputFile = _outputFile;
}


/*
 *	s e t E r r o r C o u n t
 */
inline returnValue MessageHandling::setErrorCount( int_t _errorCount )
{
	if ( _errorCount >= -1 )
	{
		errorCount = _errorCount;
		return SUCCESSFUL_RETURN;
	}
	else
		return RET_INVALID_ARGUMENTS;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
