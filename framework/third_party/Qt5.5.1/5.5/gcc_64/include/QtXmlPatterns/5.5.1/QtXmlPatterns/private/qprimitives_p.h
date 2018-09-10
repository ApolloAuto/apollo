/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

#ifndef Patternist_Primitives_H
#define Patternist_Primitives_H

#include <QtGlobal>
#include <QtCore/QHash>
#include <QtCore/QUrl>

/**
 * @file
 * @short Contains enumerators and typedefs applying
 * for Patternist on a global scale, as well as central documentation.
 */

/**
 * @short Contains Patternist, an XPath 2.0, XQuery 1.0 and XSL-T 2.0 implementation.
 *
 * @author Frans Englich <frans.englich@nokia.com>
 */
QT_BEGIN_NAMESPACE

class QString;

/**
 * @short The namespace for the internal API of Qt XML Patterns
 * @internal
 */
namespace QPatternist
{
    /**
     * @defgroup Patternist_cppWXSTypes C++ Primitives for W3C XML Schema Number Types
     *
     * The implementations of W3C XML Schema's(WXS) number types, more specifically
     * their value spaces, must in the end be represented by primitive C++ types.
     * In addition, there is an extensive range of functions and classes that in
     * different ways deals with data that will end up as instances of the WXS
     * types. For this reason, a set of typedefs for these primitives exists, that
     * are used throughout the API. This ensures consistency, reduces the amount
     * of conversions, and potentially precision loss in conversions.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */

    /**
     * This is the native C++ scalar type holding the value space
     * for atomic values of type xs:double. Taking this type, xsDouble,
     * as parameter, is the most efficient way to integrate with xs:double.
     *
     * @ingroup Patternist_cppWXSTypes
     */
    typedef qreal xsDouble;

    /**
     * This is the native C++ scalar type holding the value space
     * for atomic values of type xs:float. Taking this type, xsFloat,
     * as parameter, is the most efficient way to integrate with xs:float.
     *
     * @ingroup Patternist_cppWXSTypes
     */
    typedef xsDouble xsFloat;

    /**
     * This is the native C++ scalar type holding the value space
     * for atomic values of type xs:decimal. Taking this type, xsDecimal,
     * as parameter, is the most efficient way to integrate with xs:decimal.
     *
     * @ingroup Patternist_cppWXSTypes
     */
    typedef xsDouble xsDecimal;

    /**
     * This is the native C++ scalar type holding the value space
     * for atomic values of type xs:integer. Taking this type, xsInteger,
     * as parameter, is the most efficient way to integrate with xs:integer.
     *
     * @ingroup Patternist_cppWXSTypes
     */
    typedef qint64 xsInteger;

    /**
     * This is the native C++ scalar type holding the value space
     * for atomic values of type xs:integer. Taking this type, xsInteger,
     * as parameter, is the most efficient way to integrate with xs:integer.
     *
     * @ingroup Patternist_cppWXSTypes
     */
    typedef qint32 VariableSlotID;

    typedef qint32  DayCountProperty;
    typedef qint32  HourCountProperty;
    typedef qint32  MinuteCountProperty;
    typedef qint32  MonthCountProperty;
    typedef qint32  SecondCountProperty;
    typedef qint64  MSecondCountProperty;
    typedef qint32  SecondProperty;
    typedef qint32  YearProperty;
    typedef qint8   DayProperty;
    typedef qint8   HourProperty;
    typedef qint8   MinuteProperty;
    typedef qint8   MonthProperty;

    /**
     * Milliseconds. 1 equals 0.001 SecondProperty.
     */
    typedef qint16  MSecondProperty;

    /**
     * The hour property of a zone offset. For example, -13 in the
     * zone offset "-13:08".
     */
    typedef qint8   ZOHourProperty;

    /**
     * The minute property of a zone offset. For example, -08 in the
     * zone offset "-13:08".
     */
    typedef qint8   ZOMinuteProperty;

    /**
     * The full zone offset in minutes.
     */
    typedef qint32  ZOTotal;

    typedef xsDouble PatternPriority;

    /**
     * Signifies the import precedence of a template. For instance, the first
     * stylesheet module has 1, the first import 2, and so forth. Smaller means
     * higher import precedence. 0 is reserved for builtin templates.
     */
    typedef int ImportPrecedence;

    /**
     * @short Similar to Qt::escape(), but also escapes apostrophes and quotes,
     * such that the result is suitable as attribute content too.
     *
     * Since Qt::escape() is in QtGui, using it creates a dependency on that
     * library. This function does not.
     *
     * The implementation resides in qpatternistlocale.cpp.
     *
     * @see Qt::escape()
     */
    QString Q_AUTOTEST_EXPORT escape(const QString &input);
}

QT_END_NAMESPACE

#endif
