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

#ifndef Patternist_AbstractFloat_H
#define Patternist_AbstractFloat_H

#include <math.h>

#include <qnumeric.h>

#include <private/qcommonvalues_p.h>
#include <private/qdecimal_p.h>
#include <private/qschemanumeric_p.h>
#include <private/qvalidationerror_p.h>
#include <private/qbuiltintypes_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Base template class for Float and Double classes.
     *
     * @author Vincent Ricard <magic@magicninja.org>
     * @ingroup Patternist_xdm
     */
    template <const bool isDouble>
    class AbstractFloat : public Numeric
    {
    public:
        static Numeric::Ptr fromValue(const xsDouble num);
        static AtomicValue::Ptr fromLexical(const QString &strNumeric);

        /**
         * @todo more extensive docs.
         *
         * Performs floating point comparison.
         *
         * @returns @c true if @p a and @p are equal, otherwise @c false.
         */
        static bool isEqual(const xsDouble a, const xsDouble b);

        /**
         * Determines the Effective %Boolean Value of this number.
         *
         * @returns @c false if the number is 0 or @c NaN, otherwise @c true.
         */
        bool evaluateEBV(const QExplicitlySharedDataPointer<DynamicContext> &) const;

        /**
         * Returns this AbstractFloat represented as an @c xs:string.
         *
         * @note In the XPath/XQuery languages, converting @c xs:double and @c xs:float
         * to @c xs:string is not specified in XML Schema 1.0 Part 2: Datatypes Second Edition,
         * but in XQuery 1.0 and XPath 2.0 Functions and Operators. This will change with W3C XML
         * Schema 1.1
         *
         * @see <a href="http://www.w3.org/TR/xpath-functions/#casting-to-string">XQuery 1.0
         * and XPath 2.0 Functions and Operators, 17.1.2 Casting to xs:string and xdt:untypedAtomic</a>
         */
        virtual QString stringValue() const;

        virtual xsDouble toDouble() const;
        virtual xsInteger toInteger() const;
        virtual xsFloat toFloat() const;
        virtual xsDecimal toDecimal() const;

        virtual Numeric::Ptr round() const;
        virtual Numeric::Ptr roundHalfToEven(const xsInteger scale) const;
        virtual Numeric::Ptr floor() const;
        virtual Numeric::Ptr ceiling() const;
        virtual Numeric::Ptr abs() const;

        virtual bool isNaN() const;
        virtual bool isInf() const;

        virtual ItemType::Ptr type() const;
        virtual Item toNegated() const;
        virtual qulonglong toUnsignedInteger() const;

        virtual bool isSigned() const;
    protected:
        AbstractFloat(const xsDouble num);

    private:
        /**
         * From the Open Group's man page: "The signbit() macro shall return a
         * non-zero value if and only if the sign of its argument value is
         * negative."
         *
         * MS Windows doesn't have std::signbit() so here's
         * a reinvention of that function.
         */
        static inline int internalSignbit(const xsDouble v);
        inline bool isZero() const;

        const xsDouble m_value;
    };

    template <const bool isDouble>
    Numeric::Ptr createFloat(const xsDouble num);

#include "qabstractfloat_tpl_p.h"

    /**
     * @short An instantiation of AbsbstractFloat suitable for @c xs:double.
     *
     * @ingroup Patternist_xdm
     */
    typedef AbstractFloat<true> Double;

    /**
     * @short An instantiation of AbstractFloat suitable for @c xs:float.
     *
     * @ingroup Patternist_xdm
     */
    typedef AbstractFloat<false> Float;
}

QT_END_NAMESPACE

#endif
