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

#ifndef Patternist_Decimal_H
#define Patternist_Decimal_H

#include <private/qschemanumeric_p.h>

QT_BEGIN_NAMESPACE

/**
 * Defined in QtCore's qlocale.cpp.
 */
Q_CORE_EXPORT char *qdtoa(double d, int mode, int ndigits, int *decpt, int *sign, char **rve, char **resultp);

namespace QPatternist
{

    /**
     * @short Implements the value instance of the @c xs:decimal type.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_xdm
     * @todo Documentation is missing/incomplete
     */
    class Decimal : public Numeric
    {
    public:

        static Decimal::Ptr fromValue(const xsDecimal num);

        /**
         * Creates a Decimal from the lexical representation of @c xs:decimal stored in
         * @p strNumeric.
         *
         * A possible optimization is to create an Integer if the string ends
         * with ".0". But this is not conformant. For example, the user writes N.0
         * which according to the specification is an xs:decimal, but where the
         * expression is, is an xs:integer is required. That would pass with
         * such an optimization.
         */
        static AtomicValue::Ptr fromLexical(const QString &strNumeric);

        /**
         * Gets the Effective %Boolean Value of this number.
         *
         * @returns @c false if the number is 0 or @c NaN, otherwise @c true.
         */
        bool evaluateEBV(const QExplicitlySharedDataPointer<DynamicContext> &) const;

        virtual QString stringValue() const;

        /**
         * @returns always BuiltinTypes::xsDecimal
         */
        virtual ItemType::Ptr type() const;

        virtual xsDouble toDouble() const;
        virtual xsInteger toInteger() const;
        virtual xsFloat toFloat() const;
        virtual xsDecimal toDecimal() const;
        virtual qulonglong toUnsignedInteger() const;

        virtual Numeric::Ptr round() const;
        virtual Numeric::Ptr roundHalfToEven(const xsInteger scale) const;
        virtual Numeric::Ptr floor() const;
        virtual Numeric::Ptr ceiling() const;
        virtual Numeric::Ptr abs() const;

        /**
         * @returns always @c false, xs:decimal doesn't have
         * not-a-number in its value space.
         */
        virtual bool isNaN() const;

        /**
         * @returns always @c false, xs:decimal doesn't have
         * infinity in its value space.
         */
        virtual bool isInf() const;

        virtual Item toNegated() const;

        /**
         * Converts @p value into a canonical string representation for @c xs:decimal. This
         * function is used internally by various classes. Users probably wants to call
         * stringValue() which in turn calls this function.
         */
        static QString toString(const xsDecimal value);

        virtual bool isSigned() const;

    protected:

        Decimal(const xsDecimal num);

    private:
        const xsDecimal m_value;
    };
}

QT_END_NAMESPACE

#endif
