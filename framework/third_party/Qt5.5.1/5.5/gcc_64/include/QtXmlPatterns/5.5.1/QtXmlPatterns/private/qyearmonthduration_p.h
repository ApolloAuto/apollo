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

#ifndef Patternist_YearMonthDuration_H
#define Patternist_YearMonthDuration_H

#include <private/qabstractduration_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Implements the value instance of the @c xs:yearMonthDuration type.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_xdm
     */
    class YearMonthDuration : public AbstractDuration
    {
    public:
        typedef AtomicValue::Ptr Ptr;

        /**
         * Creates an instance from the lexical representation @p string.
         */
        static YearMonthDuration::Ptr fromLexical(const QString &string);

        static YearMonthDuration::Ptr fromComponents(const bool isPositive,
                                                     const YearProperty years,
                                                     const MonthProperty months);

        virtual ItemType::Ptr type() const;
        virtual QString stringValue() const;

        /**
         * @returns the value of this @c xs:yearMonthDuration in months.
         * @see <a href="http://www.w3.org/TR/xpath-functions/#dt-yearMonthDuration">XQuery 1.0
         * and XPath 2.0 Functions and Operators, 10.3.2.2 Calculating the value of a
         * xs:dayTimeDuration from the lexical representation</a>
         */
        virtual Value value() const;

        /**
         * If @p val is zero, is CommonValues::YearMonthDurationZero returned.
         */
        virtual Item fromValue(const Value val) const;

        /**
         * @returns the years component. Always positive.
         */
        virtual YearProperty years() const;

        /**
         * @returns the months component. Always positive.
         */
        virtual MonthProperty months() const;

        /**
         * @returns always 0.
         */
        virtual DayCountProperty days() const;

        /**
         * @returns always 0.
         */
        virtual HourProperty hours() const;

        /**
         * @returns always 0.
         */
        virtual MinuteProperty minutes() const;

        /**
         * @returns always 0.
         */
        virtual SecondProperty seconds() const;

        /**
         * @returns always 0.
         */
        virtual MSecondProperty mseconds() const;

    protected:
        friend class CommonValues;

        YearMonthDuration(const bool isPositive,
                          const YearProperty years,
                          const MonthProperty months);

    private:
        const YearProperty  m_years;
        const MonthProperty m_months;
    };
}

QT_END_NAMESPACE

#endif
