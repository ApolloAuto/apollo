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

#ifndef Patternist_AbstractDuration_H
#define Patternist_AbstractDuration_H

#include <QRegExp>

#include <private/qitem_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Base class for classes implementing durations.
     *
     * @see <a href="http://www.w3.org/TR/xmlschema-2/#duration">XML Schema Part
     * 2: Datatypes Second Edition, 3.2.6 duration</a>
     * @see <a href="http://www.w3.org/TR/xpath-datamodel/#dates-and-times">XQuery
     * 1.0 and XPath 2.0 Data Model (XDM), 3.3.2 Dates and Times</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_xdm
     * @todo Documentation is missing/incomplete
     */
    class AbstractDuration : public AtomicValue
    {
    public:
        typedef QExplicitlySharedDataPointer<AbstractDuration> Ptr;

        /**
         * @short The amount in milli seconds.
         */
        typedef qint64 Value;

        /**
         * @short Acts as a mapping table for AbstractDuration::create()
         * and describes where certain fields in a QRegExp pattern can be found
         * for a particular W3C XML Schema duration type.
         *
         * @author Frans Englich <frans.englich@nokia.com>
         * @ingroup Patternist_xdm
         */
        class CaptureTable
        {
        public:
            CaptureTable(const QRegExp &exp,
                         const qint8 yearP,
                         const qint8 monthP,
                         const qint8 dayP = -1,
                         const qint8 tDelimiterP = -1,
                         const qint8 hourP = -1,
                         const qint8 minutesP = -1,
                         const qint8 secondsP = -1,
                         const qint8 msecondsP = -1) : regExp(exp),
                                                       year(yearP),
                                                       month(monthP),
                                                       day(dayP),
                                                       tDelimiter(tDelimiterP),
                                                       hour(hourP),
                                                       minutes(minutesP),
                                                       seconds(secondsP),
                                                       mseconds(msecondsP)
            {
                Q_ASSERT(exp.isValid());
                Q_ASSERT(yearP == -1 || yearP == 2);
            }

            QRegExp regExp;
            const qint8 year;
            const qint8 month;
            const qint8 day;
            const qint8 tDelimiter;
            const qint8 hour;
            const qint8 minutes;
            const qint8 seconds;
            const qint8 mseconds;
        };

        /**
         * Determines whether this Duration is equal to @p other.
         *
         * @note Do not re-implement this function. It uses getters such as years() and
         * mseconds() for determining its truth value.
         */
        bool operator==(const AbstractDuration &other) const;

        virtual YearProperty years() const = 0;
        virtual MonthProperty months() const = 0;
        virtual DayCountProperty days() const = 0;
        virtual HourProperty hours() const = 0;
        virtual MinuteProperty minutes() const = 0;
        virtual SecondProperty seconds() const = 0;
        virtual MSecondProperty mseconds() const = 0;

        /**
         * @returns the value of this AbstractDuration. For example,
         * in the case of xs:yearMonthDuration, that is YearMonthDuration,
         * years times twelve plus the months is returned.
         */
        virtual Value value() const = 0;

        /**
         * A polymorphic factory function that returns instances of the
         * sub-class with the value @p val.
         */
        virtual Item fromValue(const Value val) const = 0;

        /**
         * Determines whether this AbstractDuration is positive. For example,
         * "P10H" is positive, while "-P10H" is not.
         *
         * @note Do not re-implement this function. Use the constructor, AbstractDuration(),
         * for changing the value.
         * @returns @c true if this AbstractDuration is positive, otherwise @c false.
         */
        bool isPositive() const;

    protected:

        AbstractDuration(const bool isPos);

        static QString serializeMSeconds(const MSecondProperty mseconds);
        static AtomicValue::Ptr create(const CaptureTable &captTable,
                                       const QString &lexical,
                                       bool *isPositive,
                                       YearProperty *years,
                                       MonthProperty *months,
                                       DayCountProperty *days,
                                       HourProperty *hours,
                                       MinuteProperty *minutes,
                                       SecondProperty *seconds,
                                       MSecondProperty *mseconds);
        const bool m_isPositive;
    };
}

QT_END_NAMESPACE

#endif
