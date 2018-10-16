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

#ifndef Patternist_ComparisonFactory_H
#define Patternist_ComparisonFactory_H

#include <private/qatomiccomparator_p.h>
#include <private/qderivedstring_p.h>
#include <private/qitem_p.h>
#include <private/qreportcontext_p.h>
#include <private/qschematype_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Provides compare(), which is a high-level helper function for
     * comparing atomic values.
     *
     * This class wraps the helper class ComparisonPlatform with a more specific,
     * high-level API.
     *
     * @see ComparisonPlatform
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_schema
     */
    class ComparisonFactory
    {
    public:
        /**
         * @short Returns the result of evaluating operator @p op applied to the atomic
         * values @p operand1 and @p operand2.
         *
         * The caller guarantees that both values are of type @p type.
         *
         * ComparisonFactory does not take ownership of @p sourceLocationReflection.
         */
        static bool compare(const AtomicValue::Ptr &operand1,
                            const AtomicComparator::Operator op,
                            const AtomicValue::Ptr &operand2,
                            const SchemaType::Ptr &type,
                            const ReportContext::Ptr &context,
                            const SourceLocationReflection *const sourceLocationReflection);

        /**
         * @short Returns the result of evaluating operator @p op applied to the atomic
         * values @p operand1 and @p operand2.
         *
         * In opposite to compare() it converts the operands from string type
         * to @p type and compares these constructed types.
         *
         * The caller guarantees that both values are of type @p type.
         *
         * ComparisonFactory does not take ownership of @p sourceLocationReflection.
         */
        static bool constructAndCompare(const DerivedString<TypeString>::Ptr &operand1,
                                        const AtomicComparator::Operator op,
                                        const DerivedString<TypeString>::Ptr &operand2,
                                        const SchemaType::Ptr &type,
                                        const ReportContext::Ptr &context,
                                        const SourceLocationReflection *const sourceLocationReflection);

    private:
        Q_DISABLE_COPY(ComparisonFactory)
    };
}

QT_END_NAMESPACE

#endif
