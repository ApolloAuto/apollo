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

#ifndef Patternist_UntypedAtomicConverter_H
#define Patternist_UntypedAtomicConverter_H

#include <private/qitem_p.h>
#include <private/qsinglecontainer_p.h>
#include <private/qcastingplatform_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Casts every item in a sequence obtained from
     * evaluating an Expression, to a requested atomic type.
     *
     * The atomic values it casts from are instances of xs:untypedAtomic(hence
     * the name). Typically, the items are from an Atomizer. UntypedAtomicConverter
     * implements the automatic conversion which typically is activated when XPath
     * is handling untyped data.
     *
     * @see <a href="http://www.w3.org/TR/xpath20/#id-function-calls">XML Path
     * Language (XPath) 2.0, 3.1.5 Function Calls, in particular the
     * Function Conversion Rules</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class UntypedAtomicConverter : public SingleContainer,
                                   public CastingPlatform<UntypedAtomicConverter, true>
    {
    public:
        UntypedAtomicConverter(const Expression::Ptr &operand,
                               const ItemType::Ptr &reqType,
                               const ReportContext::ErrorCode code = ReportContext::FORG0001);

        virtual Item evaluateSingleton(const DynamicContext::Ptr &) const;
        virtual Item::Iterator::Ptr evaluateSequence(const DynamicContext::Ptr &) const;

        virtual SequenceType::Ptr staticType() const;
        virtual SequenceType::List expectedOperandTypes() const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        /**
         * Overridden to call CastingPlatform::typeCheck()
         */
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        inline Item mapToItem(const Item &item,
                              const DynamicContext::Ptr &context) const;

        inline ItemType::Ptr targetType() const
        {
            return m_reqType;
        }

        virtual const SourceLocationReflection *actualReflection() const;

    private:
        typedef QExplicitlySharedDataPointer<const UntypedAtomicConverter> ConstPtr;
        const ItemType::Ptr m_reqType;
    };

    Item UntypedAtomicConverter::mapToItem(const Item &item, const DynamicContext::Ptr &context) const
    {
        return cast(item, context);
    }
}

QT_END_NAMESPACE

#endif
