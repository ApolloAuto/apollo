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

#ifndef Patternist_Literal_H
#define Patternist_Literal_H

#include <private/qemptycontainer_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Houses an AtomicValue, making it available as an Expression.
     *
     * This is not only literals that can be created via the XQuery syntax(strings and numbers), but
     * all other atomic values, such as <tt>xs:date</tt> or <tt>xs:time</tt>.
     *
     * @see <a href="http://www.w3.org/TR/xquery/#id-literals">XQuery 1.0: An XML Query Language,
     * 3.1.1 Literals</a>
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class Literal : public EmptyContainer
    {
    public:
        /**
         * Creates a Literal that represents @p item.
         *
         * @param item must be non-null and cannot be a QXmlNodeModelIndex.
         */
        Literal(const Item &item);

        virtual Item evaluateSingleton(const DynamicContext::Ptr &context) const;
        virtual bool evaluateEBV(const DynamicContext::Ptr &context) const;
        void evaluateToSequenceReceiver(const DynamicContext::Ptr &context) const;

        virtual SequenceType::Ptr staticType() const;
        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;
        virtual ID id() const;
        virtual QString description() const;

        /**
         * @returns Expression::DisableElimination and Expression::IsEvaluated
         */
        virtual Properties properties() const;

        /**
         * Declaring the return value of this function a const reference, leads
         * to crashes in patternistview, for a to me unknown reason.
         */
        inline Item item() const
        {
            return m_item;
        }

    private:
        const Item m_item;
    };

    /**
     * @short Creates a Literal that wraps @p item, and returns it.
     *
     * This simplifies code. Instead of writing:
     *
     * @code
     * Expression::Ptr(new Literal(item));
     * @endcode
     *
     * One can write:
     *
     * @code
     * wrapLiteral(item);
     * @endcode
     *
     * This function is not declared static, because it breaks the build on
     * at least aix-xlc-64.
     *
     * @relates Literal
     */
    inline Expression::Ptr wrapLiteral(const Item &item,
                                       const StaticContext::Ptr &context,
                                       const SourceLocationReflection *const r)
    {
        Q_ASSERT(item);

        const Expression::Ptr retval(new Literal(item));
        context->addLocation(retval.data(), context->locationFor(r));

        return retval;
    }
}

QT_END_NAMESPACE

#endif
