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

#ifndef Patternist_VariableDeclaration_H
#define Patternist_VariableDeclaration_H

#include <QSharedData>

#include <private/qexpression_p.h>
#include <private/qpatternistlocale_p.h>
#include <private/qvariablereference_p.h>

QT_BEGIN_NAMESPACE

template<typename T> class QStack;

namespace QPatternist
{
    /**
     * @short Represents a declared variable. Only used at
     * the compilation stage.
     *
     * @see FunctionArgument
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class VariableDeclaration : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<VariableDeclaration> Ptr;
        typedef QStack<VariableDeclaration::Ptr> Stack;
        typedef QList<VariableDeclaration::Ptr> List;

        /**
         * @short The key is the variable name.
         */
        typedef QHash<QXmlName, VariableDeclaration::Ptr> Hash;

        enum Type
        {
            RangeVariable,
            ExpressionVariable,
            FunctionArgument,
            PositionalVariable,
            TemplateParameter,

            /**
             * A global variable is always an external variable, but it is
             * cached differently.
             *
             * @see DynamicContext::globalItemCacheCell()
             */
            GlobalVariable,

            /**
             * External variables doesn't use slots, that's a big difference
             * compared to the other types.
             */
            ExternalVariable
        };

        /**
         * Creates a VariableDeclaration.
         *
         * @p sourceExpr and @p seqType may be @c null.
         */
        VariableDeclaration(const QXmlName n,
                            const VariableSlotID varSlot,
                            const Type t,
                            const SequenceType::Ptr &seqType) : name(n)
                                                              , slot(varSlot)
                                                              , type(t)
                                                              , sequenceType(seqType)
                                                              , canSourceRewrite(true)
        {
            Q_ASSERT(!name.isNull());
            Q_ASSERT(t == ExternalVariable || t == TemplateParameter || varSlot > -1);
        }

        inline bool isUsed() const
        {
            return !references.isEmpty();
        }

        inline const Expression::Ptr &expression() const
        {
            return m_expression;
        }

        inline void setExpression(const Expression::Ptr &expr)
        {
            m_expression = expr;
        }

        /**
         * @short Returns how many times this variable is used.
         */
        inline bool usedByMany() const
        {
            return references.count() > 1;
        }

        /**
         * @short Returns @c true if @p list contains @p lookup.
         */
        static bool contains(const VariableDeclaration::List &list,
                             const QXmlName &lookup);

        const QXmlName                  name;
        const VariableSlotID            slot;
        const Type                      type;

        /**
         * The declared type of the variable. What the value might be, depends
         * on the context which VariableDeclaration is used in. Note that
         * sequenceType is hence not in anyway obligated to the type of
         * expression().
         */
        const SequenceType::Ptr         sequenceType;
        VariableReference::List         references;

        /**
         * @short Whether a reference can rewrite itself to expression().
         *
         * The default value is @c true.
         */
        bool canSourceRewrite;

    private:
        Expression::Ptr                 m_expression;
        Q_DISABLE_COPY(VariableDeclaration)
    };

    /**
     * @short Formats @p var appropriately for display.
     *
     * @relates VariableDeclaration
     */
    static inline QString formatKeyword(const VariableDeclaration::Ptr &var,
                                        const NamePool::Ptr &np)
    {
        Q_ASSERT(var);
        Q_ASSERT(np);
        return formatKeyword(np->displayName(var->name));
    }

}

QT_END_NAMESPACE

#endif
