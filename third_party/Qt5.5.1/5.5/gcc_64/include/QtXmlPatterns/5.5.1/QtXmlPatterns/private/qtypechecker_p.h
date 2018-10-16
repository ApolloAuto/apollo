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

#ifndef Patternist_TypeChecker_H
#define Patternist_TypeChecker_H

#include <private/qstaticcontext_p.h>
#include <private/qexpression_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Contains functions that applies Function Conversion Rules and other
     * kinds of compile-time type checking tasks.
     *
     * @ingroup Patternist_types
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class TypeChecker
    {
    public:
        enum Option
        {
            /**
             * @short When set, the function conversion rules are applied.
             *
             * For instance, this is type promotion and conversions from @c
             * xs:untypedAtomic to @c xs:date. This is done for function calls,
             * but not when binding an expression to a variable.
             */
            AutomaticallyConvert = 1,

            /**
             * @short Whether the focus should be checked or not.
             *
             * Sometimes the focus is unknown at the time
             * applyFunctionConversion() is called, and therefore it is
             * of interest to post pone the check to later on.
             */
            CheckFocus = 2,

            /**
             * When applyFunctionConversion() is passed AutomaticallyConvert
             * and promotion is required, such as from @c xs:integer to
             * @c xs:float, there will be no conversion performed, with the
             * assumption that the receiver will call Numeric::toFloat() or
             * similar.
             *
             * However, when GeneratePromotion is set, code will be generated
             * that performs this conversion regardless of what any receiver
             * do.
             *
             * This is useful in the case where one Expression only pipes the
             * result of another. The only known case of that as of this
             * writing is when UserFunctionCallsite evaluates its body.
             */
            GeneratePromotion
        };
        typedef QFlags<Option> Options;

        /**
         * @short Builds a pipeline of artificial AST nodes that ensures @p operand
         * conforms to the type @p reqType by applying the Function
         * Conversion Rules.
         *
         * This new Expression is returned, or, if no conversions were necessary,
         * @p operand as it is.
         *
         * applyFunctionConversion() also performs various checks, such as if
         * @p operand needs the focus and that the focus is defined in the
         * @p context. These checks are largely guided by @p operand's
         * Expression::properties().
         *
         * @see <a href="http://www.w3.org/TR/xpath20/\#id-function-calls">XML Path
         * Language (XPath) 2.0, 3.1.5 Function Calls</a>, more specifically the
         * Function Conversion Rules
         */
        static Expression::Ptr
        applyFunctionConversion(const Expression::Ptr &operand,
                                const SequenceType::Ptr &reqType,
                                const StaticContext::Ptr &context,
                                const ReportContext::ErrorCode code = ReportContext::XPTY0004,
                                const Options = Options(AutomaticallyConvert | CheckFocus));
    private:

        static inline Expression::Ptr typeCheck(Expression *const op,
                                                const StaticContext::Ptr &context,
                                                const SequenceType::Ptr &reqType);
        /**
         * @short Implements the type checking and promotion part of the Function Conversion Rules.
         */
        static Expression::Ptr verifyType(const Expression::Ptr &operand,
                                          const SequenceType::Ptr &reqSeqType,
                                          const StaticContext::Ptr &context,
                                          const ReportContext::ErrorCode code,
                                          const Options options);

        /**
         * Determines whether type promotion is possible from one type to another. False
         * is returned when a promotion is not possible or if a promotion is not needed(as when
         * the types are identical), since that can be considered to not be type promotion.
         *
         * @returns @c true if @p fromType can be promoted to @p toType.
         * @see <a href="http://www.w3.org/TR/xpath20/#promotion">XML Path Language
         * (XPath) 2.0, B.1 Type Promotion</a>
         */
        static bool promotionPossible(const ItemType::Ptr &fromType,
                                      const ItemType::Ptr &toType,
                                      const StaticContext::Ptr &context);

        /**
         * @short Centralizes a message-string to reduce work for translators
         * and increase consistency.
         */
        static inline QString wrongType(const NamePool::Ptr &np,
                                        const ItemType::Ptr &reqType,
                                        const ItemType::Ptr &opType);

        /**
         * No implementation is provided for this constructor. This class
         * is not supposed to be instantiated.
         */
        inline TypeChecker();

        Q_DISABLE_COPY(TypeChecker)
    };
}

QT_END_NAMESPACE

#endif
