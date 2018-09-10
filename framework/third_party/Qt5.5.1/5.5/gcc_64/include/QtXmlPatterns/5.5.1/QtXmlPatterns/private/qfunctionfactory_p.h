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


#ifndef Patternist_FunctionFactory_H
#define Patternist_FunctionFactory_H

#include <QHash>
#include <QSharedData>

#include <private/qexpression_p.h>
#include <private/qfunctionsignature_p.h>
#include <private/qprimitives_p.h>
#include <QXmlName>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short An entry point for looking up and creating FunctionCall instances.
     *
     * @ingroup Patternist_functions
     * @see <a href ="http://www.w3.org/TR/xpath-functions/">XQuery 1.0
     * and XPath 2.0 Functions and Operators</a>
     * @see <a href="http://www.w3.org/TR/xpath20/#dt-function-signature">XML Path
     * Language (XPath) 2.0, Definition: Function signatures</a>
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class FunctionFactory : public QSharedData
    {
    public:

        typedef QExplicitlySharedDataPointer<FunctionFactory> Ptr;
        typedef QList<FunctionFactory::Ptr> List;

        virtual ~FunctionFactory();

        /**
         * Creates a function call implementation.
         *
         * A FunctionFactory represents a set of functions, which it
         * is able to instantiate and to serve FunctionSignatures for. Conventionally,
         * a FunctionFactory per namespace exists.
         *
         * @note This function should not issue any error unless it is absolutely
         * confident that the error cannot be fixed in another way. For example, in
         * some cases it might be that a function is available in another FunctionFactory
         * and it would therefore be wrong to issue an error signalling that no function
         * by that @p name exists, but leave that to the callee.
         * @param name the name of the function to create. In Clark syntax, this could
         * for example be {http://www.w3.org/2005/04/xpath-functions}lower-case
         * @param arguments the function's operands
         * @param context the usual StaticContext which supplies compile time data
         * and reporting functionality.
         * @param r the SourceLocationReflection that identifies the callsite.
         * @returns an instance of Expression which is the function implementation
         * for @p name. Or, a static error was raised.
         */
        virtual Expression::Ptr createFunctionCall(const QXmlName name,
                                                   const Expression::List &arguments,
                                                   const StaticContext::Ptr &context,
                                                   const SourceLocationReflection *const r) = 0;

        /**
         * Determines whether a function with the name @p name and arity @p arity
         * is available. The implementation operates on the result of
         * retrieveFunctionSignature() to determine the result.
         *
         * @param np the NamePool.
         * @param name the name of the function. For example fn:string-join.
         * @param arity the number of arguments the function must have.
         */
        virtual bool isAvailable(const NamePool::Ptr &np,
                                 const QXmlName name,
                                 const xsInteger arity);

        virtual FunctionSignature::Hash functionSignatures() const = 0;

        /**
         * Determines whether this FunctionFactory contains the function signature
         * @p signature.
         *
         * The implementation uses functionSignatures().
         */
        bool hasSignature(const FunctionSignature::Ptr &signature) const;

    protected:
        /**
         * @short This constructor cannot be removed, because it can't be synthesized, for
         * some reason.
         */
        inline FunctionFactory()
        {
        }

        /**
         * This is a convenience function for sub-classes. It retrieves the
         * function signature for function with name @p name.
         *
         * According to the specifications are function signatures identified by their
         * name and arity, but currently is the arity not part of the signature.
         *
         * If no function could be found for the given name, @c null is returned.
         */
        virtual FunctionSignature::Ptr retrieveFunctionSignature(const NamePool::Ptr &np, const QXmlName name) = 0;

    private:
        Q_DISABLE_COPY(FunctionFactory)
    };
}

QT_END_NAMESPACE

#endif
