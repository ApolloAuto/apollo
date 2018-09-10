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

#ifndef Patternist_AbstractFunctionFactory_H
#define Patternist_AbstractFunctionFactory_H

#include <private/qcommonnamespaces_p.h>
#include <private/qfunctionfactory_p.h>
#include <private/qfunctionsignature_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Supplies convenience code for the function factories.
     *
     * @ingroup Patternist_functions
     * @see XPath10CoreFunctions
     * @see XPath20CoreFunctions
     * @see XSLT10CoreFunctions
     * @author Vincent Ricard <magic@magicninja.org>
     */
    class AbstractFunctionFactory : public FunctionFactory
    {
    public:
        virtual Expression::Ptr createFunctionCall(const QXmlName name,
                                                   const Expression::List &arguments,
                                                   const StaticContext::Ptr &context,
                                                   const SourceLocationReflection *const r);

        virtual FunctionSignature::Hash functionSignatures() const;

    protected:
        /**
         * This function is responsible for creating the actual Expression, corresponding
         * to @p localName and the function signature @p sign. It is called by
         * createFunctionCall(), once it have been determined the function actually
         * exists and have the correct arity.
         *
         * This function will only be called for names in the @c fn namespace.
         */
        virtual Expression::Ptr retrieveExpression(const QXmlName name,
                                                   const Expression::List &args,
                                                   const FunctionSignature::Ptr &sign) const = 0;

        inline
        FunctionSignature::Ptr addFunction(const QXmlName::LocalNameCode localName,
                                           const FunctionSignature::Arity minArgs,
                                           const FunctionSignature::Arity maxArgs,
                                           const SequenceType::Ptr &returnType,
                                           const Expression::Properties props)
        {
            return addFunction(localName,
                               minArgs,
                               maxArgs,
                               returnType,
                               Expression::IDIgnorableExpression,
                               props);
        }

        FunctionSignature::Ptr addFunction(const QXmlName::LocalNameCode &localName,
                                           const FunctionSignature::Arity minArgs,
                                           const FunctionSignature::Arity maxArgs,
                                           const SequenceType::Ptr &returnType,
                                           const Expression::ID id = Expression::IDIgnorableExpression,
                                           const Expression::Properties props = Expression::Properties(),
                                           const StandardNamespaces::ID ns = StandardNamespaces::fn)
        {
            const QXmlName name(ns, localName);

            const FunctionSignature::Ptr s(new FunctionSignature(name, minArgs, maxArgs,
                                                                 returnType, props, id));

            m_signatures.insert(name, s);
            return s;
        }

        static inline QXmlName::LocalNameCode argument(const NamePool::Ptr &np, const char *const name)
        {
            return np->allocateLocalName(QLatin1String(name));
        }

        FunctionSignature::Hash m_signatures;

    private:
        /**
         * @short Determines whether @p arity is a valid number of
         * arguments for the function with signature @p sign.
         *
         * If it is not, a static error with error code ReportContext::XPST0017
         * is issued via @p context.
         */
        void verifyArity(const FunctionSignature::Ptr &sign,
                         const StaticContext::Ptr &context,
                         const xsInteger arity,
                         const SourceLocationReflection *const r) const;

    };
}

QT_END_NAMESPACE

#endif
