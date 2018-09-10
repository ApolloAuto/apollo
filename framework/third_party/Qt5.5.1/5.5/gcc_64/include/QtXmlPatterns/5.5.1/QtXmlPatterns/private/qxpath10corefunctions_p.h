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

#ifndef Patternist_XPath10CoreFunctions_H
#define Patternist_XPath10CoreFunctions_H

#include <private/qabstractfunctionfactory_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{

    /**
     * @short Supplies the functions available in XPath 1.0.
     *
     * @ingroup Patternist_functions
     * @see <a href="http://www.w3.org/TR/xpath.html#corelib">XML Path Language (XPath)
     * Version 1.0, 4 Core Function Library</a>
     * @see XPath20CoreFunctions
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class XPath10CoreFunctions : public AbstractFunctionFactory
    {
    protected:
        /**
         * This function is responsible for creating the actual Expression, corresponding
         * to @p localName and the function signature @p sign. It is called by
         * createFunctionCall(), once it have been determined the function actually
         * exists and have the correct arity.
         */
        virtual Expression::Ptr retrieveExpression(const QXmlName name,
                                                   const Expression::List &args,
                                                   const FunctionSignature::Ptr &sign) const;
        virtual FunctionSignature::Ptr retrieveFunctionSignature(const NamePool::Ptr &np, const QXmlName name);
    };
}

QT_END_NAMESPACE

#endif
