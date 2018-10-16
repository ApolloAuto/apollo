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

#ifndef Patternist_ExternalVariableLoader_H
#define Patternist_ExternalVariableLoader_H

#include <private/qitem_p.h>
#include <private/qsequencetype_p.h>
#include <QXmlName>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class DynamicContext;

    /**
     * @short Responsible for loading and declaring available external variables.
     *
     * An external variable in XQuery is a global variable that has been declared to receive
     * its value from the XQuery implementation, as opposed to an initializing expression. Here
     * is an example of a query with an external variable declaration, followed by a ordinary
     * global variable:
     *
     * <tt> declare variable $theName external;
     * declare variable $theName := "the value";
     * "And here's the query body(a string literal)"</tt>
     *
     * An external variable declaration can also specify a sequence type:
     *
     * <tt>declare variable $theName as xs:integer external;</tt>
     *
     * This class allows the user to supply the values to external variables. When
     * an external variable declaration occur in the query,
     * announceExternalVariable() is called.
     *
     * @ingroup Patternist_xdm
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT ExternalVariableLoader : public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<ExternalVariableLoader> Ptr;
        inline ExternalVariableLoader() {}

        virtual ~ExternalVariableLoader();

        /**
         * Called when Patternist encounters an external variable in the query. It is guaranteed
         * to be called once for each external variable appearing in a query module.
         *
         * @param name the name of the variable. Quaranteed to never be @c null.
         * @param declaredType the type that the user declared the variable to be of. Whether
         * this type matches the actual value of the variable or not is irrelevant. Patternist
         * will do the necessary error handling based on the sequence type that is returned from
         * this function. If the user didn't declare a type, the type is <tt>item()*</tt>(zero or
         * more items). Quaranteed to never be @c null.
         * @returns the sequence type of the value this ExternalVariableLoader actually supplies. However,
         * if the ExternalVariableLoader knows it cannot supply a variable by this name, @c null should be
         * returned.
         */
        virtual SequenceType::Ptr announceExternalVariable(const QXmlName name,
                                                           const SequenceType::Ptr &declaredType);

        /**
         * This function is called at runtime when the external variable by name @p name needs
         * to be evaluated. It is not defined how many times this function will be called. It
         * depends on aspects such as how the query was optimized.
         *
         * @param name the name of the variable. Quaranteed to never be @c null.
         * @param context the DynamicContext.
         * @returns the value of the variable. Remember that this value must match the
         * sequence type returned from announceExternalVariable() for the same name.
         */
        virtual Item::Iterator::Ptr evaluateSequence(const QXmlName name,
                                                     const QExplicitlySharedDataPointer<DynamicContext> &context);

        virtual Item evaluateSingleton(const QXmlName name,
                                            const QExplicitlySharedDataPointer<DynamicContext> &context);
        virtual bool evaluateEBV(const QXmlName name,
                                 const QExplicitlySharedDataPointer<DynamicContext> &context);
    };
}

QT_END_NAMESPACE

#endif
