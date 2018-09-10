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

#ifndef Patternist_CallTemplate_H
#define Patternist_CallTemplate_H

#include <private/qcallsite_p.h>
#include <private/qtemplateinvoker_p.h>
#include <private/qtemplate_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Implements @c xsl:call-template.
     *
     * @since 4.5
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class CallTemplate : public TemplateInvoker
    {
    public:
        typedef QExplicitlySharedDataPointer<CallTemplate> Ptr;

        CallTemplate(const QXmlName &name,
                     const WithParam::Hash &withParams);

        virtual Item::Iterator::Ptr evaluateSequence(const DynamicContext::Ptr &context) const;
        virtual bool evaluateEBV(const DynamicContext::Ptr &context) const;
        virtual void evaluateToSequenceReceiver(const DynamicContext::Ptr &context) const;

        virtual SequenceType::Ptr staticType() const;
        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;
        virtual Properties properties() const;
        virtual Properties dependencies() const;

        /**
         * This is a bit complicated by that we have two required types, one
         * specified by @c xsl:param in the template declaration, and one on @c
         * xsl:with-param.
         *
         * @see UserFunctionCallsite::expectedOperandTypes()
         * @see <a href="http://www.w3.org/TR/xslt20/#with-param">XSL
         * Transformations (XSLT) Version 2.0, 10.1.1 Passing Parameters to Templates</a>
         */
        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);


        inline void setTemplate(const Template::Ptr &templ)
        {
            m_template = templ;
        }

        virtual bool configureRecursion(const CallTargetDescription::Ptr &sign);
        virtual Expression::Ptr body() const;
        virtual CallTargetDescription::Ptr callTargetDescription() const;
    private:
        Template::Ptr   m_template;
    };
}

QT_END_NAMESPACE

#endif
