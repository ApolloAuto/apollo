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

#ifndef Patternist_NCNameConstructor_H
#define Patternist_NCNameConstructor_H

#include <private/qsinglecontainer_p.h>
#include <private/qpatternistlocale_p.h>
#include <private/qxmlutils_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Ensures the lexical space of the string value of the Item returned
     * from its child Expression is an NCName.
     *
     * @note It doesn't actually construct an @c xs:NCName. It only ensures the lexical
     * space is an @c NCName. The atomic value can be of any string type, such as @c xs:untypedAtomic
     * of @c xs:string.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     * @ingroup Patternist_expressions
     */
    class NCNameConstructor : public SingleContainer
    {
    public:

        NCNameConstructor(const Expression::Ptr &source);

        virtual Item evaluateSingleton(const DynamicContext::Ptr &) const;

        virtual SequenceType::List expectedOperandTypes() const;

        virtual Expression::Ptr typeCheck(const StaticContext::Ptr &context,
                                          const SequenceType::Ptr &reqType);

        virtual SequenceType::Ptr staticType() const;

        virtual ExpressionVisitorResult::Ptr accept(const ExpressionVisitor::Ptr &visitor) const;

        /**
         *  Validates @p lexicalNCName as a processing instruction's target
         *  name, and raise an error if it's not an @c  NCName.
         */
        template<typename TReportContext,
                 const ReportContext::ErrorCode NameIsXML,
                 const ReportContext::ErrorCode LexicallyInvalid>
        static inline
        void validateTargetName(const QString &lexicalNCName,
                                const TReportContext &context,
                                const SourceLocationReflection *const r);
    private:

        /**
         * This translation string is put here in order to avoid duplicate messages and
         * hence reduce work for translators and increase consistency.
         */
        static
        const QString nameIsXML(const QString &lexTarget)
        {
            return QtXmlPatterns::tr("The target name in a processing instruction "
                                     "cannot be %1 in any combination of upper "
                                     "and lower case. Therefore, %2 is invalid.")
                .arg(formatKeyword("xml"), formatKeyword(lexTarget));
        }
    };

    template<typename TReportContext,
             const ReportContext::ErrorCode NameIsXML,
             const ReportContext::ErrorCode LexicallyInvalid>
    inline
    void NCNameConstructor::validateTargetName(const QString &lexicalTarget,
                                               const TReportContext &context,
                                               const SourceLocationReflection *const r)
    {
        Q_ASSERT(context);

        if(QXmlUtils::isNCName(lexicalTarget))
        {
            if(QString::compare(QLatin1String("xml"), lexicalTarget, Qt::CaseInsensitive) == 0)
                context->error(nameIsXML(lexicalTarget), NameIsXML, r);
        }
        else
        {
            context->error(QtXmlPatterns::tr("%1 is not a valid target name in "
                                             "a processing instruction. It "
                                             "must be a %2 value, e.g. %3.")
                           .arg(formatKeyword(lexicalTarget))
                           .arg(formatType(context->namePool(),
                                           BuiltinTypes::xsNCName))
                           .arg(formatKeyword("my-name.123")),
                           LexicallyInvalid,
                           r);
        }
    }
}

QT_END_NAMESPACE

#endif
