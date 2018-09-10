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

#ifndef PATTERNIST_VARIABLELOADER_P_H
#define PATTERNIST_VARIABLELOADER_P_H

#include <QtCore/QSet>
#include <QtXmlPatterns/QXmlQuery>
#include <QtDebug>

#include <private/qdynamiccontext_p.h>
#include <private/qexternalvariableloader_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    class VariableLoader : public ExternalVariableLoader
    {
    public:
        typedef QHash<QXmlName, QVariant> BindingHash;
        typedef QExplicitlySharedDataPointer<VariableLoader> Ptr;

        inline VariableLoader(const NamePool::Ptr &np,
                              const VariableLoader::Ptr &previousLoader = VariableLoader::Ptr()) : m_namePool(np)
                                                                                                 , m_previousLoader(previousLoader)

        {
        }

        virtual QPatternist::SequenceType::Ptr announceExternalVariable(const QXmlName name,
                                                                        const QPatternist::SequenceType::Ptr &declaredType);
        virtual QPatternist::Item::Iterator::Ptr evaluateSequence(const QXmlName name,
                                                                  const QPatternist::DynamicContext::Ptr &);

        virtual QPatternist::Item evaluateSingleton(const QXmlName name,
                                                    const QPatternist::DynamicContext::Ptr &);

        void removeBinding(const QXmlName &name);
        bool hasBinding(const QXmlName &name) const;
        QVariant valueFor(const QXmlName &name) const;
        void addBinding(const QXmlName &name,
                        const QVariant &value);

        bool isSameType(const QVariant &v1,
                        const QVariant &v2) const;

        bool invalidationRequired(const QXmlName &name,
                                  const QVariant &variant) const;

    private:

        inline QPatternist::Item itemForName(const QXmlName &name) const;

        const NamePool::Ptr                 m_namePool;
        VariableLoader::Ptr                 m_previousLoader;
        BindingHash                         m_bindingHash;
    };
}

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QIODevice *)
Q_DECLARE_METATYPE(QXmlQuery)

#endif
