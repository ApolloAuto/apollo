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

#ifndef Patternist_DelegatingStaticContext_H
#define Patternist_DelegatingStaticContext_H

#include <QUrl>

#include <private/qstaticcontext_p.h>
#include <private/qfunctionfactory_p.h>
#include <private/qschematypefactory_p.h>

QT_BEGIN_NAMESPACE

namespace QPatternist
{
    /**
     * @short Delegates all members to a second instance. Used for
     * sub-classing.
     *
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT DelegatingStaticContext : public StaticContext
    {
    public:
        virtual NamespaceResolver::Ptr namespaceBindings() const;
        virtual void setNamespaceBindings(const NamespaceResolver::Ptr &);

        virtual FunctionFactory::Ptr functionSignatures() const;
        virtual SchemaTypeFactory::Ptr schemaDefinitions() const;
        virtual DynamicContext::Ptr dynamicContext() const;

        virtual QUrl baseURI() const;
        virtual void setBaseURI(const QUrl &uri);

        virtual bool compatModeEnabled() const;
        virtual void setCompatModeEnabled(const bool newVal);

        virtual QUrl defaultCollation() const;

        virtual QAbstractMessageHandler * messageHandler() const;

        virtual void setDefaultCollation(const QUrl &uri);

        virtual BoundarySpacePolicy boundarySpacePolicy() const;
        virtual void setBoundarySpacePolicy(const BoundarySpacePolicy policy);

        virtual ConstructionMode constructionMode() const;
        virtual void setConstructionMode(const ConstructionMode mode);

        virtual OrderingMode orderingMode() const;
        virtual void setOrderingMode(const OrderingMode mode);
        virtual OrderingEmptySequence orderingEmptySequence() const;
        virtual void setOrderingEmptySequence(const OrderingEmptySequence ordering);

        virtual QString defaultFunctionNamespace() const;
        virtual void setDefaultFunctionNamespace(const QString &ns);

        virtual QString defaultElementNamespace() const;
        virtual void setDefaultElementNamespace(const QString &ns);

        virtual InheritMode inheritMode() const;
        virtual void setInheritMode(const InheritMode mode);

        virtual PreserveMode preserveMode() const;
        virtual void setPreserveMode(const PreserveMode mode);

        virtual ItemType::Ptr contextItemType() const;
        virtual ItemType::Ptr currentItemType() const;

        virtual StaticContext::Ptr copy() const;

        virtual ExternalVariableLoader::Ptr externalVariableLoader() const;
        virtual ResourceLoader::Ptr resourceLoader() const;
        virtual NamePool::Ptr namePool() const;
        virtual void addLocation(const SourceLocationReflection *const reflection,
                                 const QSourceLocation &location);
        virtual LocationHash sourceLocations() const;
        virtual QSourceLocation locationFor(const SourceLocationReflection *const reflection) const;
        virtual const QAbstractUriResolver *uriResolver() const;

        virtual VariableSlotID currentRangeSlot() const;
        virtual VariableSlotID allocateRangeSlot();

    protected:
        DelegatingStaticContext(const StaticContext::Ptr &context);

    private:
        const StaticContext::Ptr    m_context;
    };
}

QT_END_NAMESPACE

#endif
