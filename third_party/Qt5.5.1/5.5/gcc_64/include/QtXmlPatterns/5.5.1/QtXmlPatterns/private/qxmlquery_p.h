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

#ifndef QXMLQUERY_P_H
#define QXMLQUERY_P_H

#include <QAbstractMessageHandler>
#include <QAbstractUriResolver>
#include <QPointer>
#include <QSourceLocation>
#include <QUrl>
#include <QVariant>
#include <QXmlName>
#include <QXmlNamePool>
#include <QXmlQuery>

#include <private/qacceltreebuilder_p.h>
#include <private/qacceltreeresourceloader_p.h>
#include <private/qcoloringmessagehandler_p.h>
#include <private/qcommonsequencetypes_p.h>
#include <private/qexpressionfactory_p.h>
#include <private/qfocus_p.h>
#include <private/qfunctionfactorycollection_p.h>
#include <private/qgenericdynamiccontext_p.h>
#include <private/qgenericstaticcontext_p.h>
#include <private/qnamepool_p.h>
#include <private/qnetworkaccessdelegator_p.h>
#include <private/qreferencecountedvalue_p.h>
#include <private/qresourcedelegator_p.h>
#include <private/qstaticfocuscontext_p.h>
#include <private/quriloader_p.h>
#include <private/qvariableloader_p.h>

QT_BEGIN_NAMESPACE

class QXmlQueryPrivate
{
public:

    inline QXmlQueryPrivate(const QXmlNamePool &np = QXmlNamePool()) : namePool(np)
                                                                     , messageHandler(0)
                                                                     , uriResolver(0)
                                                                     , queryLanguage(QXmlQuery::XQuery10)
                                                                     , m_networkAccessDelegator(new QPatternist::NetworkAccessDelegator(0, 0))
    {
        m_networkAccessDelegator->m_variableURIManager = new QPatternist::URILoader(ownerObject(), namePool.d, variableLoader());
    }

    void detach()
    {
        if(m_variableLoader)
            m_variableLoader = QPatternist::VariableLoader::Ptr(new QPatternist::VariableLoader(namePool.d, m_variableLoader));

        delete m_networkAccessDelegator->m_variableURIManager;
        m_networkAccessDelegator->m_variableURIManager = new QPatternist::URILoader(ownerObject(), namePool.d, m_variableLoader);

        if(m_resourceLoader)
        {
            const QPatternist::AccelTreeResourceLoader::Ptr nev(new QPatternist::AccelTreeResourceLoader(namePool.d,
                                                                                                         m_networkAccessDelegator));
            m_resourceLoader = QPatternist::ResourceLoader::Ptr(new QPatternist::ResourceDelegator(m_resourceLoader->deviceURIs(),
                                                                                                   m_resourceLoader,
                                                                                                   nev));
        }
    }

    bool isValid()
    {
        return expression();
    }

    inline void recompileRequired()
    {
        m_expr.reset();
    }

    inline QPatternist::VariableLoader::Ptr variableLoader()
    {
        if(!m_variableLoader)
            m_variableLoader = QPatternist::VariableLoader::Ptr(new QPatternist::VariableLoader(namePool.d));

        return m_variableLoader;
    }

    inline QPatternist::GenericStaticContext::Ptr staticContext()
    {
        if(m_staticContext && m_expr)
            return m_staticContext;
        /* Else, re-create the staticContext. */

        if(!messageHandler)
            messageHandler = new QPatternist::ColoringMessageHandler(ownerObject());

        if(!m_functionFactory)
        {
            if(queryLanguage == QXmlQuery::XSLT20)
                m_functionFactory = QPatternist::FunctionFactoryCollection::xslt20Factory(namePool.d);
            else
                m_functionFactory = QPatternist::FunctionFactoryCollection::xpath20Factory(namePool.d);
        }

        const QPatternist::GenericStaticContext::Ptr genericStaticContext(new QPatternist::GenericStaticContext(namePool.d,
                                                                                                                messageHandler,
                                                                                                                queryURI,
                                                                                                                m_functionFactory,
                                                                                                                queryLanguage));
        genericStaticContext->setResourceLoader(resourceLoader());

        genericStaticContext->setExternalVariableLoader(variableLoader());

        m_staticContext = genericStaticContext;

        if(!contextItem.isNull())
            m_staticContext = QPatternist::StaticContext::Ptr(new QPatternist::StaticFocusContext(QPatternist::AtomicValue::qtToXDMType(contextItem), m_staticContext));
        else if(   queryLanguage == QXmlQuery::XmlSchema11IdentityConstraintField
                || queryLanguage == QXmlQuery::XmlSchema11IdentityConstraintSelector
                || queryLanguage == QXmlQuery::XPath20)
            m_staticContext = QPatternist::StaticContext::Ptr(new QPatternist::StaticFocusContext(QPatternist::BuiltinTypes::node, m_staticContext));

        for (int i = 0; i < m_additionalNamespaceBindings.count(); ++i) {
            m_staticContext->namespaceBindings()->addBinding(m_additionalNamespaceBindings.at(i));
        }

        return m_staticContext;
    }

    inline QPatternist::DynamicContext::Ptr dynamicContext(QAbstractXmlReceiver *const callback = 0)
    {
        const QPatternist::StaticContext::Ptr statContext(staticContext());
        Q_ASSERT(statContext);

        QPatternist::GenericDynamicContext::Ptr dynContext(new QPatternist::GenericDynamicContext(namePool.d, statContext->messageHandler(),
                                                                                                  statContext->sourceLocations()));

        QPatternist::AutoPtr<QPatternist::NodeBuilder> nodeBuilder(new QPatternist::AccelTreeBuilder<false>(QUrl(), QUrl(), namePool.d,
                                                                                                            dynContext.data()));
        dynContext->setNodeBuilder(nodeBuilder);

        dynContext->setResourceLoader(statContext->resourceLoader());
        dynContext->setExternalVariableLoader(statContext->externalVariableLoader());
        dynContext->setUriResolver(uriResolver);

        if(callback)
            dynContext->setOutputReceiver(callback);

        if(contextItem.isNull())
            return dynContext;
        else
        {
            QPatternist::DynamicContext::Ptr focus(new QPatternist::Focus(dynContext));
            QPatternist::Item::Iterator::Ptr it(QPatternist::makeSingletonIterator(QPatternist::Item::fromPublic(contextItem)));
            it->next();
            focus->setFocusIterator(it);
            return focus;
        }
    }

    inline QPatternist::AccelTreeResourceLoader::Ptr resourceLoader()
    {
        if(!m_resourceLoader)
            m_resourceLoader = (new QPatternist::AccelTreeResourceLoader(namePool.d, m_networkAccessDelegator));

        return m_resourceLoader;
    }

    void setRequiredType(const QPatternist::SequenceType::Ptr &seqType)
    {
        Q_ASSERT(seqType);
        if(!m_requiredType || m_requiredType->is(seqType))
            return;

        m_requiredType = seqType;
        m_staticContext.reset();
    }

    QPatternist::SequenceType::Ptr requiredType()
    {
        if(m_requiredType)
            return m_requiredType;
        else
        {
            m_requiredType = QPatternist::CommonSequenceTypes::ZeroOrMoreItems;
            return m_requiredType;
        }
    }

    QPatternist::Expression::Ptr expression(QIODevice *const queryDevice = 0)
    {
        if(m_expr && !queryDevice)
            return m_expr;

        /* If we need to update, but we don't have any source code, we can
         * never create an Expression. */
        if(!queryDevice)
            return QPatternist::Expression::Ptr();

        try
        {
            /* The static context has source locations, and they need to be
             * updated to the new query. */
            m_staticContext.reset();

            if(!m_expressionFactory)
                m_expressionFactory = QPatternist::ExpressionFactory::Ptr(new QPatternist::ExpressionFactory());

            m_expr = m_expressionFactory->createExpression(queryDevice, staticContext(),
                                                           queryLanguage,
                                                           requiredType(),
                                                           queryURI,
                                                           initialTemplateName);
        }
        catch(const QPatternist::Exception)
        {
            m_expr.reset();

            /* We don't call m_staticContext.reset() because it shouldn't be
             * necessary, since m_staticContext is changed when the expression
             * is changed. */
        }

        return m_expr;
    }

    inline void addAdditionalNamespaceBinding(const QXmlName &binding)
    {
        m_additionalNamespaceBindings.append(binding);
    }

    QXmlNamePool                                namePool;
    QPointer<QAbstractMessageHandler>           messageHandler;
    /**
     * Must be absolute and valid.
     */
    QUrl                                        queryURI;
    const QAbstractUriResolver *                uriResolver;
    QXmlItem                                    contextItem;
    QXmlName                                    initialTemplateName;

    inline void setExpressionFactory(const QPatternist::ExpressionFactory::Ptr &expr)
    {
        m_expressionFactory = expr;
    }

    QXmlQuery::QueryLanguage                    queryLanguage;
    QPointer<QNetworkAccessManager>             userNetworkManager;

    inline QObject *ownerObject()
    {
        if(!m_owner)
            m_owner = new QPatternist::ReferenceCountedValue<QObject>(new QObject());

        return m_owner->value;
    }

    QPatternist::ExpressionFactory::Ptr         m_expressionFactory;
    QPatternist::StaticContext::Ptr             m_staticContext;
    QPatternist::VariableLoader::Ptr            m_variableLoader;
    QPatternist::DeviceResourceLoader::Ptr      m_resourceLoader;
    /**
     * This is the AST for the query.
     */
    QPatternist::Expression::Ptr                m_expr;
    QPatternist::ReferenceCountedValue<QObject>::Ptr m_owner;

    /**
     * This is our effective network manager, that we end up using. The one the
     * user sets is userNetworkManager.
     */
    QPatternist::SequenceType::Ptr              m_requiredType;
    QPatternist::FunctionFactory::Ptr           m_functionFactory;
    QPatternist::NetworkAccessDelegator::Ptr    m_networkAccessDelegator;

    QList<QXmlName>                             m_additionalNamespaceBindings;
};

QT_END_NAMESPACE

#endif
