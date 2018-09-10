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

#ifndef QNETWORKACCESSDELEGATOR_P_H
#define QNETWORKACCESSDELEGATOR_P_H

#include <QObject>
#include <QPointer>
#include <QSharedData>

QT_BEGIN_NAMESPACE

class QNetworkAccessManager;
class QUrl;

namespace QPatternist
{
    /**
     * @short A value based class that hands out QNetworkAccessManager
     * appropriately for variable bindings.
     *
     * NetworkAccessDelegator is an indirection mechanism which takes care of
     * the fact that QIODevice instances are injected into the data model by
     * having them represented using a URI. managerFor() returns appropriately
     * the right network access manager depending on whether the URI refers to
     * a variable, or to something else.
     *
     * The constructor take a pointer to two NetworkAccessDelegator instances.
     * First is a generic one, the second is the one which handles variable
     * bindings. managerFor() then returns the appropriate one depending on the
     * URI.
     *
     * @since 4.5
     * @see AccelTreeResourceLoader::load()
     * @author Frans Englich <frans.englich@nokia.com>
     */
    class Q_AUTOTEST_EXPORT NetworkAccessDelegator : public QObject
                                                   , public QSharedData
    {
    public:
        typedef QExplicitlySharedDataPointer<NetworkAccessDelegator> Ptr;
        NetworkAccessDelegator(QNetworkAccessManager *const genericManager,
                               QNetworkAccessManager *const variableURIManager);

        QNetworkAccessManager *managerFor(const QUrl &uri);

        QPointer<QNetworkAccessManager> m_genericManager;
        QPointer<QNetworkAccessManager> m_variableURIManager;
    private:
        Q_DISABLE_COPY(NetworkAccessDelegator)
    };
}

QT_END_NAMESPACE
#endif
