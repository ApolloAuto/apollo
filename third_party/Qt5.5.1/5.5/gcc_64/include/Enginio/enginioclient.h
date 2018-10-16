/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtEnginio module of the Qt Toolkit.
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

#ifndef ENGINIOCLIENT_H
#define ENGINIOCLIENT_H

#include <Enginio/enginioclient_global.h>
#include <Enginio/enginioclientconnection.h>
#include <QtCore/qjsonobject.h>

QT_BEGIN_NAMESPACE

class QNetworkAccessManager;
class QNetworkReply;
class EnginioReply;
class EnginioClientPrivate;
class ENGINIOCLIENT_EXPORT EnginioClient : public EnginioClientConnection
{
    Q_OBJECT

    Q_ENUMS(Enginio::Operation) // TODO remove me QTBUG-33577
    Q_ENUMS(Enginio::AuthenticationState) // TODO remove me QTBUG-33577

    Q_DECLARE_PRIVATE(EnginioClient)
public:
    explicit EnginioClient(QObject *parent = 0);
    ~EnginioClient();

    Q_INVOKABLE EnginioReply *customRequest(const QUrl &url, const QByteArray &httpOperation, const QJsonObject &data = QJsonObject());
    Q_INVOKABLE EnginioReply *fullTextSearch(const QJsonObject &query);
    Q_INVOKABLE EnginioReply *query(const QJsonObject &query, const Enginio::Operation operation = Enginio::ObjectOperation);
    Q_INVOKABLE EnginioReply *create(const QJsonObject &object, const Enginio::Operation operation = Enginio::ObjectOperation);
    Q_INVOKABLE EnginioReply *update(const QJsonObject &object, const Enginio::Operation operation = Enginio::ObjectOperation);
    Q_INVOKABLE EnginioReply *remove(const QJsonObject &object, const Enginio::Operation operation = Enginio::ObjectOperation);

    Q_INVOKABLE EnginioReply *uploadFile(const QJsonObject &associatedObject, const QUrl &file);
    Q_INVOKABLE EnginioReply *downloadUrl(const QJsonObject &object);

Q_SIGNALS:
    void sessionAuthenticated(EnginioReply *reply) const;
    void sessionAuthenticationError(EnginioReply *reply) const;
    void sessionTerminated() const;
    void finished(EnginioReply *reply);
    void error(EnginioReply *reply);
};

QT_END_NAMESPACE

#endif // ENGINIOCLIENT_H
