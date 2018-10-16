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

#ifndef ENGINIOCLIENTCONNECTION_H
#define ENGINIOCLIENTCONNECTION_H

#include <Enginio/enginioclient_global.h>
#include <Enginio/enginio.h>
#include <QObject>
#include <QtCore/qscopedpointer.h>
#include <QtCore/qtypeinfo.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qurl.h>

QT_BEGIN_NAMESPACE

class QNetworkAccessManager;
class EnginioIdentity;
class EnginioClientConnectionPrivate;

class ENGINIOCLIENT_EXPORT EnginioClientConnection : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QByteArray backendId READ backendId WRITE setBackendId NOTIFY backendIdChanged FINAL)
    Q_PROPERTY(QUrl serviceUrl READ serviceUrl WRITE setServiceUrl NOTIFY serviceUrlChanged FINAL)
    Q_PROPERTY(EnginioIdentity *identity READ identity WRITE setIdentity NOTIFY identityChanged FINAL)
    Q_PROPERTY(Enginio::AuthenticationState authenticationState READ authenticationState NOTIFY authenticationStateChanged FINAL)

    Q_ENUMS(Enginio::Operation) // TODO remove me QTBUG-33577
    Q_ENUMS(Enginio::AuthenticationState) // TODO remove me QTBUG-33577

public:
    ~EnginioClientConnection();


    QByteArray backendId() const Q_REQUIRED_RESULT;
    void setBackendId(const QByteArray &backendId);
    EnginioIdentity *identity() const Q_REQUIRED_RESULT;
    void setIdentity(EnginioIdentity *identity);
    Enginio::AuthenticationState authenticationState() const Q_REQUIRED_RESULT;

    QUrl serviceUrl() const Q_REQUIRED_RESULT;
    void setServiceUrl(const QUrl &serviceUrl);
    QNetworkAccessManager *networkManager() const Q_REQUIRED_RESULT;

    bool finishDelayedReplies();

Q_SIGNALS:
    void backendIdChanged(const QByteArray &backendId);
    void serviceUrlChanged(const QUrl& url);
    void authenticationStateChanged(Enginio::AuthenticationState state);
    void identityChanged(EnginioIdentity *identity);

protected:
    explicit EnginioClientConnection(EnginioClientConnectionPrivate &dd, QObject *parent);

private:
    Q_DECLARE_PRIVATE(EnginioClientConnection)
    Q_DISABLE_COPY(EnginioClientConnection)
};

QT_END_NAMESPACE

#endif // ENGINIOCLIENTCONNECTION_H
