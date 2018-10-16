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

#ifndef ENGINIOREPLY_P_H
#define ENGINIOREPLY_P_H

#include <QtCore/qdebug.h>
#include <QtCore/qhash.h>
#include <QtCore/qstring.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qjsonobject.h>
#include <QtCore/qjsondocument.h>
#include <QtNetwork/qnetworkreply.h>

#include <Enginio/private/enginioclient_p.h>
#include <Enginio/enginioreply.h>

#include <QtCore/private/qobject_p.h>

QT_BEGIN_NAMESPACE

class EnginioReplyStatePrivate : public QObjectPrivate {
    Q_DECLARE_PUBLIC(EnginioReplyState)
public:
    EnginioClientConnectionPrivate *_client;
    QNetworkReply *_nreply;
    mutable QByteArray _data;
    bool _delay;

    static EnginioReplyStatePrivate *get(EnginioReplyState *p)
    {
        return p->d_func();
    }


    EnginioReplyStatePrivate(EnginioClientConnectionPrivate *p, QNetworkReply *reply)
        : _client(p)
        , _nreply(reply)
        , _delay(false)
    {
        Q_ASSERT(reply);
    }

    bool isFinished() const Q_REQUIRED_RESULT
    {
        return _nreply->isFinished() && Q_LIKELY(!_delay);
    }

    QNetworkReply::NetworkError errorCode() const Q_REQUIRED_RESULT
    {
        return _nreply->error();
    }

    int backendStatus() const Q_REQUIRED_RESULT
    {
        return _nreply->attribute(QNetworkRequest::HttpStatusCodeAttribute).value<int>();
    }

    QString requestId() const Q_REQUIRED_RESULT
    {
        return QString::fromUtf8(_nreply->request().rawHeader(EnginioString::X_Request_Id));
    }

    QString errorString() const Q_REQUIRED_RESULT
    {
        if (errorType() == Enginio::BackendError)
            return QString::fromUtf8(pData());
        return _nreply->errorString();
    }

    Enginio::ErrorType errorType() const Q_REQUIRED_RESULT
    {
        if (errorCode() == QNetworkReply::NoError)
            return Enginio::NoError;
        if (pData().isEmpty())
            return Enginio::NetworkError;
        return Enginio::BackendError;
    }

    QJsonObject data() const Q_REQUIRED_RESULT
    {
        return QJsonDocument::fromJson(pData()).object();
    }

    QByteArray pData() const Q_REQUIRED_RESULT
    {
        if (_data.isEmpty() && _nreply->isFinished())
            _data = _nreply->readAll();
        return _data;
    }

    void dumpDebugInfo() const
    {
        static QHash<QNetworkAccessManager::Operation, QByteArray> operationNames;
        operationNames[QNetworkAccessManager::GetOperation] = "GET";
        operationNames[QNetworkAccessManager::PutOperation] = "PUT";
        operationNames[QNetworkAccessManager::PostOperation] = "POST";
        operationNames[QNetworkAccessManager::DeleteOperation] = "DELETE";
        operationNames[QNetworkAccessManager::CustomOperation] = "CUSTOM";

        QNetworkRequest request = _nreply->request();
        qDebug() << "NetworkReply:" << _nreply;
        qDebug() << "  Request URL:" << request.url().toString(/*FormattingOptions*/ QUrl::None);
        qDebug() << "  Operation:" << operationNames[_nreply->operation()];
        qDebug() << "  HTTP return code:" << backendStatus();
        qDebug() << "  Headers[Content-Type]:" << request.header(QNetworkRequest::ContentTypeHeader);
        qDebug() << "  Raw headers:" << request.rawHeaderList();
        qDebug() << "  RawHeaders[Accept]:" << request.rawHeader(EnginioString::Accept);
        qDebug() << "  RawHeaders[Authorization]:" << request.rawHeader(EnginioString::Authorization);
        qDebug() << "  RawHeaders[Content-Type]:" << request.rawHeader(EnginioString::Content_Type);
        qDebug() << "  RawHeaders[X_Request_Id]:" << request.rawHeader(EnginioString::X_Request_Id);

        QByteArray json = _client->_requestData.value(_nreply);
        if (!json.isEmpty()) {
            if (request.url().toString(QUrl::None).endsWith(QString::fromUtf8("account/auth/identity")))
                qDebug() << "Request Data hidden because it contains password";
            else
                qDebug() << "Request Data:" << json;
        }
        if (!pData().isEmpty())
            qDebug() << "Reply Data:" << pData();
    }

    virtual void emitFinished() = 0;
    void setNetworkReply(QNetworkReply *reply);
    void swapNetworkReply(EnginioReplyStatePrivate *other);
};

QT_END_NAMESPACE

#endif // ENGINIOREPLY_P_H
