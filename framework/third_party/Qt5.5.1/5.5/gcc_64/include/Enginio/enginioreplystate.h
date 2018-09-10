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

#ifndef ENGINIOREPLYBASE_H
#define ENGINIOREPLYBASE_H

#include <QtCore/qobject.h>
#include <QtCore/qstring.h>
#include <QtCore/qscopedpointer.h>
#include <QtCore/qtypeinfo.h>
#include <QtCore/qmetatype.h>
#include <QtNetwork/qnetworkreply.h>

#include <Enginio/enginioclient_global.h>
#include <Enginio/enginio.h>

QT_BEGIN_NAMESPACE

class EnginioClientConnectionPrivate;
class EnginioReplyStatePrivate;
class ENGINIOCLIENT_EXPORT EnginioReplyState: public QObject
{
    Q_OBJECT
    Q_ENUMS(QNetworkReply::NetworkError) // TODO remove me QTBUG-33577
    Q_ENUMS(Enginio::ErrorType) // TODO remove me QTBUG-33577

    Q_PROPERTY(Enginio::ErrorType errorType READ errorType NOTIFY dataChanged)
    Q_PROPERTY(QNetworkReply::NetworkError networkError READ networkError NOTIFY dataChanged)
    Q_PROPERTY(QString errorString READ errorString NOTIFY dataChanged)
    Q_PROPERTY(int backendStatus READ backendStatus NOTIFY dataChanged)
    Q_PROPERTY(QString requestId READ requestId CONSTANT)

    Q_DECLARE_PRIVATE(EnginioReplyState)

public:
    ~EnginioReplyState();


    Enginio::ErrorType errorType() const Q_REQUIRED_RESULT;
    QNetworkReply::NetworkError networkError() const Q_REQUIRED_RESULT;
    QString errorString() const Q_REQUIRED_RESULT;
    QString requestId() const Q_REQUIRED_RESULT;
    int backendStatus() const Q_REQUIRED_RESULT;

    bool isError() const Q_REQUIRED_RESULT;
    bool isFinished() const Q_REQUIRED_RESULT;

    void setDelayFinishedSignal(bool delay);
    bool delayFinishedSignal() Q_REQUIRED_RESULT;

    void swapNetworkReply(EnginioReplyState *other);
    void setNetworkReply(QNetworkReply *reply);

    QJsonObject data() const Q_REQUIRED_RESULT;

public Q_SLOTS:
    void dumpDebugInfo() const;

Q_SIGNALS:
    void dataChanged();
    void progress(qint64 bytesSent, qint64 bytesTotal);

protected:
    EnginioReplyState(EnginioClientConnectionPrivate *parent, QNetworkReply *reply, EnginioReplyStatePrivate *priv);
    friend class EnginioClient;
    friend class EnginioClientConnectionPrivate;
};

QT_END_NAMESPACE

#endif // ENGINIOREPLYBASE_H
