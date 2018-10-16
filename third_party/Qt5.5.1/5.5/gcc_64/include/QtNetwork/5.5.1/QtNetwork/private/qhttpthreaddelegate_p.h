/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNetwork module of the Qt Toolkit.
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

#ifndef QHTTPTHREADDELEGATE_H
#define QHTTPTHREADDELEGATE_H


//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of the Network Access API.  This header file may change from
// version to version without notice, or even be removed.
//
// We mean it.
//

#include <QObject>
#include <QThreadStorage>
#include <QNetworkProxy>
#include <QSslConfiguration>
#include <QSslError>
#include <QList>
#include <QNetworkReply>
#include "qhttpnetworkrequest_p.h"
#include "qhttpnetworkconnection_p.h"
#include <QSharedPointer>
#include "qsslconfiguration.h"
#include "private/qnoncontiguousbytedevice_p.h"
#include "qnetworkaccessauthenticationmanager_p.h"

#ifndef QT_NO_HTTP

QT_BEGIN_NAMESPACE

class QAuthenticator;
class QHttpNetworkReply;
class QEventLoop;
class QNetworkAccessCache;
class QNetworkAccessCachedHttpConnection;

class QHttpThreadDelegate : public QObject
{
    Q_OBJECT
public:
    explicit QHttpThreadDelegate(QObject *parent = 0);

    ~QHttpThreadDelegate();

    // incoming
    bool ssl;
#ifndef QT_NO_SSL
    QSslConfiguration incomingSslConfiguration;
#endif
    QHttpNetworkRequest httpRequest;
    qint64 downloadBufferMaximumSize;
    qint64 readBufferMaxSize;
    qint64 bytesEmitted;
    // From backend, modified by us for signal compression
    QSharedPointer<QAtomicInt> pendingDownloadData;
    QSharedPointer<QAtomicInt> pendingDownloadProgress;
#ifndef QT_NO_NETWORKPROXY
    QNetworkProxy cacheProxy;
    QNetworkProxy transparentProxy;
#endif
    QSharedPointer<QNetworkAccessAuthenticationManager> authenticationManager;
    bool synchronous;

    // outgoing, Retrieved in the synchronous HTTP case
    QByteArray synchronousDownloadData;
    QList<QPair<QByteArray,QByteArray> > incomingHeaders;
    int incomingStatusCode;
    QString incomingReasonPhrase;
    bool isPipeliningUsed;
    bool isSpdyUsed;
    qint64 incomingContentLength;
    QNetworkReply::NetworkError incomingErrorCode;
    QString incomingErrorDetail;
#ifndef QT_NO_BEARERMANAGEMENT
    QSharedPointer<QNetworkSession> networkSession;
#endif

protected:
    // The zerocopy download buffer, if used:
    QSharedPointer<char> downloadBuffer;
    // The QHttpNetworkConnection that is used
    QNetworkAccessCachedHttpConnection *httpConnection;
    QByteArray cacheKey;
    QHttpNetworkReply *httpReply;

    // Used for implementing the synchronous HTTP, see startRequestSynchronously()
    QEventLoop *synchronousRequestLoop;

signals:
    void authenticationRequired(const QHttpNetworkRequest &request, QAuthenticator *);
#ifndef QT_NO_NETWORKPROXY
    void proxyAuthenticationRequired(const QNetworkProxy &, QAuthenticator *);
#endif
#ifndef QT_NO_SSL
    void encrypted();
    void sslErrors(const QList<QSslError> &, bool *, QList<QSslError> *);
    void sslConfigurationChanged(const QSslConfiguration);
    void preSharedKeyAuthenticationRequired(QSslPreSharedKeyAuthenticator *);
#endif
    void downloadMetaData(QList<QPair<QByteArray,QByteArray> >, int, QString, bool,
                          QSharedPointer<char>, qint64, bool);
    void downloadProgress(qint64, qint64);
    void downloadData(QByteArray);
    void error(QNetworkReply::NetworkError, const QString);
    void downloadFinished();
public slots:
    // This are called via QueuedConnection from user thread
    void startRequest();
    void abortRequest();
    void readBufferSizeChanged(qint64 size);
    void readBufferFreed(qint64 size);

    // This is called with a BlockingQueuedConnection from user thread
    void startRequestSynchronously();
protected slots:
    // From QHttp*
    void readyReadSlot();
    void finishedSlot();
    void finishedWithErrorSlot(QNetworkReply::NetworkError errorCode, const QString &detail = QString());
    void synchronousFinishedSlot();
    void synchronousFinishedWithErrorSlot(QNetworkReply::NetworkError errorCode, const QString &detail = QString());
    void headerChangedSlot();
    void synchronousHeaderChangedSlot();
    void dataReadProgressSlot(qint64 done, qint64 total);
    void cacheCredentialsSlot(const QHttpNetworkRequest &request, QAuthenticator *authenticator);
#ifndef QT_NO_SSL
    void encryptedSlot();
    void sslErrorsSlot(const QList<QSslError> &errors);
    void preSharedKeyAuthenticationRequiredSlot(QSslPreSharedKeyAuthenticator *authenticator);
#endif

    void synchronousAuthenticationRequiredSlot(const QHttpNetworkRequest &request, QAuthenticator *);
#ifndef QT_NO_NETWORKPROXY
    void synchronousProxyAuthenticationRequiredSlot(const QNetworkProxy &, QAuthenticator *);
#endif

protected:
    // Cache for all the QHttpNetworkConnection objects.
    // This is per thread.
    static QThreadStorage<QNetworkAccessCache *> connections;

};

// This QNonContiguousByteDevice is connected to the QNetworkAccessHttpBackend
// and represents the PUT/POST data.
class QNonContiguousByteDeviceThreadForwardImpl : public QNonContiguousByteDevice
{
    Q_OBJECT
protected:
    bool wantDataPending;
    qint64 m_amount;
    char *m_data;
    QByteArray m_dataArray;
    bool m_atEnd;
    qint64 m_size;
    qint64 m_pos; // to match calls of haveDataSlot with the expected position
public:
    QNonContiguousByteDeviceThreadForwardImpl(bool aE, qint64 s)
        : QNonContiguousByteDevice(),
          wantDataPending(false),
          m_amount(0),
          m_data(0),
          m_atEnd(aE),
          m_size(s),
          m_pos(0)
    {
    }

    ~QNonContiguousByteDeviceThreadForwardImpl()
    {
    }

    qint64 pos() Q_DECL_OVERRIDE
    {
        return m_pos;
    }

    const char* readPointer(qint64 maximumLength, qint64 &len) Q_DECL_OVERRIDE
    {
        if (m_amount > 0) {
            len = m_amount;
            return m_data;
        }

        if (m_atEnd) {
            len = -1;
        } else if (!wantDataPending) {
            len = 0;
            wantDataPending = true;
            emit wantData(maximumLength);
        } else {
            // Do nothing, we already sent a wantData signal and wait for results
            len = 0;
        }
        return 0;
    }

    bool advanceReadPointer(qint64 a) Q_DECL_OVERRIDE
    {
        if (m_data == 0)
            return false;

        m_amount -= a;
        m_data += a;
        m_pos += a;

        // To main thread to inform about our state. The m_pos will be sent as a sanity check.
        emit processedData(m_pos, a);

        return true;
    }

    bool atEnd() Q_DECL_OVERRIDE
    {
        if (m_amount > 0)
            return false;
        else
            return m_atEnd;
    }

    bool reset() Q_DECL_OVERRIDE
    {
        m_amount = 0;
        m_data = 0;
        m_dataArray.clear();

        if (wantDataPending) {
            // had requested the user thread to send some data (only 1 in-flight at any moment)
            wantDataPending = false;
        }

        // Communicate as BlockingQueuedConnection
        bool b = false;
        emit resetData(&b);
        if (b) {
            // the reset succeeded, we're at pos 0 again
            m_pos = 0;
            // the HTTP code will anyway abort the request if !b.
        }
        return b;
    }

    qint64 size() Q_DECL_OVERRIDE
    {
        return m_size;
    }

public slots:
    // From user thread:
    void haveDataSlot(qint64 pos, QByteArray dataArray, bool dataAtEnd, qint64 dataSize)
    {
        if (pos != m_pos) {
            // Sometimes when re-sending a request in the qhttpnetwork* layer there is a pending haveData from the
            // user thread on the way to us. We need to ignore it since it is the data for the wrong(later) chunk.
            return;
        }
        wantDataPending = false;

        m_dataArray = dataArray;
        m_data = const_cast<char*>(m_dataArray.constData());
        m_amount = dataArray.size();

        m_atEnd = dataAtEnd;
        m_size = dataSize;

        // This will tell the HTTP code (QHttpNetworkConnectionChannel) that we have data available now
        emit readyRead();
    }

signals:
    // void readyRead(); in parent class
    // void readProgress(qint64 current, qint64 total); happens in the main thread with the real bytedevice

    // to main thread:
    void wantData(qint64);
    void processedData(qint64 pos, qint64 amount);
    void resetData(bool *b);
};

QT_END_NAMESPACE

#endif // QT_NO_HTTP

#endif // QHTTPTHREADDELEGATE_H
