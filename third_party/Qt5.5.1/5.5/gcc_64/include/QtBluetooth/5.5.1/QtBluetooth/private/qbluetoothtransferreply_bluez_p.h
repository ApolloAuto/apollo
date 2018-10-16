/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtBluetooth module of the Qt Toolkit.
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

#ifndef QBLUETOOTHTRANSFERREPLY_BLUEZ_P_H
#define QBLUETOOTHTRANSFERREPLY_BLUEZ_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/QIODevice>
#include <QtDBus/QtDBus>

#include <QtBluetooth/QBluetoothTransferRequest>
#include <QtBluetooth/QBluetoothTransferManager>

#include "qbluetoothtransferreply.h"

class OrgOpenobexClientInterface;
class AgentAdaptor;
class OrgBluezObexClient1Interface;
class OrgBluezObexObjectPush1Interface;

QT_BEGIN_NAMESPACE

class Q_BLUETOOTH_EXPORT QBluetoothTransferReplyBluez : public QBluetoothTransferReply
{
    Q_OBJECT

public:
    explicit QBluetoothTransferReplyBluez(QIODevice *input, const QBluetoothTransferRequest &request,
                                          QBluetoothTransferManager *parent = 0);
    ~QBluetoothTransferReplyBluez();

    bool isFinished() const;
    bool isRunning() const;

    QBluetoothTransferReply::TransferError error() const;
    QString errorString() const;

private slots:
    bool start();

private:
    void startOPP(const QString &filename);

    OrgOpenobexClientInterface *m_client;
    AgentAdaptor *m_agent;
    OrgBluezObexClient1Interface *m_clientBluez;
    OrgBluezObexObjectPush1Interface *m_objectPushBluez;


    QTemporaryFile *m_tempfile;
    QIODevice *m_source;

    bool m_running;
    bool m_finished;

    quint64 m_size;

    QBluetoothTransferReply::TransferError m_error;
    QString m_errorStr;

    QString m_agent_path;

    QString m_transfer_path;
    QString fileToTranser;

    static bool copyToTempFile(QIODevice *to, QIODevice *from);
    void cleanupSession();

private slots:
    void copyDone();
    void sessionCreated(QDBusPendingCallWatcher *watcher);
    void sessionStarted(QDBusPendingCallWatcher *watcher);
    void sessionChanged(const QString &interface,
                        const QVariantMap &changed_properties,
                        const QStringList &invalidated_properties);

public slots:
    void abort();
    void Complete(const QDBusObjectPath &in0);
    void Error(const QDBusObjectPath &in0, const QString &in1);
    void Progress(const QDBusObjectPath &in0, qulonglong in1);
    void Release();
    QString Request(const QDBusObjectPath &in0);
    void sendReturned(QDBusPendingCallWatcher*);

};

QT_END_NAMESPACE

#endif // QBLUETOOTHTRANSFERREPLY_H
