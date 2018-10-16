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

#ifndef QBLUETOOTHTRANSFERREPLY_H
#define QBLUETOOTHTRANSFERREPLY_H

#include <QtCore/QIODevice>

#include <QtBluetooth/QBluetoothTransferRequest>
#include <QtBluetooth/QBluetoothTransferManager>

QT_BEGIN_NAMESPACE

class QBluetoothTransferReplyPrivate;

class Q_BLUETOOTH_EXPORT QBluetoothTransferReply : public QObject
{
    Q_OBJECT

public:
    enum TransferError {
        NoError = 0,
        UnknownError,
        FileNotFoundError,
        HostNotFoundError,
        UserCanceledTransferError,
        IODeviceNotReadableError,
        ResourceBusyError,
        SessionError
    };
    Q_ENUM(TransferError)

    ~QBluetoothTransferReply();

    virtual bool isFinished() const = 0;
    virtual bool isRunning() const = 0;

    QBluetoothTransferManager *manager() const;

    virtual TransferError error() const = 0;
    virtual QString errorString() const = 0;

    QBluetoothTransferRequest request() const;

public Q_SLOTS:
    void abort();

Q_SIGNALS:
    //TODO Remove QBluetoothTransferReply* parameter in Qt 6
    void finished(QBluetoothTransferReply *);
    void transferProgress(qint64 bytesTransferred, qint64 bytesTotal);
    void error(QBluetoothTransferReply::TransferError lastError);

protected:
    explicit QBluetoothTransferReply(QObject *parent = 0);
    void setManager(QBluetoothTransferManager *manager);
    void setRequest(const QBluetoothTransferRequest &request);

protected:
    QBluetoothTransferReplyPrivate *d_ptr;

private:
    Q_DECLARE_PRIVATE(QBluetoothTransferReply)

};

QT_END_NAMESPACE

#endif // QBLUETOOTHTRANSFERREPLY_H
