/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNfc module of the Qt Toolkit.
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

#ifndef QNEARFIELDTARGET_H
#define QNEARFIELDTARGET_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QMetaType>
#include <QtCore/QSharedDataPointer>
#include <QtNfc/qnfcglobal.h>

QT_BEGIN_NAMESPACE
class QString;
class QUrl;
QT_END_NAMESPACE

QT_BEGIN_NAMESPACE

class QNdefMessage;
class QNearFieldTargetPrivate;

class Q_NFC_EXPORT QNearFieldTarget : public QObject
{
    Q_OBJECT

    Q_DECLARE_PRIVATE(QNearFieldTarget)

public:
    enum Type {
        ProprietaryTag,
        NfcTagType1,
        NfcTagType2,
        NfcTagType3,
        NfcTagType4,
        MifareTag
    };
    Q_ENUM(Type)

    enum AccessMethod {
        UnknownAccess = 0x00,
        NdefAccess = 0x01,
        TagTypeSpecificAccess = 0x02,
        LlcpAccess = 0x04
    };
    Q_ENUM(AccessMethod)
    Q_DECLARE_FLAGS(AccessMethods, AccessMethod)

    enum Error {
        NoError,
        UnknownError,
        UnsupportedError,
        TargetOutOfRangeError,
        NoResponseError,
        ChecksumMismatchError,
        InvalidParametersError,
        NdefReadError,
        NdefWriteError
    };
    Q_ENUM(Error)

    class RequestIdPrivate;
    class Q_NFC_EXPORT RequestId
    {
    public:
        RequestId();
        RequestId(const RequestId &other);
        RequestId(RequestIdPrivate *p);
        ~RequestId();

        bool isValid() const;

        int refCount() const;

        bool operator<(const RequestId &other) const;
        bool operator==(const RequestId &other) const;
        bool operator!=(const RequestId &other) const;
        RequestId &operator=(const RequestId &other);

        QSharedDataPointer<RequestIdPrivate> d;
    };

    explicit QNearFieldTarget(QObject *parent = 0);
    virtual ~QNearFieldTarget();

    virtual QByteArray uid() const = 0;
    virtual QUrl url() const;

    virtual Type type() const = 0;
    virtual AccessMethods accessMethods() const = 0;

    bool isProcessingCommand() const;

    // NdefAccess
    virtual bool hasNdefMessage();
    virtual RequestId readNdefMessages();
    virtual RequestId writeNdefMessages(const QList<QNdefMessage> &messages);

    // TagTypeSpecificAccess
    virtual RequestId sendCommand(const QByteArray &command);
    virtual RequestId sendCommands(const QList<QByteArray> &commands);

    virtual bool waitForRequestCompleted(const RequestId &id, int msecs = 5000);

    QVariant requestResponse(const RequestId &id);
    void setResponseForRequest(const QNearFieldTarget::RequestId &id, const QVariant &response,
                               bool emitRequestCompleted = true);

protected:
    Q_INVOKABLE virtual bool handleResponse(const QNearFieldTarget::RequestId &id,
                                            const QByteArray &response);

Q_SIGNALS:
    void disconnected();

    void ndefMessageRead(const QNdefMessage &message);
    void ndefMessagesWritten();

    void requestCompleted(const QNearFieldTarget::RequestId &id);

    void error(QNearFieldTarget::Error error, const QNearFieldTarget::RequestId &id);

private:
    QNearFieldTargetPrivate *d_ptr;
};

Q_NFC_EXPORT quint16 qNfcChecksum(const char * data, uint len);

Q_DECLARE_OPERATORS_FOR_FLAGS(QNearFieldTarget::AccessMethods)

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QNearFieldTarget::RequestId)

#endif // QNEARFIELDTARGET_H
