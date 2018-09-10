/***************************************************************************
**
** Copyright (C) 2012 Research In Motion
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

#ifndef QNEARFIELDTARGET_QNX_H
#define QNEARFIELDTARGET_QNX_H

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

#include <qnearfieldtarget.h>
#include <qnearfieldtarget_p.h>
#include <qndefmessage.h>

#include <nfc/nfc.h>
#include <nfc/nfc_types.h>

#include "qnx/qnxnfcmanager_p.h"

QT_BEGIN_NAMESPACE

#define TAG_NAME_BUFFER 64

template <typename T>
class NearFieldTarget : public T
{
public:

    NearFieldTarget(QObject *parent, nfc_target_t *target, const QList<QNdefMessage> &messages)
        :   T(parent), m_target(target)
    {
        char buf[TAG_NAME_BUFFER];
        size_t bufLength;

        if (nfc_get_tag_name(target, buf, TAG_NAME_BUFFER, &bufLength) != NFC_RESULT_SUCCESS)
            qWarning() << Q_FUNC_INFO << "Could not get tag name";
        else
            m_tagName = QByteArray(buf, bufLength);

        if (nfc_get_tag_id(target, reinterpret_cast<uchar_t *>(buf), TAG_NAME_BUFFER, &bufLength) != NFC_RESULT_SUCCESS)
            qWarning() << Q_FUNC_INFO << "Could not get tag id";
        else
            m_tagId = QByteArray(buf,bufLength);

        m_ndefMessages = messages;
    }

    ~NearFieldTarget()
    {
        //Not entierely sure if this is the right place to do that
        nfc_destroy_target(m_target);
    }

    QByteArray uid() const
    {
        return m_tagId;
    }

    QNearFieldTarget::Type type() const
    {
        if (m_tagName == QByteArray("Jewel"))
            return QNearFieldTarget::NfcTagType1;
        else if (m_tagName == QByteArray("Topaz"))
            return QNearFieldTarget::NfcTagType1;
        else if (m_tagName == QByteArray("Topaz 512"))
            return QNearFieldTarget::NfcTagType1;
        else if (m_tagName == QByteArray("Mifare UL"))
            return QNearFieldTarget::NfcTagType2;
        else if (m_tagName == QByteArray("Mifare UL C"))
            return QNearFieldTarget::NfcTagType2;
        else if (m_tagName == QByteArray("Mifare 1K"))
            return QNearFieldTarget::MifareTag;
        else if (m_tagName == QByteArray("Kovio"))
            return QNearFieldTarget::NfcTagType2;

        return QNearFieldTarget::ProprietaryTag;
    }

    QNearFieldTarget::AccessMethods accessMethods() const
    {
        QNearFieldTarget::AccessMethods result = QNearFieldTarget::NdefAccess;
        return result;
    }

    bool hasNdefMessage()
    {
        return m_ndefMessages.count() > 0;
    }

    QNearFieldTarget::RequestId readNdefMessages()
    {
        for (int i = 0; i < m_ndefMessages.size(); i++) {
            emit QNearFieldTarget::ndefMessageRead(m_ndefMessages.at(i));
        }
        QNearFieldTarget::RequestId requestId = QNearFieldTarget::RequestId(new QNearFieldTarget::RequestIdPrivate());
        QMetaObject::invokeMethod(this, "requestCompleted", Qt::QueuedConnection,
                                  Q_ARG(const QNearFieldTarget::RequestId, requestId));
        return requestId;
    }

    QNearFieldTarget::RequestId sendCommand(const QByteArray &command)
    {
        Q_UNUSED(command);
    #if 0
        const int max_nfc_command_length = 256;
        //Not tested
        bool isSupported;
        nfc_tag_supports_tag_type (m_target,TAG_TYPE_ISO_14443_3, &isSupported);
        nfc_tag_type_t tagType= TAG_TYPE_ISO_14443_3;
        if (!isSupported) {
            nfc_tag_supports_tag_type (m_target,TAG_TYPE_ISO_14443_4, &isSupported);
            tagType = TAG_TYPE_ISO_14443_4;
            if (!isSupported) {
                nfc_tag_supports_tag_type (m_target,TAG_TYPE_ISO_15693_3, &isSupported);
                tagType = TAG_TYPE_ISO_15693_3;
                //We don't support this tag
                if (!isSupported) {
                    emit QNearFieldTarget::error(QNearFieldTarget::UnsupportedError, QNearFieldTarget::RequestId());
                    return QNearFieldTarget::RequestId();
                }
            }
        }
        m_cmdRespons = reinterpret_cast<char *> malloc (max_nfc_command_length);
        nfc_result_t result = nfc_tag_transceive (m_target, tagType, command.data(), command.length(), m_cmdRespons, max_nfc_command_length, &m_cmdResponseLength);
        if (result != NFC_RESULT_SUCCESS) {
            emit QNearFieldTarget::error(QNearFieldTarget::UnknownError, QNearFieldTarget::RequestId());
            qWarning() << Q_FUNC_INFO << "nfc_tag_transceive failed"
        }
    #else
        emit QNearFieldTarget::error(QNearFieldTarget::UnsupportedError, QNearFieldTarget::RequestId());
        return QNearFieldTarget::RequestId();
    #endif
    }

    QNearFieldTarget::RequestId sendCommands(const QList<QByteArray> &commands)
    {
        Q_UNUSED(commands);
        QNearFieldTarget::RequestId requestId;
        for (int i = 0; i < commands.size(); i++) {
            requestId = sendCommand(commands.at(i)); //The request id of the last command will be returned
        }
        return requestId;
    }

    QNearFieldTarget::RequestId writeNdefMessages(const QList<QNdefMessage> &messages)
    {
        for (int i=0; i<messages.count(); i++) {
            nfc_ndef_message_t *newMessage;
            QByteArray msg = messages.at(i).toByteArray();
            nfc_result_t result = nfc_create_ndef_message_from_bytes(reinterpret_cast<const uchar_t *> (msg.constData()), msg.length(), &newMessage);
            if (result != NFC_RESULT_SUCCESS) {
                qWarning() << Q_FUNC_INFO << "Could not convert QNdefMessage to byte array" << result;
                nfc_delete_ndef_message(newMessage, true);
                emit QNearFieldTarget::error(QNearFieldTarget::UnknownError,
                             QNearFieldTarget::RequestId());
                return QNearFieldTarget::RequestId();
            }

            result = nfc_write_ndef_message_to_tag(m_target, newMessage, i == 0 ? false : true);
            nfc_delete_ndef_message(newMessage, true);

            if (result != NFC_RESULT_SUCCESS) {
                qWarning() << Q_FUNC_INFO << "Could not write message";
                emit QNearFieldTarget::error(QNearFieldTarget::NdefWriteError,
                             QNearFieldTarget::RequestId());

                return QNearFieldTarget::RequestId();
            }

        }
        QNearFieldTarget::RequestId requestId = QNearFieldTarget::RequestId(new QNearFieldTarget::RequestIdPrivate());
        QMetaObject::invokeMethod(this, "requestCompleted", Qt::QueuedConnection,
                                  Q_ARG(const QNearFieldTarget::RequestId, requestId));
        return requestId;
    }

protected:
    nfc_target_t *m_target;
#if 0
    char m_cmdRespons;
    size_t m_cmdResponseLength;
#endif
    QByteArray m_tagName;
    QByteArray m_tagId;
    QList<QNdefMessage> m_ndefMessages;
};

QT_END_NAMESPACE

#endif // QNEARFIELDTARGET_QNX_H
