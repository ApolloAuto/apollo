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

#ifndef QNXNFCMANAGER_H
#define QNXNFCMANAGER_H

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

#include "nfc/nfc_types.h"
#include "nfc/nfc.h"
#include <QSocketNotifier>
#include <QDebug>
#include "../qndefmessage.h"
#include "../qndefrecord.h"
#include <bps/navigator_invoke.h>
#include "../qnearfieldtarget_qnx_p.h"
#include <QTimer>
#include "qnxnfceventfilter_p.h"

#ifdef QQNXNFC_DEBUG
#define qQNXNFCDebug qDebug
#else
#define qQNXNFCDebug QT_NO_QDEBUG_MACRO
#endif

QT_BEGIN_NAMESPACE

class QLlcpSocketPrivate;

class Q_DECL_EXPORT QNXNFCManager : public QObject
{
    Q_OBJECT
public:
    static QNXNFCManager *instance();
    void registerForNewInstance();
    void unregisterForInstance();
    void unregisterTargetDetection(QObject *);
    nfc_target_t *getLastTarget();
    bool isAvailable();
    void registerLLCPConnection(nfc_llcp_connection_listener_t, QObject *);
    void unregisterLLCPConnection(nfc_llcp_connection_listener_t);
    void requestTargetLost(QObject *, int);
    void unregisterTargetLost(QObject *);

private:
    QNXNFCManager();
    ~QNXNFCManager();

    static QNXNFCManager *m_instance;
    int m_instanceCount;

    QNXNFCEventFilter *ndefEventFilter;

    int nfcFD;
    QSocketNotifier *nfcNotifier;

    QList<QNdefMessage> decodeTargetMessage(nfc_target_t *);
    QList<QPair<nfc_llcp_connection_listener_t, QObject *> > llcpConnections;
    QList<QPair<unsigned int ,QObject*> > nfcTargets;

    QHash<QObject *, QList<QByteArray> > ndefFilters;
    QList<QByteArray> absNdefFilters;

    //QList<QPair<QObject*, QMetaMethod> > ndefMessageHandlers;

    //There can only be one target. The last detected one is saved here
    //currently we do not get notified when the target is disconnected. So the target might be invalid
    nfc_target_t *m_lastTarget;
    bool m_available;

    void llcpReadComplete(nfc_event_t *nfcEvent);
    void llcpWriteComplete(nfc_event_t *nfcEvent);
    void nfcReadWriteEvent(nfc_event_t *nfcEvent);
    void llcpConnectionEvent(nfc_event_t *nfcEvent);
    void targetLostEvent(nfc_event_t *nfcEvent);
    void targetLost(unsigned int target);
    void startBTHandover();

    void setupInvokeTarget();

private slots:
    void newNfcEvent(int fd);
    void invokeNdefMessage(const QNdefMessage &);

public:
    //TODO add a parameter to only detect a special target for now we are detecting all target types
    bool startTargetDetection();

    void updateNdefFilters(QList<QByteArray>,QObject *);

    QNdefMessage decodeMessage(nfc_ndef_message_t *nextMessage);

signals:
    void newLlcpConnection(nfc_target_t *);
    void ndefMessage(const QNdefMessage &, QNearFieldTarget *);
    void targetDetected(QNearFieldTarget *, const QList<QNdefMessage> &);
    void readResult(QByteArray&, nfc_target_t *);
    void llcpDisconnected();
};

QT_END_NAMESPACE

#endif

