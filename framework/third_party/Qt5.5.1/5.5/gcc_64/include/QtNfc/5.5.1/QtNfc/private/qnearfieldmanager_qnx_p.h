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

#ifndef QNEARFIELDMANAGER_QNX_P_H
#define QNEARFIELDMANAGER_QNX_P_H

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

#include "qnearfieldmanager_p.h"
#include "qnearfieldmanager.h"
#include "qnearfieldtarget.h"

#include "qnx/qnxnfcmanager_p.h"

QT_BEGIN_NAMESPACE

class QNearFieldManagerPrivateImpl : public QNearFieldManagerPrivate
{
    Q_OBJECT

public:
    QNearFieldManagerPrivateImpl();
    ~QNearFieldManagerPrivateImpl();

    bool isAvailable() const;

    bool startTargetDetection();

    void stopTargetDetection();

    int registerNdefMessageHandler(QObject *object, const QMetaMethod &method);

    int registerNdefMessageHandler(const QNdefFilter &filter, QObject *object, const QMetaMethod &method);

    bool unregisterNdefMessageHandler(int handlerId);

    void requestAccess(QNearFieldManager::TargetAccessModes accessModes);

    void releaseAccess(QNearFieldManager::TargetAccessModes accessModes);

private slots:
    void handleMessage(const QNdefMessage&, QNearFieldTarget *);
    void newTarget(QNearFieldTarget *target, const QList<QNdefMessage> &);

private:
    void updateNdefFilter();
    QList<QNearFieldTarget::Type> m_detectTargetTypes;

    int m_handlerID;
    QList< QPair<QPair<int, QObject *>, QMetaMethod> > ndefMessageHandlers;
    QList< QPair<QPair<int, QObject *>, QPair<QNdefFilter, QMetaMethod> > > ndefFilterHandlers;
};

QT_END_NAMESPACE

#endif // QNEARFIELDMANAGER_QNX_P_H
