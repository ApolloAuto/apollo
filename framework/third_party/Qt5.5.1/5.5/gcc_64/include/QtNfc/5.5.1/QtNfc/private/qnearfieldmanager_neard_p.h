/***************************************************************************
**
** Copyright (C) 2014 BlackBerry Limited. All rights reserved.
** Copyright (C) 2014 BasysKom GmbH.
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

#ifndef QNEARFIELDMANAGER_NEARD_H
#define QNEARFIELDMANAGER_NEARD_H

#include "qnearfieldmanager_p.h"
#include "qnearfieldmanager.h"
#include "qnearfieldtarget.h"
#include "neard/neard_helper_p.h"

#include <QDBusObjectPath>
#include <QDBusVariant>
#include <QMap>

class OrgNeardManagerInterface;

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

    // not implemented
    int registerNdefMessageHandler(QObject *object, const QMetaMethod &method);

    int registerNdefMessageHandler(const QNdefFilter &filter, QObject *object, const QMetaMethod &method);

    bool unregisterNdefMessageHandler(int handlerId);

    void requestAccess(QNearFieldManager::TargetAccessModes accessModes);

    void releaseAccess(QNearFieldManager::TargetAccessModes accessModes);

private Q_SLOTS:
    void handleTagFound(const QDBusObjectPath&);
    void handleTagRemoved(const QDBusObjectPath&);

private:
    QString m_adapterPath;
    QMap<QString, QNearFieldTarget*> m_activeTags;
    NeardHelper *m_neardHelper;
};

QT_END_NAMESPACE


#endif // QNEARFIELDMANAGER_NEARD_H
