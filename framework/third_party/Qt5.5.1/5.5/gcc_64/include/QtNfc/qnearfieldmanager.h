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

#ifndef QNEARFIELDMANAGER_H
#define QNEARFIELDMANAGER_H

#include <QtCore/QObject>
#include <QtNfc/qnfcglobal.h>
#include <QtNfc/QNearFieldTarget>
#include <QtNfc/QNdefRecord>
#include <QtNfc/QNdefFilter>

QT_BEGIN_NAMESPACE

class QNearFieldManagerPrivate;
class Q_NFC_EXPORT QNearFieldManager : public QObject
{
    Q_OBJECT

    Q_DECLARE_PRIVATE(QNearFieldManager)

public:
    enum TargetAccessMode {
        NoTargetAccess = 0x00,
        NdefReadTargetAccess = 0x01,
        NdefWriteTargetAccess = 0x02,
        TagTypeSpecificTargetAccess = 0x04
    };
    Q_ENUM(TargetAccessMode)
    Q_DECLARE_FLAGS(TargetAccessModes, TargetAccessMode)

    explicit QNearFieldManager(QObject *parent = 0);
    explicit QNearFieldManager(QNearFieldManagerPrivate *backend, QObject *parent = 0);
    ~QNearFieldManager();

    bool isAvailable() const;

    void setTargetAccessModes(TargetAccessModes accessModes);
    TargetAccessModes targetAccessModes() const;

    bool startTargetDetection();
    void stopTargetDetection();

    int registerNdefMessageHandler(QObject *object, const char *method);
    int registerNdefMessageHandler(QNdefRecord::TypeNameFormat typeNameFormat,
                                   const QByteArray &type,
                                   QObject *object, const char *method);
    int registerNdefMessageHandler(const QNdefFilter &filter,
                                   QObject *object, const char *method);

    bool unregisterNdefMessageHandler(int handlerId);

Q_SIGNALS:
    void targetDetected(QNearFieldTarget *target);
    void targetLost(QNearFieldTarget *target);

private:
    QNearFieldManagerPrivate *d_ptr;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QNearFieldManager::TargetAccessModes)

QT_END_NAMESPACE

#endif // QNEARFIELDMANAGER_H
