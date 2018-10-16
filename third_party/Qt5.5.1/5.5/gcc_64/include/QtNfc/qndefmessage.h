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

#ifndef QNDEFMESSAGE_H
#define QNDEFMESSAGE_H

#include <QtCore/QVector>
#include <QtCore/QSet>
#include <QtCore/QList>
#include <QtCore/QMetaType>
#include <QtNfc/qnfcglobal.h>
#include <QtNfc/QNdefRecord>

QT_BEGIN_NAMESPACE

class Q_NFC_EXPORT QNdefMessage : public QList<QNdefRecord>
{
public:
    inline QNdefMessage() { }
    inline explicit QNdefMessage(const QNdefRecord &record) { append(record); }
    inline QNdefMessage(const QNdefMessage &message) : QList<QNdefRecord>(message) { }
    inline QNdefMessage(const QList<QNdefRecord> &records) : QList<QNdefRecord>(records) { }

    bool operator==(const QNdefMessage &other) const;

    QByteArray toByteArray() const;

    static QNdefMessage fromByteArray(const QByteArray &message);
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QNdefMessage)

#endif // QNDEFMESSAGE_H
