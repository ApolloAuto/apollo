/****************************************************************************
**
** Copyright (C) 2009 Marco Martin <notmart@gmail.com>
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QDBUSTRAYTYPES_P_H
#define QDBUSTRAYTYPES_P_H

#ifndef QT_NO_SYSTEMTRAYICON

#include <QObject>
#include <QString>
#include <QDBusArgument>
#include <QDBusConnection>
#include <QDBusObjectPath>
#include <QPixmap>

QT_BEGIN_NAMESPACE

// Custom message type to send icons across D-Bus
struct QXdgDBusImageStruct
{
    QXdgDBusImageStruct() { }
    QXdgDBusImageStruct(int w, int h)
        : width(w), height(h), data(width * height * 4, 0) { }
    int width;
    int height;
    QByteArray data;
};

typedef QVector<QXdgDBusImageStruct> QXdgDBusImageVector;

QXdgDBusImageVector iconToQXdgDBusImageVector(const QIcon &icon);

// Custom message type to send tooltips across D-Bus
struct QXdgDBusToolTipStruct
{
    QString icon;
    QXdgDBusImageVector image;
    QString title;
    QString subTitle;
};

const QDBusArgument &operator<<(QDBusArgument &argument, const QXdgDBusImageStruct &icon);
const QDBusArgument &operator>>(const QDBusArgument &argument, QXdgDBusImageStruct &icon);

const QDBusArgument &operator<<(QDBusArgument &argument, const QXdgDBusImageVector &iconVector);
const QDBusArgument &operator>>(const QDBusArgument &argument, QXdgDBusImageVector &iconVector);

const QDBusArgument &operator<<(QDBusArgument &argument, const QXdgDBusToolTipStruct &toolTip);
const QDBusArgument &operator>>(const QDBusArgument &argument, QXdgDBusToolTipStruct &toolTip);

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QXdgDBusImageStruct)
Q_DECLARE_METATYPE(QXdgDBusImageVector)
Q_DECLARE_METATYPE(QXdgDBusToolTipStruct)

#endif // QT_NO_SYSTEMTRAYICON
#endif // QDBUSTRAYTYPES_P_H
