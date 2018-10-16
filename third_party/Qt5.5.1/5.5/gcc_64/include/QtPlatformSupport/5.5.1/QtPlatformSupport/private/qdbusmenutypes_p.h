/****************************************************************************
**
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

#ifndef QDBUSMENUTYPES_H
#define QDBUSMENUTYPES_H

#include <QObject>
#include <QString>
#include <QDBusArgument>
#include <QDBusConnection>
#include <QDBusObjectPath>
#include <QPixmap>

QT_BEGIN_NAMESPACE

class QDBusPlatformMenu;
class QDBusPlatformMenuItem;
class QDBusMenuItem;
typedef QList<QDBusMenuItem> QDBusMenuItemList;

class QDBusMenuItem
{
public:
    QDBusMenuItem() { }
    QDBusMenuItem(const QDBusPlatformMenuItem *item);

    static QDBusMenuItemList items(const QList<int> &ids, const QStringList &propertyNames);
    static QString convertMnemonic(const QString &label);
    static void registerDBusTypes();

    int m_id;
    QVariantMap m_properties;
};

const QDBusArgument &operator<<(QDBusArgument &arg, const QDBusMenuItem &item);
const QDBusArgument &operator>>(const QDBusArgument &arg, QDBusMenuItem &item);

class QDBusMenuItemKeys
{
public:

    int id;
    QStringList properties;
};

const QDBusArgument &operator<<(QDBusArgument &arg, const QDBusMenuItemKeys &keys);
const QDBusArgument &operator>>(const QDBusArgument &arg, QDBusMenuItemKeys &keys);

typedef QList<QDBusMenuItemKeys> QDBusMenuItemKeysList;

class QDBusMenuLayoutItem
{
public:
    uint populate(int id, int depth, const QStringList &propertyNames);
    void populate(const QDBusPlatformMenu *menu, int depth, const QStringList &propertyNames);
    void populate(const QDBusPlatformMenuItem *item, int depth, const QStringList &propertyNames);

    int m_id;
    QVariantMap m_properties;
    QList<QDBusMenuLayoutItem> m_children;
};

const QDBusArgument &operator<<(QDBusArgument &arg, const QDBusMenuLayoutItem &);
const QDBusArgument &operator>>(const QDBusArgument &arg, QDBusMenuLayoutItem &item);

typedef QList<QDBusMenuLayoutItem> QDBusMenuLayoutItemList;

class QDBusMenuEvent
{
public:
    int m_id;
    QString m_eventId;
    QDBusVariant m_data;
    uint m_timestamp;
};

const QDBusArgument &operator<<(QDBusArgument &arg, const QDBusMenuEvent &ev);
const QDBusArgument &operator>>(const QDBusArgument &arg, QDBusMenuEvent &ev);

typedef QList<QDBusMenuEvent> QDBusMenuEventList;

#ifndef QT_NO_DEBUG_STREAM
QDebug operator<<(QDebug d, const QDBusMenuItem &item);
QDebug operator<<(QDebug d, const QDBusMenuLayoutItem &item);
#endif

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QDBusMenuItem)
Q_DECLARE_METATYPE(QDBusMenuItemList)
Q_DECLARE_METATYPE(QDBusMenuItemKeys)
Q_DECLARE_METATYPE(QDBusMenuItemKeysList)
Q_DECLARE_METATYPE(QDBusMenuLayoutItem)
Q_DECLARE_METATYPE(QDBusMenuLayoutItemList)
Q_DECLARE_METATYPE(QDBusMenuEvent)
Q_DECLARE_METATYPE(QDBusMenuEventList)

#endif
