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

/*
    This file was originally created by qdbusxml2cpp version 0.8
    Command line was:
    qdbusxml2cpp -p qxdgnotificationproxy ../../3rdparty/dbus-ifaces/org.freedesktop.Notifications.xml

    However it is maintained manually.

    It is also not part of the public API. This header file may change from
    version to version without notice, or even be removed.
*/

#ifndef QXDGNOTIFICATIONPROXY_P_H
#define QXDGNOTIFICATIONPROXY_P_H

#include <QtCore/QObject>
#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QVariant>
#include <QtDBus/QtDBus>

QT_BEGIN_NAMESPACE

Q_DECLARE_LOGGING_CATEGORY(qLcTray)

/*
 * Proxy class for interface org.freedesktop.Notifications
 */
class QXdgNotificationInterface: public QDBusAbstractInterface
{
    Q_OBJECT
public:
    static inline const char *staticInterfaceName()
    { return "org.freedesktop.Notifications"; }

public:
    QXdgNotificationInterface(const QString &service, const QString &path,
                              const QDBusConnection &connection, QObject *parent = 0);

    ~QXdgNotificationInterface();

public Q_SLOTS: // METHODS
    inline QDBusPendingReply<> closeNotification(uint id)
    {
        QList<QVariant> argumentList;
        argumentList << QVariant::fromValue(id);
        return asyncCallWithArgumentList(QStringLiteral("CloseNotification"), argumentList);
    }

    inline QDBusPendingReply<QStringList> getCapabilities()
    {
        QList<QVariant> argumentList;
        return asyncCallWithArgumentList(QStringLiteral("GetCapabilities"), argumentList);
    }

    inline QDBusPendingReply<QString, QString, QString, QString> getServerInformation()
    {
        QList<QVariant> argumentList;
        return asyncCallWithArgumentList(QStringLiteral("GetServerInformation"), argumentList);
    }
    inline QDBusReply<QString> getServerInformation(QString &vendor, QString &version, QString &specVersion)
    {
        QList<QVariant> argumentList;
        QDBusMessage reply = callWithArgumentList(QDBus::Block, QStringLiteral("GetServerInformation"), argumentList);
        if (reply.type() == QDBusMessage::ReplyMessage && reply.arguments().count() == 4) {
            vendor = qdbus_cast<QString>(reply.arguments().at(1));
            version = qdbus_cast<QString>(reply.arguments().at(2));
            specVersion = qdbus_cast<QString>(reply.arguments().at(3));
        }
        return reply;
    }

    // see https://developer.gnome.org/notification-spec/#basic-design
    inline QDBusPendingReply<uint> notify(const QString &appName, uint replacesId, const QString &appIcon,
                                          const QString &summary, const QString &body, const QStringList &actions,
                                          const QVariantMap &hints, int timeout)
    {
        qCDebug(qLcTray) << appName << replacesId << appIcon << summary << body << actions << hints << timeout;
        QList<QVariant> argumentList;
        argumentList << QVariant::fromValue(appName) << QVariant::fromValue(replacesId) <<
                        QVariant::fromValue(appIcon) << QVariant::fromValue(summary) <<
                        QVariant::fromValue(body) << QVariant::fromValue(actions) <<
                        QVariant::fromValue(hints) << QVariant::fromValue(timeout);
        return asyncCallWithArgumentList(QStringLiteral("Notify"), argumentList);
    }

Q_SIGNALS:
    void ActionInvoked(uint id, const QString &action_key);
    void NotificationClosed(uint id, uint reason);
};

namespace org {
  namespace freedesktop {
    typedef ::QXdgNotificationInterface Notifications;
  }
}

QT_END_NAMESPACE

#endif
