/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2014 Klarälvdalens Datakonsult AB, a KDAB Group company, info@kdab.com, author Milian Wolff <milian.wolff@kdab.com>
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWebChannel module of the Qt Toolkit.
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

#ifndef QWEBCHANNEL_H
#define QWEBCHANNEL_H

#include <QObject>
#include <QJsonValue>

#include <QtWebChannel/qwebchannelglobal.h>

QT_BEGIN_NAMESPACE

class QWebChannelPrivate;
class QWebChannelAbstractTransport;

class Q_WEBCHANNEL_EXPORT QWebChannel : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(QWebChannel)
    Q_PROPERTY(bool blockUpdates READ blockUpdates WRITE setBlockUpdates NOTIFY blockUpdatesChanged)
public:
    explicit QWebChannel(QObject *parent = 0);
    ~QWebChannel();

    void registerObjects(const QHash<QString, QObject*> &objects);
    QHash<QString, QObject*> registeredObjects() const;
    Q_INVOKABLE void registerObject(const QString &id, QObject *object);
    Q_INVOKABLE void deregisterObject(QObject *object);

    bool blockUpdates() const;

    void setBlockUpdates(bool block);

Q_SIGNALS:
    void blockUpdatesChanged(bool block);

public Q_SLOTS:
    void connectTo(QWebChannelAbstractTransport *transport);
    void disconnectFrom(QWebChannelAbstractTransport *transport);

private:
    Q_DECLARE_PRIVATE(QWebChannel)
    QWebChannel(QWebChannelPrivate &dd, QObject *parent = 0);
    Q_PRIVATE_SLOT(d_func(), void _q_transportDestroyed(QObject*))

    friend class QMetaObjectPublisher;
    friend class QQmlWebChannel;
    friend class TestWebChannel;
};

QT_END_NAMESPACE

#endif // QWEBCHANNEL_H

