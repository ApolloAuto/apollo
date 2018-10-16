/****************************************************************************
**
** Copyright (C) 2014 Kurt Pattyn <pattyn.kurt@gmail.com>.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWebSockets module of the Qt Toolkit.
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
#ifndef QWEBSOCKETCORSAUTHENTICATOR_H
#define QWEBSOCKETCORSAUTHENTICATOR_H

#include "QtWebSockets/qwebsockets_global.h"
#include <QtCore/QScopedPointer>

QT_BEGIN_NAMESPACE

class QWebSocketCorsAuthenticatorPrivate;

class Q_WEBSOCKETS_EXPORT QWebSocketCorsAuthenticator
{
    Q_DECLARE_PRIVATE(QWebSocketCorsAuthenticator)

public:
    explicit QWebSocketCorsAuthenticator(const QString &origin);
    ~QWebSocketCorsAuthenticator();
    explicit QWebSocketCorsAuthenticator(const QWebSocketCorsAuthenticator &other);

#ifdef Q_COMPILER_RVALUE_REFS
    QWebSocketCorsAuthenticator(QWebSocketCorsAuthenticator &&other);
    QWebSocketCorsAuthenticator &operator =(QWebSocketCorsAuthenticator &&other);
#endif

    void swap(QWebSocketCorsAuthenticator &other);

    QWebSocketCorsAuthenticator &operator =(const QWebSocketCorsAuthenticator &other);

    QString origin() const;

    void setAllowed(bool allowed);
    bool allowed() const;

private:
    QScopedPointer<QWebSocketCorsAuthenticatorPrivate> d_ptr;
};

QT_END_NAMESPACE

#endif // QWEBSOCKETCORSAUTHENTICATOR_H
