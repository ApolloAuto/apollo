/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQMLDEBUGSERVICE_H
#define QQMLDEBUGSERVICE_H

#include <QtCore/qobject.h>
#include <QtCore/QDataStream>

#include <private/qtqmlglobal_p.h>

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

QT_BEGIN_NAMESPACE

class QQmlEngine;

class QQmlDebugServicePrivate;
class Q_QML_PRIVATE_EXPORT QQmlDebugService : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQmlDebugService)
    Q_DISABLE_COPY(QQmlDebugService)

public:
    explicit QQmlDebugService(const QString &, float version, QObject *parent = 0);
    ~QQmlDebugService();

    QString name() const;
    float version() const;

    enum State { NotConnected, Unavailable, Enabled };
    State state() const;

    void sendMessage(const QByteArray &);
    void sendMessages(const QList<QByteArray> &);

    static int idForObject(QObject *);
    static QObject *objectForId(int);
    static QList<QObject*> objectForLocationInfo(const QString &filename,
                                          int lineNumber, int columnNumber);
    static void removeInvalidObjectsFromHash();
    static void clearObjectsFromHash();

    static QString objectToString(QObject *obj);

    static bool isDebuggingEnabled();
    static bool hasDebuggingClient();
    static bool blockingMode();

protected:
    QQmlDebugService(QQmlDebugServicePrivate &dd, const QString &name, float version, QObject *parent = 0);

    State registerService();

    virtual void stateAboutToBeChanged(State);
    virtual void stateChanged(State);
    virtual void messageReceived(const QByteArray &);

    virtual void engineAboutToBeAdded(QQmlEngine *);
    virtual void engineAboutToBeRemoved(QQmlEngine *);
    virtual void engineAdded(QQmlEngine *);
    virtual void engineRemoved(QQmlEngine *);

signals:
    void attachedToEngine(QQmlEngine *);
    void detachedFromEngine(QQmlEngine *);

private:
    friend class QQmlDebugServer;
    friend class QQmlDebugServerPrivate;
};

class Q_QML_PRIVATE_EXPORT QQmlDebugStream : public QDataStream
{
public:
    QQmlDebugStream();
    explicit QQmlDebugStream(QIODevice *d);
    QQmlDebugStream(QByteArray *ba, QIODevice::OpenMode flags);
    QQmlDebugStream(const QByteArray &ba);
};

QT_END_NAMESPACE

#endif // QQMLDEBUGSERVICE_H

