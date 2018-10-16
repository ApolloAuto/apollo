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

#ifndef QQMLINCUBATOR_H
#define QQMLINCUBATOR_H

#include <QtQml/qtqmlglobal.h>
#include <QtQml/qqmlerror.h>

QT_BEGIN_NAMESPACE


class QQmlEngine;

class QQmlIncubatorPrivate;
class Q_QML_EXPORT QQmlIncubator
{
    Q_DISABLE_COPY(QQmlIncubator)
public:
    enum IncubationMode {
        Asynchronous,
        AsynchronousIfNested,
        Synchronous
    };
    enum Status {
        Null,
        Ready,
        Loading,
        Error
    };

    QQmlIncubator(IncubationMode = Asynchronous);
    virtual ~QQmlIncubator();

    void clear();
    void forceCompletion();

    bool isNull() const;
    bool isReady() const;
    bool isError() const;
    bool isLoading() const;

    QList<QQmlError> errors() const;

    IncubationMode incubationMode() const;

    Status status() const;

    QObject *object() const;

protected:
    virtual void statusChanged(Status);
    virtual void setInitialState(QObject *);

private:
    friend class QQmlComponent;
    friend class QQmlEnginePrivate;
    friend class QQmlIncubatorPrivate;
    QQmlIncubatorPrivate *d;
};

class QQmlEnginePrivate;
class Q_QML_EXPORT QQmlIncubationController
{
    Q_DISABLE_COPY(QQmlIncubationController)
public:
    QQmlIncubationController();
    virtual ~QQmlIncubationController();

    QQmlEngine *engine() const;
    int incubatingObjectCount() const;

    void incubateFor(int msecs);
    void incubateWhile(volatile bool *flag, int msecs=0);

protected:
    virtual void incubatingObjectCountChanged(int);

private:
    friend class QQmlEngine;
    friend class QQmlEnginePrivate;
    friend class QQmlIncubatorPrivate;
    QQmlEnginePrivate *d;
};

QT_END_NAMESPACE

#endif // QQMLINCUBATOR_H
