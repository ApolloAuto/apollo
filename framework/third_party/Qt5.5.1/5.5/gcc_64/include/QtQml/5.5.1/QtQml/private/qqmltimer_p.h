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

#ifndef QQMLTIMER_H
#define QQMLTIMER_H

#include <qqml.h>

#include <QtCore/qobject.h>

#include <private/qtqmlglobal_p.h>

QT_BEGIN_NAMESPACE

class QQmlTimerPrivate;
class Q_QML_PRIVATE_EXPORT QQmlTimer : public QObject, public QQmlParserStatus
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQmlTimer)
    Q_INTERFACES(QQmlParserStatus)
    Q_PROPERTY(int interval READ interval WRITE setInterval NOTIFY intervalChanged)
    Q_PROPERTY(bool running READ isRunning WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool repeat READ isRepeating WRITE setRepeating NOTIFY repeatChanged)
    Q_PROPERTY(bool triggeredOnStart READ triggeredOnStart WRITE setTriggeredOnStart NOTIFY triggeredOnStartChanged)
    Q_PROPERTY(QObject *parent READ parent CONSTANT)

public:
    QQmlTimer(QObject *parent=0);

    void setInterval(int interval);
    int interval() const;

    bool isRunning() const;
    void setRunning(bool running);

    bool isRepeating() const;
    void setRepeating(bool repeating);

    bool triggeredOnStart() const;
    void setTriggeredOnStart(bool triggeredOnStart);

protected:
    void classBegin();
    void componentComplete();

    bool event(QEvent *);

public Q_SLOTS:
    void start();
    void stop();
    void restart();

Q_SIGNALS:
    void triggered();
    void runningChanged();
    void intervalChanged();
    void repeatChanged();
    void triggeredOnStartChanged();

private:
    void update();

private Q_SLOTS:
    void ticked();
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQmlTimer)

#endif
