/****************************************************************************
**
** Copyright (C) 2014 Jolla Ltd, author: <gunnar.sletta@jollamobile.com>
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKITEMGRABRESULT_H
#define QQUICKITEMGRABRESULT_H

#include <QtCore/QObject>
#include <QtCore/QSize>
#include <QtCore/QUrl>
#include <QtGui/QImage>
#include <QtQml/QJSValue>
#include <QtQuick/qtquickglobal.h>

QT_BEGIN_NAMESPACE

class QImage;

class QQuickItemGrabResultPrivate;

class Q_QUICK_EXPORT QQuickItemGrabResult : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickItemGrabResult)

    Q_PROPERTY(QImage image READ image CONSTANT)
    Q_PROPERTY(QUrl url READ url CONSTANT)
public:
    QImage image() const;
    QUrl url() const;

    Q_INVOKABLE bool saveToFile(const QString &fileName);

protected:
    bool event(QEvent *);

Q_SIGNALS:
    void ready();

private Q_SLOTS:
    void setup();
    void render();

private:
    friend class QQuickItem;

    QQuickItemGrabResult(QObject *parent = 0);
};

QT_END_NAMESPACE

#endif
