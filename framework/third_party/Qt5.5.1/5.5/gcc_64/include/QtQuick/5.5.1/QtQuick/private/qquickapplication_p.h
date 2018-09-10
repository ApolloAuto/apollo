/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
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

#ifndef QQUICKAPPLICATION_P_H
#define QQUICKAPPLICATION_P_H

#include <QtCore/QObject>
#include <qqml.h>
#include <QtQml/private/qqmlglobal_p.h>
#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE


class Q_AUTOTEST_EXPORT QQuickApplication : public QQmlApplication
{
    Q_OBJECT
    Q_PROPERTY(bool active READ active NOTIFY activeChanged) // deprecated, use 'state' instead
    Q_PROPERTY(Qt::LayoutDirection layoutDirection READ layoutDirection NOTIFY layoutDirectionChanged)
    Q_PROPERTY(bool supportsMultipleWindows READ supportsMultipleWindows CONSTANT)
    Q_PROPERTY(Qt::ApplicationState state READ state NOTIFY stateChanged)

public:
    explicit QQuickApplication(QObject *parent = 0);
    virtual ~QQuickApplication();
    bool active() const;
    Qt::LayoutDirection layoutDirection() const;
    bool supportsMultipleWindows() const;
    Qt::ApplicationState state() const;

Q_SIGNALS:
    void activeChanged();
    void layoutDirectionChanged();
    void stateChanged(Qt::ApplicationState state);

private:
    Q_DISABLE_COPY(QQuickApplication)
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickApplication)

#endif // QQUICKAPPLICATION_P_H
