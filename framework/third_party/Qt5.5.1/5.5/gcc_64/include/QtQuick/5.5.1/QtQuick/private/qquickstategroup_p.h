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

#ifndef QQUICKSTATEGROUP_H
#define QQUICKSTATEGROUP_H

#include "qquickstate_p.h"

QT_BEGIN_NAMESPACE

class QQuickStateGroupPrivate;
class Q_QUICK_PRIVATE_EXPORT QQuickStateGroup : public QObject, public QQmlParserStatus
{
    Q_OBJECT
    Q_INTERFACES(QQmlParserStatus)
    Q_DECLARE_PRIVATE(QQuickStateGroup)

    Q_PROPERTY(QString state READ state WRITE setState NOTIFY stateChanged)
    Q_PROPERTY(QQmlListProperty<QQuickState> states READ statesProperty DESIGNABLE false)
    Q_PROPERTY(QQmlListProperty<QQuickTransition> transitions READ transitionsProperty DESIGNABLE false)

public:
    QQuickStateGroup(QObject * = 0);
    virtual ~QQuickStateGroup();

    QString state() const;
    void setState(const QString &);

    QQmlListProperty<QQuickState> statesProperty();
    QList<QQuickState *> states() const;

    QQmlListProperty<QQuickTransition> transitionsProperty();

    QQuickState *findState(const QString &name) const;
    void removeState(QQuickState *state);

    virtual void classBegin();
    virtual void componentComplete();
Q_SIGNALS:
    void stateChanged(const QString &);

private:
    friend class QQuickState;
    friend class QQuickStatePrivate;
    bool updateAutoState();
    void stateAboutToComplete();
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickStateGroup)

#endif // QQUICKSTATEGROUP_H
