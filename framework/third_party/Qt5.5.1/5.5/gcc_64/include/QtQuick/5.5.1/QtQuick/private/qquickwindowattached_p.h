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

#ifndef QQUICKWINDOW_ATTACHED_P_H
#define QQUICKWINDOW_ATTACHED_P_H

#include <qqml.h>
#include <QWindow>

QT_BEGIN_NAMESPACE

class QQuickItem;
class QQuickWindow;

class Q_AUTOTEST_EXPORT QQuickWindowAttached : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QWindow::Visibility visibility READ visibility NOTIFY visibilityChanged)
    Q_PROPERTY(bool active READ isActive NOTIFY activeChanged)
    Q_PROPERTY(QQuickItem* activeFocusItem READ activeFocusItem NOTIFY activeFocusItemChanged)
    Q_PROPERTY(QQuickItem* contentItem READ contentItem NOTIFY contentItemChanged)
    Q_PROPERTY(int width READ width NOTIFY widthChanged)
    Q_PROPERTY(int height READ height NOTIFY heightChanged)

public:
    QQuickWindowAttached(QObject* attachee);

    QWindow::Visibility visibility() const;
    bool isActive() const;
    QQuickItem* activeFocusItem() const;
    QQuickItem* contentItem() const;
    int width() const;
    int height() const;

Q_SIGNALS:

    void visibilityChanged();
    void activeChanged();
    void activeFocusItemChanged();
    void contentItemChanged();
    void widthChanged();
    void heightChanged();

protected Q_SLOTS:
    void windowChanged(QQuickWindow*);

private:
    QQuickWindow* m_window;
    QQuickItem* m_attachee;
};

QT_END_NAMESPACE

#endif
