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

#ifndef QSGWINDOWSRENDERLOOP_P_H
#define QSGWINDOWSRENDERLOOP_P_H

#include <QtCore/QObject>
#include <QtCore/QElapsedTimer>

#include <QtGui/QOpenGLContext>

#include "qsgrenderloop_p.h"

QT_BEGIN_NAMESPACE

class QSGRenderContext;

class QSGWindowsRenderLoop : public QSGRenderLoop
{
    Q_OBJECT
public:
    explicit QSGWindowsRenderLoop();
    ~QSGWindowsRenderLoop();

    void show(QQuickWindow *window);
    void hide(QQuickWindow *window);

    void windowDestroyed(QQuickWindow *window);

    void exposureChanged(QQuickWindow *window);
    QImage grab(QQuickWindow *window);

    void update(QQuickWindow *window);
    void maybeUpdate(QQuickWindow *window);

    QAnimationDriver *animationDriver() const { return m_animationDriver; }

    QSGContext *sceneGraphContext() const { return m_sg; }
    QSGRenderContext *createRenderContext(QSGContext *) const { return m_rc; }

    void releaseResources(QQuickWindow *) { }

    void render();
    void renderWindow(QQuickWindow *window);

    bool event(QEvent *event);
    bool anyoneShowing() const;

    bool interleaveIncubation() const;

public Q_SLOTS:
    void started();
    void stopped();

private:
    struct WindowData {
        QQuickWindow *window;
        bool pendingUpdate;
    };

    void handleObscurity();
    void maybePostUpdateTimer();
    WindowData *windowData(QQuickWindow *window);

    QList<WindowData> m_windows;

    QOpenGLContext *m_gl;
    QSGContext *m_sg;
    QSGRenderContext *m_rc;

    QAnimationDriver *m_animationDriver;

    int m_updateTimer;
    int m_animationTimer;

    int m_vsyncDelta;
};

QT_END_NAMESPACE

#endif // QSGWINDOWSRENDERLOOP_P_H
