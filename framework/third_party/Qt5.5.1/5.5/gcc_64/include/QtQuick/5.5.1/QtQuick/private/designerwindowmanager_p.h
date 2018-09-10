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

#ifndef DESIGNERWINDOWMANAGER_P_H
#define DESIGNERWINDOWMANAGER_P_H

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

#include <QtCore/QScopedPointer>

#include <private/qsgrenderloop_p.h>
#include <private/qtquickglobal_p.h>
#include <QtQuick/private/qsgcontext_p.h>


QT_BEGIN_NAMESPACE

class QQuickWindow;
class QSGContext;
class QSGRenderContext;
class QAnimationDriver;
class QOpenGLContext;

class DesignerWindowManager : public QSGRenderLoop
{
    Q_OBJECT
public:
    DesignerWindowManager();

    void show(QQuickWindow *window);
    void hide(QQuickWindow *window);

    void windowDestroyed(QQuickWindow *window);

    void makeOpenGLContext(QQuickWindow *window);
    void exposureChanged(QQuickWindow *window);
    QImage grab(QQuickWindow *window);

    void maybeUpdate(QQuickWindow *window);
    void update(QQuickWindow *window); // identical for this implementation.

    void releaseResources(QQuickWindow *) { }

    QAnimationDriver *animationDriver() const { return 0; }

    QSGContext *sceneGraphContext() const;
    QSGRenderContext *createRenderContext(QSGContext *) const { return m_renderContext.data(); }

    static void createOpenGLContext(QQuickWindow *window);

private:
    QScopedPointer<QOpenGLContext> m_openGlContext;
    QScopedPointer<QSGContext> m_sgContext;
    QScopedPointer<QSGRenderContext> m_renderContext;
};

QT_END_NAMESPACE

#endif // DESIGNERWINDOWMANAGER_P_H
