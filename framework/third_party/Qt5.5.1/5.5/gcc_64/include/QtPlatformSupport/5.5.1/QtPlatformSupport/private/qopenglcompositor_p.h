/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
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

#ifndef QOPENGLCOMPOSITOR_H
#define QOPENGLCOMPOSITOR_H

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

#include <QtCore/QTimer>
#include <QtGui/private/qopengltextureblitter_p.h>

QT_BEGIN_NAMESPACE

class QOpenGLContext;
class QOpenGLFramebufferObject;
class QWindow;
class QPlatformTextureList;

class QOpenGLCompositorWindow
{
public:
    virtual QWindow *sourceWindow() const = 0;
    virtual const QPlatformTextureList *textures() const = 0;
    virtual void beginCompositing() { }
    virtual void endCompositing() { }
};

class QOpenGLCompositor : public QObject
{
    Q_OBJECT

public:
    static QOpenGLCompositor *instance();
    static void destroy();

    void setTarget(QOpenGLContext *context, QWindow *window);
    QOpenGLContext *context() const { return m_context; }
    QWindow *targetWindow() const { return m_targetWindow; }

    void update();
    QImage grab();

    QList<QOpenGLCompositorWindow *> windows() const { return m_windows; }
    void addWindow(QOpenGLCompositorWindow *window);
    void removeWindow(QOpenGLCompositorWindow *window);
    void moveToTop(QOpenGLCompositorWindow *window);
    void changeWindowIndex(QOpenGLCompositorWindow *window, int newIdx);

signals:
    void topWindowChanged(QOpenGLCompositorWindow *window);

private slots:
    void handleRenderAllRequest();

private:
    QOpenGLCompositor();
    ~QOpenGLCompositor();

    void renderAll(QOpenGLFramebufferObject *fbo);
    void render(QOpenGLCompositorWindow *window);

    QOpenGLContext *m_context;
    QWindow *m_targetWindow;
    QTimer m_updateTimer;
    QOpenGLTextureBlitter m_blitter;
    QList<QOpenGLCompositorWindow *> m_windows;
};

QT_END_NAMESPACE

#endif // QOPENGLCOMPOSITOR_H
