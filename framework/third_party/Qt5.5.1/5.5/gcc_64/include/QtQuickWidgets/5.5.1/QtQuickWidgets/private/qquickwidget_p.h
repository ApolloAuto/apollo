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

#ifndef QQUICKWIDGET_P_H
#define QQUICKWIDGET_P_H

#include "qquickwidget.h"
#include <private/qwidget_p.h>

#include <QtCore/qurl.h>
#include <QtCore/qelapsedtimer.h>
#include <QtCore/qtimer.h>
#include <QtCore/qpointer.h>
#include <QtCore/QWeakPointer>

#include <QtQml/qqmlengine.h>

#include "private/qquickitemchangelistener_p.h"

QT_BEGIN_NAMESPACE

class QQmlContext;
class QQmlError;
class QQuickItem;
class QQmlComponent;
class QQuickRenderControl;
class QOpenGLContext;
class QOffscreenSurface;

class QQuickWidgetPrivate
        : public QWidgetPrivate,
          public QQuickItemChangeListener
{
    Q_DECLARE_PUBLIC(QQuickWidget)
public:
    static QQuickWidgetPrivate* get(QQuickWidget *view) { return view->d_func(); }
    static const QQuickWidgetPrivate* get(const QQuickWidget *view) { return view->d_func(); }

    QQuickWidgetPrivate();
    ~QQuickWidgetPrivate();

    void execute();
    void itemGeometryChanged(QQuickItem *item, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void initResize();
    void updateSize();
    void updatePosition();
    void updateFrambufferObjectSize();
    void setRootObject(QObject *);
    void render(bool needsSync);
    void renderSceneGraph();
    void createContext();
    void destroyContext();
    void handleContextCreationFailure(const QSurfaceFormat &format, bool isEs);

    QObject *focusObject() Q_DECL_OVERRIDE;

    GLuint textureId() const Q_DECL_OVERRIDE;
    QImage grabFramebuffer() Q_DECL_OVERRIDE;

    void init(QQmlEngine* e = 0);
    void handleWindowChange();
    void invalidateRenderControl();

    QSize rootObjectSize() const;

    QPointer<QQuickItem> root;

    QUrl source;

    QPointer<QQmlEngine> engine;
    QQmlComponent *component;
    QBasicTimer resizetimer;
    QQuickWindow *offscreenWindow;
    QOffscreenSurface *offscreenSurface;
    QQuickRenderControl *renderControl;
    QOpenGLFramebufferObject *fbo;
    QOpenGLFramebufferObject *resolvedFbo;
    QOpenGLContext *context;

    QQuickWidget::ResizeMode resizeMode;
    QSize initialSize;
    QElapsedTimer frameTimer;

    QBasicTimer updateTimer;
    bool eventPending;
    bool updatePending;
    bool fakeHidden;
};

QT_END_NAMESPACE

#endif // QQuickWidget_P_H
