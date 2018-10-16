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

#ifndef QQUICKWINDOW_H
#define QQUICKWINDOW_H

#include <QtQuick/qtquickglobal.h>
#include <QtCore/qmetatype.h>
#include <QtGui/qopengl.h>
#include <QtGui/qwindow.h>
#include <QtGui/qevent.h>
#include <QtQml/qqml.h>

QT_BEGIN_NAMESPACE

class QRunnable;
class QQuickItem;
class QSGTexture;
class QInputMethodEvent;
class QQuickWindowPrivate;
class QQuickWindowAttached;
class QOpenGLFramebufferObject;
class QQmlIncubationController;
class QInputMethodEvent;
class QQuickCloseEvent;
class QQuickRenderControl;

class Q_QUICK_EXPORT QQuickWindow : public QWindow
{
    Q_OBJECT
    Q_PRIVATE_PROPERTY(QQuickWindow::d_func(), QQmlListProperty<QObject> data READ data DESIGNABLE false)
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)
    Q_PROPERTY(QQuickItem* contentItem READ contentItem CONSTANT)
    Q_PROPERTY(QQuickItem* activeFocusItem READ activeFocusItem NOTIFY activeFocusItemChanged REVISION 1)
    Q_CLASSINFO("DefaultProperty", "data")
    Q_DECLARE_PRIVATE(QQuickWindow)
public:
    enum CreateTextureOption {
        TextureHasAlphaChannel  = 0x0001,
        TextureHasMipmaps       = 0x0002,
        TextureOwnsGLTexture    = 0x0004,
        TextureCanUseAtlas      = 0x0008
    };

    enum RenderStage {
        BeforeSynchronizingStage,
        AfterSynchronizingStage,
        BeforeRenderingStage,
        AfterRenderingStage,
        AfterSwapStage
    };

    Q_DECLARE_FLAGS(CreateTextureOptions, CreateTextureOption)

    enum SceneGraphError {
        ContextNotAvailable = 1
    };
    Q_ENUMS(SceneGraphError)

    QQuickWindow(QWindow *parent = 0);
    explicit QQuickWindow(QQuickRenderControl *renderControl);

    virtual ~QQuickWindow();

    QQuickItem *contentItem() const;

    QQuickItem *activeFocusItem() const;
    QObject *focusObject() const Q_DECL_OVERRIDE;

    QQuickItem *mouseGrabberItem() const;

    bool sendEvent(QQuickItem *, QEvent *);

    QImage grabWindow();

    void setRenderTarget(QOpenGLFramebufferObject *fbo);
    QOpenGLFramebufferObject *renderTarget() const;

    void setRenderTarget(uint fboId, const QSize &size);
    uint renderTargetId() const;
    QSize renderTargetSize() const;

    void resetOpenGLState();

    QQmlIncubationController *incubationController() const;

#ifndef QT_NO_ACCESSIBILITY
    QAccessibleInterface *accessibleRoot() const Q_DECL_OVERRIDE;
#endif

    // Scene graph specific functions
    QSGTexture *createTextureFromImage(const QImage &image) const;
    QSGTexture *createTextureFromImage(const QImage &image, CreateTextureOptions options) const;
    QSGTexture *createTextureFromId(uint id, const QSize &size, CreateTextureOptions options = CreateTextureOption(0)) const;

    void setClearBeforeRendering(bool enabled);
    bool clearBeforeRendering() const;

    void setColor(const QColor &color);
    QColor color() const;

    static bool hasDefaultAlphaBuffer();
    static void setDefaultAlphaBuffer(bool useAlpha);

    void setPersistentOpenGLContext(bool persistent);
    bool isPersistentOpenGLContext() const;

    void setPersistentSceneGraph(bool persistent);
    bool isPersistentSceneGraph() const;

    QOpenGLContext *openglContext() const;
    bool isSceneGraphInitialized() const;

    void scheduleRenderJob(QRunnable *job, RenderStage schedule);

    qreal effectiveDevicePixelRatio() const;

Q_SIGNALS:
    void frameSwapped();
    Q_REVISION(2) void openglContextCreated(QOpenGLContext *context);
    void sceneGraphInitialized();
    void sceneGraphInvalidated();
    void beforeSynchronizing();
    Q_REVISION(2) void afterSynchronizing();
    void beforeRendering();
    void afterRendering();
    Q_REVISION(2) void afterAnimating();
    Q_REVISION(2) void sceneGraphAboutToStop();

    Q_REVISION(1) void closing(QQuickCloseEvent *close);
    void colorChanged(const QColor &);
    Q_REVISION(1) void activeFocusItemChanged();
    Q_REVISION(2) void sceneGraphError(QQuickWindow::SceneGraphError error, const QString &message);


public Q_SLOTS:
    void update();
    void releaseResources();

protected:
    QQuickWindow(QQuickWindowPrivate &dd, QWindow *parent = 0);

    void exposeEvent(QExposeEvent *) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *) Q_DECL_OVERRIDE;

    void showEvent(QShowEvent *) Q_DECL_OVERRIDE;
    void hideEvent(QHideEvent *) Q_DECL_OVERRIDE;
    // TODO Qt 6: reimplement QWindow::closeEvent to emit closing

    void focusInEvent(QFocusEvent *) Q_DECL_OVERRIDE;
    void focusOutEvent(QFocusEvent *) Q_DECL_OVERRIDE;

    bool event(QEvent *) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *) Q_DECL_OVERRIDE;
#ifndef QT_NO_WHEELEVENT
    void wheelEvent(QWheelEvent *) Q_DECL_OVERRIDE;
#endif

private Q_SLOTS:
    void maybeUpdate();
    void cleanupSceneGraph();
    void forcePolish();
    void setTransientParent_helper(QQuickWindow *window);
    void runJobsAfterSwap();

private:
    friend class QQuickItem;
    friend class QQuickWidget;
    friend class QQuickRenderControl;
    friend class QQuickAnimatorController;
    Q_DISABLE_COPY(QQuickWindow)
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QQuickWindow *)

#endif // QQUICKWINDOW_H

