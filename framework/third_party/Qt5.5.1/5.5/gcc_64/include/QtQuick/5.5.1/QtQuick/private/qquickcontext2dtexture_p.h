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

#ifndef QQUICKCONTEXT2DTEXTURE_P_H
#define QQUICKCONTEXT2DTEXTURE_P_H

#include <QtQuick/qsgtexture.h>
#include "qquickcanvasitem_p.h"
#include "qquickcontext2d_p.h"

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>

#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>
#include <QtCore/QThread>

QT_BEGIN_NAMESPACE

class QQuickContext2DTile;
class QQuickContext2DCommandBuffer;

class QQuickContext2DTexture : public QObject
{
    Q_OBJECT
public:
    class PaintEvent : public QEvent {
    public:
        PaintEvent(QQuickContext2DCommandBuffer *b) : QEvent(QEvent::Type(QEvent::User + 1)), buffer(b) {}
        QQuickContext2DCommandBuffer *buffer;
    };

    class CanvasChangeEvent : public QEvent {
    public:
        CanvasChangeEvent(const QSize &cSize,
                          const QSize &tSize,
                          const QRect &cWindow,
                          const QRect &dRect,
                          bool sm,
                          bool aa)
            : QEvent(QEvent::Type(QEvent::User + 2))
            , canvasSize(cSize)
            , tileSize(tSize)
            , canvasWindow(cWindow)
            , dirtyRect(dRect)
            , smooth(sm)
            , antialiasing(aa)
        {
        }
        QSize canvasSize;
        QSize tileSize;
        QRect canvasWindow;
        QRect dirtyRect;
        bool smooth;
        bool antialiasing;
    };

    QQuickContext2DTexture();
    ~QQuickContext2DTexture();

    virtual QQuickCanvasItem::RenderTarget renderTarget() const = 0;
    static QRect tiledRect(const QRectF& window, const QSize& tileSize);

    bool setCanvasSize(const QSize &size);
    bool setTileSize(const QSize &size);
    bool setCanvasWindow(const QRect& canvasWindow);
    void setSmooth(bool smooth);
    void setAntialiasing(bool antialiasing);
    bool setDirtyRect(const QRect &dirtyRect);
    bool canvasDestroyed();
    void setOnCustomThread(bool is) { m_onCustomThread = is; }
    bool isOnCustomThread() const { return m_onCustomThread; }

    // Called during sync() on the scene graph thread while GUI is blocked.
    virtual QSGTexture *textureForNextFrame(QSGTexture *lastFrame, QQuickWindow *window) = 0;
    bool event(QEvent *e);

    void initializeOpenGL(QOpenGLContext *gl, QOffscreenSurface *s) {
        m_gl = gl;
        m_surface = s;
    }

Q_SIGNALS:
    void textureChanged();

public Q_SLOTS:
    void canvasChanged(const QSize& canvasSize, const QSize& tileSize, const QRect& canvasWindow, const QRect& dirtyRect, bool smooth, bool antialiasing);
    void paint(QQuickContext2DCommandBuffer *ccb);
    void markDirtyTexture();
    void setItem(QQuickCanvasItem* item);
    virtual void grabImage(const QRectF& region = QRectF()) = 0;

protected:
    virtual QVector2D scaleFactor() const { return QVector2D(1, 1); }

    void paintWithoutTiles(QQuickContext2DCommandBuffer *ccb);
    virtual QPaintDevice* beginPainting() {m_painting = true; return 0; }
    virtual void endPainting() {m_painting = false;}
    virtual QQuickContext2DTile* createTile() const = 0;
    virtual void compositeTile(QQuickContext2DTile* tile) = 0;

    void clearTiles();
    virtual QSize adjustedTileSize(const QSize &ts);
    QRect createTiles(const QRect& window);

    QList<QQuickContext2DTile*> m_tiles;
    QQuickContext2D *m_context;

    QOpenGLContext *m_gl;
    QSurface *m_surface;

    QQuickContext2D::State m_state;

    QQuickCanvasItem* m_item;
    QSize m_canvasSize;
    QSize m_tileSize;
    QRect m_canvasWindow;

    QMutex m_mutex;
    QWaitCondition m_condition;

    uint m_canvasWindowChanged : 1;
    uint m_dirtyTexture : 1;
    uint m_smooth : 1;
    uint m_antialiasing : 1;
    uint m_tiledCanvas : 1;
    uint m_painting : 1;
    uint m_onCustomThread : 1; // Not GUI and not SGRender
};

class QQuickContext2DFBOTexture : public QQuickContext2DTexture
{
    Q_OBJECT

public:
    QQuickContext2DFBOTexture();
    ~QQuickContext2DFBOTexture();
    QQuickContext2DTile* createTile() const Q_DECL_OVERRIDE;
    QPaintDevice* beginPainting() Q_DECL_OVERRIDE;
    void endPainting() Q_DECL_OVERRIDE;
    QRectF normalizedTextureSubRect() const;
    QQuickCanvasItem::RenderTarget renderTarget() const Q_DECL_OVERRIDE;
    void compositeTile(QQuickContext2DTile* tile) Q_DECL_OVERRIDE;
    QSize adjustedTileSize(const QSize &ts) Q_DECL_OVERRIDE;

    QSGTexture *textureForNextFrame(QSGTexture *, QQuickWindow *window) Q_DECL_OVERRIDE;

protected:
    QVector2D scaleFactor() const Q_DECL_OVERRIDE;

public Q_SLOTS:
    void grabImage(const QRectF& region = QRectF()) Q_DECL_OVERRIDE;

private:
    bool doMultisampling() const;
    QOpenGLFramebufferObject *m_fbo;
    QOpenGLFramebufferObject *m_multisampledFbo;
    QSize m_fboSize;
    QPaintDevice *m_paint_device;


    GLuint m_displayTextures[2];
    int m_displayTexture;
};

class QSGPlainTexture;
class QQuickContext2DImageTexture : public QQuickContext2DTexture
{
    Q_OBJECT

public:
    QQuickContext2DImageTexture();
    ~QQuickContext2DImageTexture();

    virtual QQuickCanvasItem::RenderTarget renderTarget() const;

    virtual QQuickContext2DTile* createTile() const;
    virtual QPaintDevice* beginPainting();
    virtual void endPainting();
    virtual void compositeTile(QQuickContext2DTile* tile);

    virtual QSGTexture *textureForNextFrame(QSGTexture *lastFrame, QQuickWindow *window);

public Q_SLOTS:
    virtual void grabImage(const QRectF& region = QRectF());

private:
    QImage m_image;
    QImage m_displayImage;
    QPainter m_painter;
};

QT_END_NAMESPACE

#endif // QQUICKCONTEXT2DTEXTURE_P_H
