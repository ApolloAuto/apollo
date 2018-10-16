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

#ifndef QQUICKCANVASITEM_P_H
#define QQUICKCANVASITEM_P_H

#include <QtQuick/qquickitem.h>
#include <private/qv8engine_p.h>
#include <QtCore/QThread>
#include <QtGui/QImage>

QT_BEGIN_NAMESPACE

class QQuickCanvasContext;

class QQuickCanvasItemPrivate;
class QQuickPixmap;

class QQuickCanvasPixmap : public QQmlRefCount
{
public:
    QQuickCanvasPixmap(const QImage& image);
    QQuickCanvasPixmap(QQuickPixmap *pixmap);
    ~QQuickCanvasPixmap();

    QImage image();

    qreal width() const;
    qreal height() const;
    bool isValid() const;
    QQuickPixmap *pixmap() const { return m_pixmap;}

private:
    QQuickPixmap *m_pixmap;
    QImage m_image;
};

class QQuickCanvasItem : public QQuickItem
{
    Q_OBJECT
    Q_ENUMS(RenderTarget)
    Q_ENUMS(RenderStrategy)

    Q_PROPERTY(bool available READ isAvailable NOTIFY availableChanged)
    Q_PROPERTY(QString contextType READ contextType WRITE setContextType NOTIFY contextTypeChanged)
    Q_PROPERTY(QQmlV4Handle context READ context NOTIFY contextChanged)
    Q_PROPERTY(QSizeF canvasSize READ canvasSize WRITE setCanvasSize NOTIFY canvasSizeChanged)
    Q_PROPERTY(QSize tileSize READ tileSize WRITE setTileSize NOTIFY tileSizeChanged)
    Q_PROPERTY(QRectF canvasWindow READ canvasWindow WRITE setCanvasWindow NOTIFY canvasWindowChanged)
    Q_PROPERTY(RenderTarget renderTarget READ renderTarget WRITE setRenderTarget NOTIFY renderTargetChanged)
    Q_PROPERTY(RenderStrategy renderStrategy READ renderStrategy WRITE setRenderStrategy NOTIFY renderStrategyChanged)

public:
    enum RenderTarget {
        Image,
        FramebufferObject
    };

    enum RenderStrategy {
        Immediate,
        Threaded,
        Cooperative
    };

    QQuickCanvasItem(QQuickItem *parent = 0);
    ~QQuickCanvasItem();

    bool isAvailable() const;

    QString contextType() const;
    void setContextType(const QString &contextType);

    QQmlV4Handle context() const;

    QSizeF canvasSize() const;
    void setCanvasSize(const QSizeF &);

    QSize tileSize() const;
    void setTileSize(const QSize &);

    QRectF canvasWindow() const;
    void setCanvasWindow(const QRectF& rect);

    RenderTarget renderTarget() const;
    void setRenderTarget(RenderTarget target);

    RenderStrategy renderStrategy() const;
    void setRenderStrategy(RenderStrategy strategy);

    QQuickCanvasContext *rawContext() const;

    QImage toImage(const QRectF& rect = QRectF()) const;

    Q_INVOKABLE void getContext(QQmlV4Function *args);

    Q_INVOKABLE void requestAnimationFrame(QQmlV4Function *args);
    Q_INVOKABLE void cancelRequestAnimationFrame(QQmlV4Function *args);

    Q_INVOKABLE void requestPaint();
    Q_INVOKABLE void markDirty(const QRectF& dirtyRect = QRectF());

    Q_INVOKABLE bool save(const QString &filename) const;
    Q_INVOKABLE QString toDataURL(const QString& type = QLatin1String("image/png")) const;
    QQmlRefPointer<QQuickCanvasPixmap> loadedPixmap(const QUrl& url);

    bool isTextureProvider() const Q_DECL_OVERRIDE;
    QSGTextureProvider *textureProvider() const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void paint(const QRect &region);
    void painted();
    void availableChanged();
    void contextTypeChanged();
    void contextChanged();
    void canvasSizeChanged();
    void tileSizeChanged();
    void canvasWindowChanged();
    void renderTargetChanged();
    void renderStrategyChanged();
    void imageLoaded();

public Q_SLOTS:
    void loadImage(const QUrl& url);
    void unloadImage(const QUrl& url);
    bool isImageLoaded(const QUrl& url) const;
    bool isImageLoading(const QUrl& url) const;
    bool isImageError(const QUrl& url) const;

private Q_SLOTS:
    void sceneGraphInitialized();
    void checkAnimationCallbacks();
    void invalidateSceneGraph();

protected:
    void componentComplete() Q_DECL_OVERRIDE;
    void itemChange(QQuickItem::ItemChange, const QQuickItem::ItemChangeData &) Q_DECL_OVERRIDE;
    void updatePolish() Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;
    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void releaseResources() Q_DECL_OVERRIDE;
private:
    Q_DECLARE_PRIVATE(QQuickCanvasItem)
    Q_INVOKABLE void delayedCreate();
    bool createContext(const QString &contextType);
    void initializeContext(QQuickCanvasContext *context, const QVariantMap &args = QVariantMap());
    QRect tiledRect(const QRectF &window, const QSize &tileSize);
    bool isPaintConnected();
};

class QQuickContext2DRenderThread : public QThread
{
    Q_OBJECT
public:
    QQuickContext2DRenderThread(QQmlEngine *eng);
    ~QQuickContext2DRenderThread();

    static QQuickContext2DRenderThread *instance(QQmlEngine *engine);

private:
    QQmlEngine *m_engine;
    QObject *m_eventLoopQuitHack;
    static QHash<QQmlEngine *,QQuickContext2DRenderThread*> renderThreads;
    static QMutex renderThreadsMutex;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickCanvasItem)

#endif //QQUICKCANVASITEM_P_H
