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

#ifndef QSGCONTEXT_H
#define QSGCONTEXT_H

#include <QtCore/QObject>
#include <QtCore/qabstractanimation.h>

#include <QtGui/QImage>
#include <QtGui/QSurfaceFormat>

#include <private/qtquickglobal_p.h>
#include <private/qrawfont_p.h>

#include <QtQuick/qsgnode.h>
#include <QtQuick/private/qsgdepthstencilbuffer_p.h>

QT_BEGIN_NAMESPACE

namespace QSGAtlasTexture {
    class Manager;
}

class QSGContextPrivate;
class QSGRectangleNode;
class QSGImageNode;
class QSGPainterNode;
class QSGGlyphNode;
class QSGNinePatchNode;
class QSGRenderer;
class QSGDistanceFieldGlyphCache;
class QQuickWindow;
class QSGTexture;
class QSGMaterial;
class QSGMaterialShader;
class QSGRenderLoop;
class QSGLayer;

class QOpenGLContext;
class QOpenGLFramebufferObject;

class QQuickTextureFactory;
class QSGDistanceFieldGlyphCacheManager;
class QSGContext;
class QQuickPaintedItem;

Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_TIME_RENDERLOOP)
Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_TIME_COMPILATION)
Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_TIME_TEXTURE)
Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_TIME_GLYPH)
Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_TIME_RENDERER)

Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_INFO)
Q_DECLARE_LOGGING_CATEGORY(QSG_LOG_RENDERLOOP)

class Q_QUICK_PRIVATE_EXPORT QSGRenderContext : public QObject
{
    Q_OBJECT
public:
    QSGRenderContext(QSGContext *context);
    ~QSGRenderContext();

    QOpenGLContext *openglContext() const { return m_gl; }
    QSGContext *sceneGraphContext() const { return m_sg; }
    virtual bool isValid() const { return m_gl; }

    virtual void initialize(QOpenGLContext *context);
    virtual void invalidate();

    virtual void renderNextFrame(QSGRenderer *renderer, GLuint fboId);
    virtual void endSync();

    virtual QSharedPointer<QSGDepthStencilBuffer> depthStencilBufferForFbo(QOpenGLFramebufferObject *fbo);
    QSGDepthStencilBufferManager *depthStencilBufferManager();

    virtual QSGDistanceFieldGlyphCache *distanceFieldGlyphCache(const QRawFont &font);
    QSGTexture *textureForFactory(QQuickTextureFactory *factory, QQuickWindow *window);

    virtual QSGTexture *createTexture(const QImage &image) const;
    virtual QSGTexture *createTextureNoAtlas(const QImage &image) const;
    virtual QSGRenderer *createRenderer();

    virtual void compile(QSGMaterialShader *shader, QSGMaterial *material, const char *vertexCode = 0, const char *fragmentCode = 0);
    virtual void initialize(QSGMaterialShader *shader);

    void setAttachToGLContext(bool attach);
    void registerFontengineForCleanup(QFontEngine *engine);

    static QSGRenderContext *from(QOpenGLContext *context);

    bool hasBrokenIndexBufferObjects() const { return m_brokenIBOs; }
    int maxTextureSize() const { return m_maxTextureSize; }

Q_SIGNALS:
    void initialized();
    void invalidated();

public Q_SLOTS:
    void textureFactoryDestroyed(QObject *o);

protected:
    QOpenGLContext *m_gl;
    QSGContext *m_sg;

    QMutex m_mutex;
    QHash<QQuickTextureFactory *, QSGTexture *> m_textures;
    QSet<QSGTexture *> m_texturesToDelete;
    QSGAtlasTexture::Manager *m_atlasManager;

    QSGDepthStencilBufferManager *m_depthStencilManager;
    QSGDistanceFieldGlyphCacheManager *m_distanceFieldCacheManager;

    QSet<QFontEngine *> m_fontEnginesToClean;
    int m_maxTextureSize;
    bool m_brokenIBOs;
    bool m_serializedRender;
    bool m_attachToGLContext;
};


class Q_QUICK_PRIVATE_EXPORT QSGContext : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QSGContext)

public:
    enum AntialiasingMethod {
        UndecidedAntialiasing,
        VertexAntialiasing,
        MsaaAntialiasing
    };

    explicit QSGContext(QObject *parent = 0);
    ~QSGContext();

    virtual void renderContextInitialized(QSGRenderContext *renderContext);
    virtual void renderContextInvalidated(QSGRenderContext *renderContext);
    virtual QSGRenderContext *createRenderContext();

    QSGRectangleNode *createRectangleNode(const QRectF &rect, const QColor &c);
    virtual QSGRectangleNode *createRectangleNode();
    virtual QSGImageNode *createImageNode();
    virtual QSGPainterNode *createPainterNode(QQuickPaintedItem *item);
    virtual QSGGlyphNode *createGlyphNode(QSGRenderContext *rc, bool preferNativeGlyphNode);
    virtual QSGNinePatchNode *createNinePatchNode();
    virtual QSGLayer *createLayer(QSGRenderContext *renderContext);
    virtual QAnimationDriver *createAnimationDriver(QObject *parent);

    virtual QSize minimumFBOSize() const;
    virtual QSurfaceFormat defaultSurfaceFormat() const;

    void setDistanceFieldEnabled(bool enabled);
    bool isDistanceFieldEnabled() const;

    static QSGContext *createDefaultContext();
    static QQuickTextureFactory *createTextureFactoryFromImage(const QImage &image);
    static QSGRenderLoop *createWindowManager();
};

QT_END_NAMESPACE

#endif // QSGCONTEXT_H
