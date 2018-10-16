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

#ifndef QSGADAPTATIONLAYER_P_H
#define QSGADAPTATIONLAYER_P_H

#include <QtQuick/qsgnode.h>
#include <QtQuick/qsgtexture.h>
#include <QtCore/qobject.h>
#include <QtCore/qrect.h>
#include <QtGui/qbrush.h>
#include <QtGui/qcolor.h>
#include <QtCore/qsharedpointer.h>
#include <QtGui/qglyphrun.h>
#include <QtCore/qurl.h>
#include <private/qfontengine_p.h>
#include <QtGui/private/qdatabuffer_p.h>
#include <private/qopenglcontext_p.h>
#include <private/qdistancefield_p.h>

// ### remove
#include <QtQuick/private/qquicktext_p.h>

QT_BEGIN_NAMESPACE

class QSGNode;
class QImage;
class TextureReference;
class QSGDistanceFieldGlyphCacheManager;
class QSGDistanceFieldGlyphNode;
class QOpenGLContext;
class QSGImageNode;
class QSGPainterNode;
class QSGRectangleNode;
class QSGGlyphNode;
class QSGNinePatchNode;
class QSGRootNode;

class Q_QUICK_PRIVATE_EXPORT QSGNodeVisitorEx
{
public:
    // visit(...) returns true if the children are supposed to be
    // visisted and false if they're supposed to be skipped by the visitor.

    virtual bool visit(QSGTransformNode *) = 0;
    virtual void endVisit(QSGTransformNode *) = 0;
    virtual bool visit(QSGClipNode *) = 0;
    virtual void endVisit(QSGClipNode *) = 0;
    virtual bool visit(QSGGeometryNode *) = 0;
    virtual void endVisit(QSGGeometryNode *) = 0;
    virtual bool visit(QSGOpacityNode *) = 0;
    virtual void endVisit(QSGOpacityNode *) = 0;
    virtual bool visit(QSGImageNode *) = 0;
    virtual void endVisit(QSGImageNode *) = 0;
    virtual bool visit(QSGPainterNode *) = 0;
    virtual void endVisit(QSGPainterNode *) = 0;
    virtual bool visit(QSGRectangleNode *) = 0;
    virtual void endVisit(QSGRectangleNode *) = 0;
    virtual bool visit(QSGGlyphNode *) = 0;
    virtual void endVisit(QSGGlyphNode *) = 0;
    virtual bool visit(QSGNinePatchNode *) = 0;
    virtual void endVisit(QSGNinePatchNode *) = 0;
    virtual bool visit(QSGRootNode *) = 0;
    virtual void endVisit(QSGRootNode *) = 0;

    void visitChildren(QSGNode *node);
};


class Q_QUICK_PRIVATE_EXPORT QSGVisitableNode : public QSGGeometryNode
{
public:
    QSGVisitableNode() { setFlag(IsVisitableNode); }

    virtual void accept(QSGNodeVisitorEx *) = 0;
};

class Q_QUICK_PRIVATE_EXPORT QSGRectangleNode : public QSGVisitableNode
{
public:
    virtual void setRect(const QRectF &rect) = 0;
    virtual void setColor(const QColor &color) = 0;
    virtual void setPenColor(const QColor &color) = 0;
    virtual void setPenWidth(qreal width) = 0;
    virtual void setGradientStops(const QGradientStops &stops) = 0;
    virtual void setRadius(qreal radius) = 0;
    virtual void setAntialiasing(bool antialiasing) { Q_UNUSED(antialiasing) }
    virtual void setAligned(bool aligned) = 0;

    virtual void update() = 0;

    virtual void accept(QSGNodeVisitorEx *visitor) { if (visitor->visit(this)) visitor->visitChildren(this); visitor->endVisit(this); }
};


class Q_QUICK_PRIVATE_EXPORT QSGImageNode : public QSGVisitableNode
{
public:
    virtual void setTargetRect(const QRectF &rect) = 0;
    virtual void setInnerTargetRect(const QRectF &rect) = 0;
    virtual void setInnerSourceRect(const QRectF &rect) = 0;
    // The sub-source rect's width and height specify the number of times the inner source rect
    // is repeated inside the inner target rect. The x and y specify which (normalized) location
    // in the inner source rect maps to the upper-left corner of the inner target rect.
    virtual void setSubSourceRect(const QRectF &rect) = 0;
    virtual void setTexture(QSGTexture *texture) = 0;
    virtual void setAntialiasing(bool antialiasing) { Q_UNUSED(antialiasing) }
    virtual void setMirror(bool mirror) = 0;
    virtual void setMipmapFiltering(QSGTexture::Filtering filtering) = 0;
    virtual void setFiltering(QSGTexture::Filtering filtering) = 0;
    virtual void setHorizontalWrapMode(QSGTexture::WrapMode wrapMode) = 0;
    virtual void setVerticalWrapMode(QSGTexture::WrapMode wrapMode) = 0;

    virtual void update() = 0;

    virtual void accept(QSGNodeVisitorEx *visitor) { if (visitor->visit(this)) visitor->visitChildren(this); visitor->endVisit(this); }
};

class Q_QUICK_PRIVATE_EXPORT QSGPainterNode : public QSGVisitableNode
{
public:

    virtual void setPreferredRenderTarget(QQuickPaintedItem::RenderTarget target) = 0;
    virtual void setSize(const QSize &size) = 0;
    virtual void setDirty(const QRect &dirtyRect = QRect()) = 0;
    virtual void setOpaquePainting(bool opaque) = 0;
    virtual void setLinearFiltering(bool linearFiltering) = 0;
    virtual void setMipmapping(bool mipmapping) = 0;
    virtual void setSmoothPainting(bool s) = 0;
    virtual void setFillColor(const QColor &c) = 0;
    virtual void setContentsScale(qreal s) = 0;
    virtual void setFastFBOResizing(bool dynamic) = 0;

    virtual QImage toImage() const = 0;
    virtual void update() = 0;
    virtual QSGTexture *texture() const = 0;

    virtual void accept(QSGNodeVisitorEx *visitor) { if (visitor->visit(this)) visitor->visitChildren(this); visitor->endVisit(this); }
};

class Q_QUICK_PRIVATE_EXPORT QSGNinePatchNode : public QSGVisitableNode
{
public:
    virtual void setTexture(QSGTexture *texture) = 0;
    virtual void setBounds(const QRectF &bounds) = 0;
    virtual void setDevicePixelRatio(qreal ratio) = 0;
    virtual void setPadding(qreal left, qreal top, qreal right, qreal bottom) = 0;

    virtual void update() = 0;

    virtual void accept(QSGNodeVisitorEx *visitor) { if (visitor->visit(this)) visitor->visitChildren(this); visitor->endVisit(this); }
};

class Q_QUICK_EXPORT QSGLayer : public QSGDynamicTexture
{
    Q_OBJECT
public:
    virtual void setItem(QSGNode *item) = 0;
    virtual void setRect(const QRectF &rect) = 0;
    virtual void setSize(const QSize &size) = 0;
    virtual void scheduleUpdate() = 0;
    virtual QImage toImage() const = 0;
    virtual void setLive(bool live) = 0;
    virtual void setRecursive(bool recursive) = 0;
    virtual void setFormat(GLenum format) = 0;
    virtual void setHasMipmaps(bool mipmap) = 0;
    virtual void setDevicePixelRatio(qreal ratio) = 0;
    Q_SLOT virtual void markDirtyTexture() = 0;
    Q_SLOT virtual void invalidated() = 0;

Q_SIGNALS:
    void updateRequested();
    void scheduledUpdateCompleted();
};

class Q_QUICK_PRIVATE_EXPORT QSGGlyphNode : public QSGVisitableNode
{
public:
    enum AntialiasingMode
    {
        GrayAntialiasing,
        LowQualitySubPixelAntialiasing,
        HighQualitySubPixelAntialiasing
    };

    QSGGlyphNode() : m_ownerElement(0) {}

    virtual void setGlyphs(const QPointF &position, const QGlyphRun &glyphs) = 0;
    virtual void setColor(const QColor &color) = 0;
    virtual void setStyle(QQuickText::TextStyle style) = 0;
    virtual void setStyleColor(const QColor &color) = 0;
    virtual QPointF baseLine() const = 0;

    virtual QRectF boundingRect() const { return m_bounding_rect; }
    virtual void setBoundingRect(const QRectF &bounds) { m_bounding_rect = bounds; }

    virtual void setPreferredAntialiasingMode(AntialiasingMode) = 0;

    virtual void update() = 0;

    void setOwnerElement(QQuickItem *ownerElement) { m_ownerElement = ownerElement; }
    QQuickItem *ownerElement() const { return m_ownerElement; }

    virtual void accept(QSGNodeVisitorEx *visitor) { if (visitor->visit(this)) visitor->visitChildren(this); visitor->endVisit(this); }
protected:
    QRectF m_bounding_rect;
    QQuickItem *m_ownerElement;
};

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldGlyphConsumer
{
public:
    virtual ~QSGDistanceFieldGlyphConsumer() {}

    virtual void invalidateGlyphs(const QVector<quint32> &glyphs) = 0;
};

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldGlyphCache
{
public:
    QSGDistanceFieldGlyphCache(QSGDistanceFieldGlyphCacheManager *man, QOpenGLContext *c, const QRawFont &font);
    virtual ~QSGDistanceFieldGlyphCache();

    struct Metrics {
        qreal width;
        qreal height;
        qreal baselineX;
        qreal baselineY;

        bool isNull() const { return width == 0 || height == 0; }
    };

    struct TexCoord {
        qreal x;
        qreal y;
        qreal width;
        qreal height;
        qreal xMargin;
        qreal yMargin;

        TexCoord() : x(0), y(0), width(-1), height(-1), xMargin(0), yMargin(0) { }

        bool isNull() const { return width <= 0 || height <= 0; }
        bool isValid() const { return width >= 0 && height >= 0; }
    };

    struct Texture {
        GLuint textureId;
        QSize size;

        Texture() : textureId(0), size(QSize()) { }
        bool operator == (const Texture &other) const { return textureId == other.textureId; }
    };

    const QSGDistanceFieldGlyphCacheManager *manager() const { return m_manager; }

    const QRawFont &referenceFont() const { return m_referenceFont; }

    qreal fontScale(qreal pixelSize) const
    {
        return pixelSize / QT_DISTANCEFIELD_BASEFONTSIZE(m_doubleGlyphResolution);
    }
    int distanceFieldRadius() const
    {
        return QT_DISTANCEFIELD_RADIUS(m_doubleGlyphResolution) / QT_DISTANCEFIELD_SCALE(m_doubleGlyphResolution);
    }
    int glyphCount() const { return m_glyphCount; }
    bool doubleGlyphResolution() const { return m_doubleGlyphResolution; }

    Metrics glyphMetrics(glyph_t glyph, qreal pixelSize);
    inline TexCoord glyphTexCoord(glyph_t glyph);
    inline const Texture *glyphTexture(glyph_t glyph);

    void populate(const QVector<glyph_t> &glyphs);
    void release(const QVector<glyph_t> &glyphs);

    void update();

    void registerGlyphNode(QSGDistanceFieldGlyphConsumer *node) { m_registeredNodes.append(node); }
    void unregisterGlyphNode(QSGDistanceFieldGlyphConsumer *node) { m_registeredNodes.removeOne(node); }

    virtual void registerOwnerElement(QQuickItem *ownerElement);
    virtual void unregisterOwnerElement(QQuickItem *ownerElement);
    virtual void processPendingGlyphs();

protected:
    struct GlyphPosition {
        glyph_t glyph;
        QPointF position;
    };

    struct GlyphData {
        Texture *texture;
        TexCoord texCoord;
        QRectF boundingRect;
        QPainterPath path;
        quint32 ref;

        GlyphData() : texture(0), ref(0) { }
    };

    virtual void requestGlyphs(const QSet<glyph_t> &glyphs) = 0;
    virtual void storeGlyphs(const QList<QDistanceField> &glyphs) = 0;
    virtual void referenceGlyphs(const QSet<glyph_t> &glyphs) = 0;
    virtual void releaseGlyphs(const QSet<glyph_t> &glyphs) = 0;

    void setGlyphsPosition(const QList<GlyphPosition> &glyphs);
    void setGlyphsTexture(const QVector<glyph_t> &glyphs, const Texture &tex);
    void markGlyphsToRender(const QVector<glyph_t> &glyphs);
    inline void removeGlyph(glyph_t glyph);

    void updateTexture(GLuint oldTex, GLuint newTex, const QSize &newTexSize);

    inline bool containsGlyph(glyph_t glyph);
    GLuint textureIdForGlyph(glyph_t glyph) const;

    GlyphData &glyphData(glyph_t glyph);

#if defined(QSG_DISTANCEFIELD_CACHE_DEBUG)
    void saveTexture(GLuint textureId, int width, int height) const;
#endif

    inline bool isCoreProfile() const { return m_coreProfile; }

private:
    QSGDistanceFieldGlyphCacheManager *m_manager;

    QRawFont m_referenceFont;
    int m_glyphCount;

    bool m_doubleGlyphResolution;
    bool m_coreProfile;

    QList<Texture> m_textures;
    QHash<glyph_t, GlyphData> m_glyphsData;
    QDataBuffer<glyph_t> m_pendingGlyphs;
    QSet<glyph_t> m_populatingGlyphs;
    QLinkedList<QSGDistanceFieldGlyphConsumer*> m_registeredNodes;

    static Texture s_emptyTexture;
};

inline QSGDistanceFieldGlyphCache::TexCoord QSGDistanceFieldGlyphCache::glyphTexCoord(glyph_t glyph)
{
    return glyphData(glyph).texCoord;
}

inline const QSGDistanceFieldGlyphCache::Texture *QSGDistanceFieldGlyphCache::glyphTexture(glyph_t glyph)
{
    return glyphData(glyph).texture;
}

inline void QSGDistanceFieldGlyphCache::removeGlyph(glyph_t glyph)
{
    GlyphData &gd = glyphData(glyph);
    gd.texCoord = TexCoord();
    gd.texture = &s_emptyTexture;
}

inline bool QSGDistanceFieldGlyphCache::containsGlyph(glyph_t glyph)
{
    return glyphData(glyph).texCoord.isValid();
}


QT_END_NAMESPACE

#endif
