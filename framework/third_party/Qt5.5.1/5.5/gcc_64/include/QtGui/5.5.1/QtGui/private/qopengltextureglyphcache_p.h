/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QOPENGLTEXTUREGLYPHCACHE_P_H
#define QOPENGLTEXTUREGLYPHCACHE_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of the QLibrary class.  This header file may change from
// version to version without notice, or even be removed.
//
// We mean it.
//

#include <private/qtextureglyphcache_p.h>
#include <private/qopenglcontext_p.h>
#include <qopenglshaderprogram.h>
#include <qopenglfunctions.h>
#include <qopenglbuffer.h>
#include <qopenglvertexarrayobject.h>

// #define QT_GL_TEXTURE_GLYPH_CACHE_DEBUG

QT_BEGIN_NAMESPACE

class QOpenGL2PaintEngineExPrivate;

class QOpenGLGlyphTexture : public QOpenGLSharedResource
{
public:
    explicit QOpenGLGlyphTexture(QOpenGLContext *ctx)
        : QOpenGLSharedResource(ctx->shareGroup())
        , m_width(0)
        , m_height(0)
    {
        if (!ctx->d_func()->workaround_brokenFBOReadBack)
            QOpenGLFunctions(ctx).glGenFramebuffers(1, &m_fbo);

#ifdef QT_GL_TEXTURE_GLYPH_CACHE_DEBUG
        qDebug(" -> QOpenGLGlyphTexture() %p for context %p.", this, ctx);
#endif
    }

    void freeResource(QOpenGLContext *context) Q_DECL_OVERRIDE
    {
        QOpenGLContext *ctx = context;
#ifdef QT_GL_TEXTURE_GLYPH_CACHE_DEBUG
        qDebug("~QOpenGLGlyphTexture() %p for context %p.", this, ctx);
#endif
        if (!ctx->d_func()->workaround_brokenFBOReadBack)
            ctx->functions()->glDeleteFramebuffers(1, &m_fbo);
        if (m_width || m_height)
            ctx->functions()->glDeleteTextures(1, &m_texture);
    }

    void invalidateResource() Q_DECL_OVERRIDE
    {
        m_texture = 0;
        m_fbo = 0;
        m_width = 0;
        m_height = 0;
    }

    GLuint m_texture;
    GLuint m_fbo;
    int m_width;
    int m_height;
};

class Q_GUI_EXPORT QOpenGLTextureGlyphCache : public QImageTextureGlyphCache
{
public:
    QOpenGLTextureGlyphCache(QFontEngine::GlyphFormat glyphFormat, const QTransform &matrix);
    ~QOpenGLTextureGlyphCache();

    virtual void createTextureData(int width, int height) Q_DECL_OVERRIDE;
    virtual void resizeTextureData(int width, int height) Q_DECL_OVERRIDE;
    virtual void fillTexture(const Coord &c, glyph_t glyph, QFixed subPixelPosition) Q_DECL_OVERRIDE;
    virtual int glyphPadding() const Q_DECL_OVERRIDE;
    virtual int maxTextureWidth() const Q_DECL_OVERRIDE;
    virtual int maxTextureHeight() const Q_DECL_OVERRIDE;

    inline GLuint texture() const {
        QOpenGLTextureGlyphCache *that = const_cast<QOpenGLTextureGlyphCache *>(this);
        QOpenGLGlyphTexture *glyphTexture = that->m_textureResource;
        return glyphTexture ? glyphTexture->m_texture : 0;
    }

    inline int width() const {
        QOpenGLTextureGlyphCache *that = const_cast<QOpenGLTextureGlyphCache *>(this);
        QOpenGLGlyphTexture *glyphTexture = that->m_textureResource;
        return glyphTexture ? glyphTexture->m_width : 0;
    }
    inline int height() const {
        QOpenGLTextureGlyphCache *that = const_cast<QOpenGLTextureGlyphCache *>(this);
        QOpenGLGlyphTexture *glyphTexture = that->m_textureResource;
        return glyphTexture ? glyphTexture->m_height : 0;
    }

    inline void setPaintEnginePrivate(QOpenGL2PaintEngineExPrivate *p) { pex = p; }

    inline const QOpenGLContextGroup *contextGroup() const { return m_textureResource ? m_textureResource->group() : 0; }

    inline int serialNumber() const { return m_serialNumber; }

    enum FilterMode {
        Nearest,
        Linear
    };
    FilterMode filterMode() const { return m_filterMode; }
    void setFilterMode(FilterMode m) { m_filterMode = m; }

    void clear();

private:
    void setupVertexAttribs();

    QOpenGLGlyphTexture *m_textureResource;

    QOpenGL2PaintEngineExPrivate *pex;
    QOpenGLShaderProgram *m_blitProgram;
    FilterMode m_filterMode;

    GLfloat m_vertexCoordinateArray[8];
    GLfloat m_textureCoordinateArray[8];

    int m_serialNumber;

    QOpenGLBuffer m_buffer;
    QOpenGLVertexArrayObject m_vao;
};

QT_END_NAMESPACE

#endif // QOPENGLTEXTUREGLYPHCACHE_P_H

