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

#ifndef QSGDEFAULTDISTANCEFIELDGLYPHCACHE_H
#define QSGDEFAULTDISTANCEFIELDGLYPHCACHE_H

#include "qsgadaptationlayer_p.h"
#include <QtGui/qopenglfunctions.h>
#include <qopenglshaderprogram.h>
#include <qopenglbuffer.h>
#include <qopenglvertexarrayobject.h>
#include <QtGui/private/qopenglengineshadersource_p.h>
#include <private/qsgareaallocator_p.h>

QT_BEGIN_NAMESPACE

class QOpenGLSharedResourceGuard;
#if !defined(QT_OPENGL_ES_2)
class QOpenGLFunctions_3_2_Core;
#endif

class Q_QUICK_PRIVATE_EXPORT QSGDefaultDistanceFieldGlyphCache : public QSGDistanceFieldGlyphCache
{
public:
    QSGDefaultDistanceFieldGlyphCache(QSGDistanceFieldGlyphCacheManager *man, QOpenGLContext *c, const QRawFont &font);
    virtual ~QSGDefaultDistanceFieldGlyphCache();

    void requestGlyphs(const QSet<glyph_t> &glyphs);
    void storeGlyphs(const QList<QDistanceField> &glyphs);
    void referenceGlyphs(const QSet<glyph_t> &glyphs);
    void releaseGlyphs(const QSet<glyph_t> &glyphs);

    bool useTextureResizeWorkaround() const;
    bool useTextureUploadWorkaround() const;
    bool createFullSizeTextures() const;
    int maxTextureSize() const;

    void setMaxTextureCount(int max) { m_maxTextureCount = max; }
    int maxTextureCount() const { return m_maxTextureCount; }

private:
    struct TextureInfo {
        GLuint texture;
        QSize size;
        QRect allocatedArea;
        QDistanceField image;
        int padding;

        TextureInfo(const QRect &preallocRect = QRect()) : texture(0), allocatedArea(preallocRect), padding(-1)
        { }
    };

    void createTexture(TextureInfo * texInfo, int width, int height);
    void resizeTexture(TextureInfo * texInfo, int width, int height);

    TextureInfo *textureInfo(int index)
    {
        for (int i = m_textures.count(); i <= index; ++i) {
            if (createFullSizeTextures())
                m_textures.append(QRect(0, 0, maxTextureSize(), maxTextureSize()));
            else
                m_textures.append(TextureInfo());
        }

        return &m_textures[index];
    }

    void createBlitProgram()
    {
        m_blitProgram = new QOpenGLShaderProgram;
        {
            QString source;
            source.append(QLatin1String(qopenglslMainWithTexCoordsVertexShader));
            source.append(QLatin1String(qopenglslUntransformedPositionVertexShader));

            QOpenGLShader *vertexShader = new QOpenGLShader(QOpenGLShader::Vertex, m_blitProgram);
            vertexShader->compileSourceCode(source);

            m_blitProgram->addShader(vertexShader);
        }
        {
            QString source;
            source.append(QLatin1String(qopenglslMainFragmentShader));
            source.append(QLatin1String(qopenglslImageSrcFragmentShader));

            QOpenGLShader *fragmentShader = new QOpenGLShader(QOpenGLShader::Fragment, m_blitProgram);
            fragmentShader->compileSourceCode(source);

            m_blitProgram->addShader(fragmentShader);
        }
        m_blitProgram->bindAttributeLocation("vertexCoordsArray", QT_VERTEX_COORDS_ATTR);
        m_blitProgram->bindAttributeLocation("textureCoordArray", QT_TEXTURE_COORDS_ATTR);
        m_blitProgram->link();
    }

    mutable int m_maxTextureSize;
    int m_maxTextureCount;

    QList<TextureInfo> m_textures;
    QHash<glyph_t, TextureInfo *> m_glyphsTexture;
    QSet<glyph_t> m_unusedGlyphs;

    QSGAreaAllocator *m_areaAllocator;

    QOpenGLShaderProgram *m_blitProgram;
    QOpenGLBuffer m_blitBuffer;
    QOpenGLVertexArrayObject m_vao;

    QOpenGLSharedResourceGuard *m_fboGuard;
    QOpenGLFunctions *m_funcs;
#if !defined(QT_OPENGL_ES_2)
    QOpenGLFunctions_3_2_Core *m_coreFuncs;
#endif
};

QT_END_NAMESPACE

#endif // QSGDEFAULTDISTANCEFIELDGLYPHCACHE_H
