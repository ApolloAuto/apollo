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

#ifndef QSGATLASTEXTURE_P_H
#define QSGATLASTEXTURE_P_H

#include <QtCore/QSize>

#include <QtGui/qopengl.h>

#include <QtQuick/QSGTexture>
#include <QtQuick/private/qsgtexture_p.h>
#include <QtQuick/private/qsgareaallocator_p.h>

QT_BEGIN_NAMESPACE

namespace QSGAtlasTexture
{

class Texture;
class Atlas;

class Manager : public QObject
{
    Q_OBJECT

public:
    Manager();
    ~Manager();

    QSGTexture *create(const QImage &image);
    void invalidate();

private:
    Atlas *m_atlas;

    QSize m_atlas_size;
    int m_atlas_size_limit;
};

class Atlas : public QObject
{
public:
    Atlas(const QSize &size);
    ~Atlas();

    void invalidate();

    int textureId() const;
    void bind(QSGTexture::Filtering filtering);

    void upload(Texture *texture);
    void uploadBgra(Texture *texture);

    Texture *create(const QImage &image);
    void remove(Texture *t);

    QSize size() const { return m_size; }

    GLuint internalFormat() const { return m_internalFormat; }
    GLuint externalFormat() const { return m_externalFormat; }

private:
    QSGAreaAllocator m_allocator;
    GLuint m_texture_id;
    QSize m_size;
    QList<Texture *> m_pending_uploads;

    GLuint m_internalFormat;
    GLuint m_externalFormat;

    uint m_allocated : 1;
    uint m_use_bgra_fallback: 1;

    uint m_debug_overlay : 1;
};

class Texture : public QSGTexture
{
    Q_OBJECT
public:
    Texture(Atlas *atlas, const QRect &textureRect, const QImage &image);
    ~Texture();

    int textureId() const { return m_atlas->textureId(); }
    QSize textureSize() const { return atlasSubRectWithoutPadding().size(); }
    bool hasAlphaChannel() const { return m_has_alpha; }
    bool hasMipmaps() const { return false; }
    bool isAtlasTexture() const { return true; }

    QRectF normalizedTextureSubRect() const { return m_texture_coords_rect; }

    QRect atlasSubRect() const { return m_allocated_rect; }
    QRect atlasSubRectWithoutPadding() const { return m_allocated_rect.adjusted(1, 1, -1, -1); }

    bool isTexture() const { return true; }

    QSGTexture *removedFromAtlas() const;

    void releaseImage() { m_image = QImage(); }
    const QImage &image() const { return m_image; }

    void bind();

private:
    QRect m_allocated_rect;
    QRectF m_texture_coords_rect;

    QImage m_image;

    Atlas *m_atlas;

    mutable QSGPlainTexture *m_nonatlas_texture;

    uint m_has_alpha : 1;
};

}

QT_END_NAMESPACE

#endif
