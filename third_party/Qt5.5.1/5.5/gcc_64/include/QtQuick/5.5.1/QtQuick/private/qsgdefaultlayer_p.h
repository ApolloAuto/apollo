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
#ifndef QSGDEFAULTLAYER_P_H
#define QSGDEFAULTLAYER_P_H

#include <private/qsgadaptationlayer_p.h>
#include <private/qsgcontext_p.h>
#include <qsgsimplerectnode.h>

#define QSG_DEBUG_FBO_OVERLAY

class Q_QUICK_PRIVATE_EXPORT QSGDefaultLayer : public QSGLayer
{
    Q_OBJECT
public:
    QSGDefaultLayer(QSGRenderContext *context);
    ~QSGDefaultLayer();

    bool updateTexture() Q_DECL_OVERRIDE;

    // The item's "paint node", not effect node.
    QSGNode *item() const { return m_item; }
    void setItem(QSGNode *item) Q_DECL_OVERRIDE;

    QRectF rect() const { return m_rect; }
    void setRect(const QRectF &rect) Q_DECL_OVERRIDE;

    QSize size() const { return m_size; }
    void setSize(const QSize &size) Q_DECL_OVERRIDE;

    void setHasMipmaps(bool mipmap) Q_DECL_OVERRIDE;

    void bind() Q_DECL_OVERRIDE;

    bool hasAlphaChannel() const Q_DECL_OVERRIDE;
    bool hasMipmaps() const Q_DECL_OVERRIDE;
    int textureId() const Q_DECL_OVERRIDE;
    QSize textureSize() const Q_DECL_OVERRIDE { return m_size; }

    GLenum format() const { return m_format; }
    void setFormat(GLenum format) Q_DECL_OVERRIDE;

    bool live() const { return bool(m_live); }
    void setLive(bool live) Q_DECL_OVERRIDE;

    bool recursive() const { return bool(m_recursive); }
    void setRecursive(bool recursive) Q_DECL_OVERRIDE;

    void setDevicePixelRatio(qreal ratio) Q_DECL_OVERRIDE { m_device_pixel_ratio = ratio; }

    void scheduleUpdate() Q_DECL_OVERRIDE;

    QImage toImage() const Q_DECL_OVERRIDE;

public Q_SLOTS:
    void markDirtyTexture() Q_DECL_OVERRIDE;
    void invalidated() Q_DECL_OVERRIDE;

private:
    void grab();

    QSGNode *m_item;
    QRectF m_rect;
    QSize m_size;
    qreal m_device_pixel_ratio;
    GLenum m_format;

    QSGRenderer *m_renderer;
    QOpenGLFramebufferObject *m_fbo;
    QOpenGLFramebufferObject *m_secondaryFbo;
    QSharedPointer<QSGDepthStencilBuffer> m_depthStencilBuffer;

    GLuint m_transparentTexture;

#ifdef QSG_DEBUG_FBO_OVERLAY
    QSGSimpleRectNode *m_debugOverlay;
#endif

    QSGRenderContext *m_context;

    uint m_mipmap : 1;
    uint m_live : 1;
    uint m_recursive : 1;
    uint m_dirtyTexture : 1;
    uint m_multisamplingChecked : 1;
    uint m_multisampling : 1;
    uint m_grab : 1;
};

#endif // QSGDEFAULTLAYER_P_H
