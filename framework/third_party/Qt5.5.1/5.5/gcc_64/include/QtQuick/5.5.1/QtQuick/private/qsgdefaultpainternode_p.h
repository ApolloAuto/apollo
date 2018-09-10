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

#ifndef QSGDEFAULTPAINTERNODE_P_H
#define QSGDEFAULTPAINTERNODE_P_H

#include <private/qsgadaptationlayer_p.h>
#include "qsgtexturematerial.h"
#include "qsgtexture_p.h"

#include <QtQuick/qquickpainteditem.h>

#include <QtGui/qcolor.h>

QT_BEGIN_NAMESPACE

class QOpenGLFramebufferObject;
class QOpenGLPaintDevice;

class Q_QUICK_PRIVATE_EXPORT QSGPainterTexture : public QSGPlainTexture
{
public:
    QSGPainterTexture();

    void setDirtyRect(const QRect &rect) { m_dirty_rect = rect; }

    void bind();

private:
    QRect m_dirty_rect;
};

class Q_QUICK_PRIVATE_EXPORT QSGDefaultPainterNode : public QSGPainterNode
{
public:
    QSGDefaultPainterNode(QQuickPaintedItem *item);
    virtual ~QSGDefaultPainterNode();

    void setPreferredRenderTarget(QQuickPaintedItem::RenderTarget target);

    void setSize(const QSize &size);
    QSize size() const { return m_size; }

    void setDirty(const QRect &dirtyRect = QRect());

    void setOpaquePainting(bool opaque);
    bool opaquePainting() const { return m_opaquePainting; }

    void setLinearFiltering(bool linearFiltering);
    bool linearFiltering() const { return m_linear_filtering; }

    void setMipmapping(bool mipmapping);
    bool mipmapping() const { return m_mipmapping; }

    void setSmoothPainting(bool s);
    bool smoothPainting() const { return m_smoothPainting; }

    void setFillColor(const QColor &c);
    QColor fillColor() const { return m_fillColor; }

    void setContentsScale(qreal s);
    qreal contentsScale() const { return m_contentsScale; }

    void setFastFBOResizing(bool dynamic);
    bool fastFBOResizing() const { return m_fastFBOResizing; }

    QImage toImage() const;
    void update();

    void paint();

    QSGTexture *texture() const { return m_texture; }

private:
    void updateTexture();
    void updateGeometry();
    void updateRenderTarget();
    void updateFBOSize();

    QSGRenderContext *m_context;

    QQuickPaintedItem::RenderTarget m_preferredRenderTarget;
    QQuickPaintedItem::RenderTarget m_actualRenderTarget;

    QQuickPaintedItem *m_item;

    QOpenGLFramebufferObject *m_fbo;
    QOpenGLFramebufferObject *m_multisampledFbo;
    QImage m_image;

    QSGOpaqueTextureMaterial m_material;
    QSGTextureMaterial m_materialO;
    QSGGeometry m_geometry;
    QSGPainterTexture *m_texture;
    QOpenGLPaintDevice *m_gl_device;

    QSize m_size;
    QSize m_fboSize;
    bool m_dirtyContents;
    QRect m_dirtyRect;
    bool m_opaquePainting;
    bool m_linear_filtering;
    bool m_mipmapping;
    bool m_smoothPainting;
    bool m_extensionsChecked;
    bool m_multisamplingSupported;
    bool m_fastFBOResizing;
    QColor m_fillColor;
    qreal m_contentsScale;

    bool m_dirtyGeometry;
    bool m_dirtyRenderTarget;
    bool m_dirtyTexture;
};

QT_END_NAMESPACE

#endif // QSGDEFAULTPAINTERNODE_P_H
