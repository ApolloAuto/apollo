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

#ifndef QSGSIMPLETEXTURENODE_H
#define QSGSIMPLETEXTURENODE_H

#include <QtQuick/qsgnode.h>
#include <QtQuick/qsggeometry.h>
#include <QtQuick/qsgtexturematerial.h>

QT_BEGIN_NAMESPACE

class QSGSimpleTextureNodePrivate;

class Q_QUICK_EXPORT QSGSimpleTextureNode : public QSGGeometryNode
{
public:
    QSGSimpleTextureNode();
    ~QSGSimpleTextureNode();

    void setRect(const QRectF &rect);
    inline void setRect(qreal x, qreal y, qreal w, qreal h) { setRect(QRectF(x, y, w, h)); }
    QRectF rect() const;

    void setSourceRect(const QRectF &r);
    inline void setSourceRect(qreal x, qreal y, qreal w, qreal h) { setSourceRect(QRectF(x, y, w, h)); }
    QRectF sourceRect() const;

    void setTexture(QSGTexture *texture);
    QSGTexture *texture() const;

    void setFiltering(QSGTexture::Filtering filtering);
    QSGTexture::Filtering filtering() const;

    enum TextureCoordinatesTransformFlag {
        NoTransform        = 0x00,
        MirrorHorizontally = 0x01,
        MirrorVertically   = 0x02
    };
    Q_DECLARE_FLAGS(TextureCoordinatesTransformMode, TextureCoordinatesTransformFlag)

    void setTextureCoordinatesTransform(TextureCoordinatesTransformMode mode);
    TextureCoordinatesTransformMode textureCoordinatesTransform() const;

    void setOwnsTexture(bool owns);
    bool ownsTexture() const;

private:
    QSGGeometry m_geometry;
    QSGOpaqueTextureMaterial m_opaque_material;
    QSGTextureMaterial m_material;

    QRectF m_rect;

    Q_DECLARE_PRIVATE(QSGSimpleTextureNode)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QSGSimpleTextureNode::TextureCoordinatesTransformMode)

QT_END_NAMESPACE

#endif // QSGSIMPLETEXTURENODE_H
