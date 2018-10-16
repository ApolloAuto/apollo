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

#ifndef QQUICKIMAGE_P_H
#define QQUICKIMAGE_P_H

#include "qquickimagebase_p.h"
#include <QtQuick/qsgtextureprovider.h>

QT_BEGIN_NAMESPACE

class QQuickImagePrivate;
class Q_AUTOTEST_EXPORT QQuickImage : public QQuickImageBase
{
    Q_OBJECT
    Q_ENUMS(FillMode)
    Q_ENUMS(HAlignment)
    Q_ENUMS(VAlignment)

    Q_PROPERTY(FillMode fillMode READ fillMode WRITE setFillMode NOTIFY fillModeChanged)
    Q_PROPERTY(qreal paintedWidth READ paintedWidth NOTIFY paintedGeometryChanged)
    Q_PROPERTY(qreal paintedHeight READ paintedHeight NOTIFY paintedGeometryChanged)
    Q_PROPERTY(HAlignment horizontalAlignment READ horizontalAlignment WRITE setHorizontalAlignment NOTIFY horizontalAlignmentChanged)
    Q_PROPERTY(VAlignment verticalAlignment READ verticalAlignment WRITE setVerticalAlignment NOTIFY verticalAlignmentChanged)
    Q_PROPERTY(bool mipmap READ mipmap WRITE setMipmap NOTIFY mipmapChanged REVISION 1)
    Q_PROPERTY(bool autoTransform READ autoTransform WRITE setAutoTransform NOTIFY autoTransformChanged REVISION 2)

public:
    QQuickImage(QQuickItem *parent=0);
    ~QQuickImage();

    enum HAlignment { AlignLeft = Qt::AlignLeft,
                       AlignRight = Qt::AlignRight,
                       AlignHCenter = Qt::AlignHCenter };
    enum VAlignment { AlignTop = Qt::AlignTop,
                       AlignBottom = Qt::AlignBottom,
                       AlignVCenter = Qt::AlignVCenter };

    enum FillMode { Stretch, PreserveAspectFit, PreserveAspectCrop, Tile, TileVertically, TileHorizontally, Pad };

    FillMode fillMode() const;
    void setFillMode(FillMode);

    qreal paintedWidth() const;
    qreal paintedHeight() const;

    QRectF boundingRect() const Q_DECL_OVERRIDE;

    HAlignment horizontalAlignment() const;
    void setHorizontalAlignment(HAlignment align);

    VAlignment verticalAlignment() const;
    void setVerticalAlignment(VAlignment align);

    bool isTextureProvider() const Q_DECL_OVERRIDE { return true; }
    QSGTextureProvider *textureProvider() const Q_DECL_OVERRIDE;

    bool mipmap() const;
    void setMipmap(bool use);

    virtual void emitAutoTransformBaseChanged() Q_DECL_OVERRIDE { emit autoTransformChanged(); }

Q_SIGNALS:
    void fillModeChanged();
    void paintedGeometryChanged();
    void horizontalAlignmentChanged(HAlignment alignment);
    void verticalAlignmentChanged(VAlignment alignment);
    Q_REVISION(1) void mipmapChanged(bool);
    Q_REVISION(2) void autoTransformChanged();

private Q_SLOTS:
    void invalidateSceneGraph();

protected:
    QQuickImage(QQuickImagePrivate &dd, QQuickItem *parent);
    void pixmapChange() Q_DECL_OVERRIDE;
    void updatePaintedGeometry();
    void releaseResources() Q_DECL_OVERRIDE;

    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;

private:
    Q_DISABLE_COPY(QQuickImage)
    Q_DECLARE_PRIVATE(QQuickImage)
};

QT_END_NAMESPACE
QML_DECLARE_TYPE(QQuickImage)
#endif // QQUICKIMAGE_P_H
