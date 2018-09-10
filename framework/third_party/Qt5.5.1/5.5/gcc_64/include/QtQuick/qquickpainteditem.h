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

#ifndef QQUICKPAINTEDITEM_P_H
#define QQUICKPAINTEDITEM_P_H

#include <QtQuick/qquickitem.h>
#include <QtGui/qcolor.h>

QT_BEGIN_NAMESPACE

class QQuickPaintedItemPrivate;
class Q_QUICK_EXPORT QQuickPaintedItem : public QQuickItem
{
    Q_OBJECT
    Q_ENUMS(RenderTarget)

    Q_PROPERTY(QSize contentsSize READ contentsSize WRITE setContentsSize NOTIFY contentsSizeChanged)
    Q_PROPERTY(QColor fillColor READ fillColor WRITE setFillColor NOTIFY fillColorChanged)
    Q_PROPERTY(qreal contentsScale READ contentsScale WRITE setContentsScale NOTIFY contentsScaleChanged)
    Q_PROPERTY(RenderTarget renderTarget READ renderTarget WRITE setRenderTarget NOTIFY renderTargetChanged)
public:
    QQuickPaintedItem(QQuickItem *parent = 0);
    virtual ~QQuickPaintedItem();

    enum RenderTarget {
        Image,
        FramebufferObject,
        InvertedYFramebufferObject
    };

    enum PerformanceHint {
        FastFBOResizing = 0x1
    };
    Q_DECLARE_FLAGS(PerformanceHints, PerformanceHint)

    void update(const QRect &rect = QRect());

    bool opaquePainting() const;
    void setOpaquePainting(bool opaque);

    bool antialiasing() const;
    void setAntialiasing(bool enable);

    bool mipmap() const;
    void setMipmap(bool enable);

    PerformanceHints performanceHints() const;
    void setPerformanceHint(PerformanceHint hint, bool enabled = true);
    void setPerformanceHints(PerformanceHints hints);

    QRectF contentsBoundingRect() const;

    QSize contentsSize() const;
    void setContentsSize(const QSize &);
    void resetContentsSize();

    qreal contentsScale() const;
    void setContentsScale(qreal);

    QColor fillColor() const;
    void setFillColor(const QColor&);

    RenderTarget renderTarget() const;
    void setRenderTarget(RenderTarget target);

    virtual void paint(QPainter *painter) = 0;

    bool isTextureProvider() const Q_DECL_OVERRIDE;
    QSGTextureProvider *textureProvider() const Q_DECL_OVERRIDE;

Q_SIGNALS:
    void fillColorChanged();
    void contentsSizeChanged();
    void contentsScaleChanged();
    void renderTargetChanged();

protected:
    QQuickPaintedItem(QQuickPaintedItemPrivate &dd, QQuickItem *parent = 0);
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;
    void releaseResources() Q_DECL_OVERRIDE;

private Q_SLOTS:
    void invalidateSceneGraph();

private:
    Q_DISABLE_COPY(QQuickPaintedItem)
    Q_DECLARE_PRIVATE(QQuickPaintedItem)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QQuickPaintedItem::PerformanceHints)

QT_END_NAMESPACE

#endif // QQUICKPAINTEDITEM_P_H
