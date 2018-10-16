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


#ifndef QSGDEFAULTRECTANGLENODE_P_H
#define QSGDEFAULTRECTANGLENODE_P_H

#include <private/qsgadaptationlayer_p.h>

#include <QtQuick/qsgvertexcolormaterial.h>

QT_BEGIN_NAMESPACE

class QSGContext;

class Q_QUICK_PRIVATE_EXPORT QSGSmoothColorMaterial : public QSGMaterial
{
public:
    QSGSmoothColorMaterial();

    int compare(const QSGMaterial *other) const;

protected:
    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
};

class Q_QUICK_PRIVATE_EXPORT QSGDefaultRectangleNode : public QSGRectangleNode
{
public:
    QSGDefaultRectangleNode();

    virtual void setRect(const QRectF &rect);
    virtual void setColor(const QColor &color);
    virtual void setPenColor(const QColor &color);
    virtual void setPenWidth(qreal width);
    virtual void setGradientStops(const QGradientStops &stops);
    virtual void setRadius(qreal radius);
    virtual void setAntialiasing(bool antialiasing);
    virtual void setAligned(bool aligned);
    virtual void update();

private:
    void updateGeometry();
    void updateGradientTexture();

    QSGVertexColorMaterial m_material;
    QSGSmoothColorMaterial m_smoothMaterial;

    QRectF m_rect;
    QGradientStops m_gradient_stops;
    QColor m_color;
    QColor m_border_color;
    qreal m_radius;
    qreal m_pen_width;

    uint m_aligned : 1;
    uint m_antialiasing : 1;
    uint m_gradient_is_opaque : 1;
    uint m_dirty_geometry : 1;

    QSGGeometry m_geometry;
};

QT_END_NAMESPACE

#endif
