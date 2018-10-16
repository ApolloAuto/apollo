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


#ifndef QSGDEFAULTIMAGENODE_P_H
#define QSGDEFAULTIMAGENODE_P_H

#include <private/qsgadaptationlayer_p.h>
#include <QtQuick/qsgtexturematerial.h>

QT_BEGIN_NAMESPACE

class Q_QUICK_PRIVATE_EXPORT QSGSmoothTextureMaterial : public QSGTextureMaterial
{
public:
    QSGSmoothTextureMaterial();

    void setTexture(QSGTexture *texture);

protected:
    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
};

class Q_QUICK_PRIVATE_EXPORT QSGDefaultImageNode : public QSGImageNode
{
public:
    QSGDefaultImageNode();
    virtual void setTargetRect(const QRectF &rect);
    virtual void setInnerTargetRect(const QRectF &rect);
    virtual void setInnerSourceRect(const QRectF &rect);
    virtual void setSubSourceRect(const QRectF &rect);
    virtual void setTexture(QSGTexture *t);
    virtual void setAntialiasing(bool antialiasing);
    virtual void setMirror(bool mirror);
    virtual void update();

    virtual void setMipmapFiltering(QSGTexture::Filtering filtering);
    virtual void setFiltering(QSGTexture::Filtering filtering);
    virtual void setHorizontalWrapMode(QSGTexture::WrapMode wrapMode);
    virtual void setVerticalWrapMode(QSGTexture::WrapMode wrapMode);

    virtual void preprocess();

private:
    void updateGeometry();

    QRectF m_targetRect;
    QRectF m_innerTargetRect;
    QRectF m_innerSourceRect;
    QRectF m_subSourceRect;

    QSGOpaqueTextureMaterial m_material;
    QSGTextureMaterial m_materialO;
    QSGSmoothTextureMaterial m_smoothMaterial;

    uint m_antialiasing : 1;
    uint m_mirror : 1;
    uint m_dirtyGeometry : 1;

    QSGGeometry m_geometry;
};

QT_END_NAMESPACE

#endif
