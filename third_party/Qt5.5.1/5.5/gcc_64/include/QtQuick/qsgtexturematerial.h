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

#ifndef QSGTEXTUREMATERIAL_H
#define QSGTEXTUREMATERIAL_H

#include <QtQuick/qsgmaterial.h>
#include <QtQuick/qsgtexture.h>

QT_BEGIN_NAMESPACE

class Q_QUICK_EXPORT QSGOpaqueTextureMaterial : public QSGMaterial
{
public:
    QSGOpaqueTextureMaterial();

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    virtual int compare(const QSGMaterial *other) const;

    void setTexture(QSGTexture *texture);
    QSGTexture *texture() const { return m_texture; }

    void setMipmapFiltering(QSGTexture::Filtering filteringType) { m_mipmap_filtering = filteringType; }
    QSGTexture::Filtering mipmapFiltering() const { return QSGTexture::Filtering(m_mipmap_filtering); }

    void setFiltering(QSGTexture::Filtering filteringType) { m_filtering = filteringType; }
    QSGTexture::Filtering filtering() const { return QSGTexture::Filtering(m_filtering); }

    void setHorizontalWrapMode(QSGTexture::WrapMode mode) { m_horizontal_wrap = mode; }
    QSGTexture::WrapMode horizontalWrapMode() const { return QSGTexture::WrapMode(m_horizontal_wrap); }

    void setVerticalWrapMode(QSGTexture::WrapMode mode) { m_vertical_wrap = mode; }
    QSGTexture::WrapMode verticalWrapMode() const { return QSGTexture::WrapMode(m_vertical_wrap); }

protected:
    QSGTexture *m_texture;

    uint m_filtering: 2;
    uint m_mipmap_filtering: 2;
    uint m_horizontal_wrap : 1;
    uint m_vertical_wrap: 1;

    uint m_reserved : 26;
};


class Q_QUICK_EXPORT QSGTextureMaterial : public QSGOpaqueTextureMaterial
{
public:
    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
};

QT_END_NAMESPACE

#endif
