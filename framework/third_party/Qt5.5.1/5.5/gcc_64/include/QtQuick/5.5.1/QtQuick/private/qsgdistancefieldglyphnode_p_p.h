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

#ifndef QSGDISTANCEFIELDGLYPHNODE_P_P_H
#define QSGDISTANCEFIELDGLYPHNODE_P_P_H

#include <QtQuick/qsgmaterial.h>
#include "qsgdistancefieldglyphnode_p.h"
#include "qsgadaptationlayer_p.h"

QT_BEGIN_NAMESPACE

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldTextMaterial: public QSGMaterial
{
public:
    QSGDistanceFieldTextMaterial();
    ~QSGDistanceFieldTextMaterial();

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    virtual int compare(const QSGMaterial *other) const;

    virtual void setColor(const QColor &color);
    const QVector4D &color() const { return m_color; }

    void setGlyphCache(QSGDistanceFieldGlyphCache *a) { m_glyph_cache = a; }
    QSGDistanceFieldGlyphCache *glyphCache() const { return m_glyph_cache; }

    void setTexture(const QSGDistanceFieldGlyphCache::Texture * tex) { m_texture = tex; }
    const QSGDistanceFieldGlyphCache::Texture * texture() const { return m_texture; }

    void setFontScale(qreal fontScale) { m_fontScale = fontScale; }
    qreal fontScale() const { return m_fontScale; }

    QSize textureSize() const { return m_size; }

    bool updateTextureSize();

protected:
    QSize m_size;
    QVector4D m_color;
    QSGDistanceFieldGlyphCache *m_glyph_cache;
    const QSGDistanceFieldGlyphCache::Texture *m_texture;
    qreal m_fontScale;
};

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldStyledTextMaterial : public QSGDistanceFieldTextMaterial
{
public:
    QSGDistanceFieldStyledTextMaterial();
    ~QSGDistanceFieldStyledTextMaterial();

    virtual QSGMaterialType *type() const = 0;
    virtual QSGMaterialShader *createShader() const = 0;
    virtual int compare(const QSGMaterial *other) const;

    void setStyleColor(const QColor &color);
    const QVector4D &styleColor() const { return m_styleColor; }

protected:
    QVector4D m_styleColor;
};

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldOutlineTextMaterial : public QSGDistanceFieldStyledTextMaterial
{
public:
    QSGDistanceFieldOutlineTextMaterial();
    ~QSGDistanceFieldOutlineTextMaterial();

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
};

class Q_QUICK_PRIVATE_EXPORT QSGDistanceFieldShiftedStyleTextMaterial : public QSGDistanceFieldStyledTextMaterial
{
public:
    QSGDistanceFieldShiftedStyleTextMaterial();
    ~QSGDistanceFieldShiftedStyleTextMaterial();

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    virtual int compare(const QSGMaterial *other) const;

    void setShift(const QPointF &shift) { m_shift = shift; }
    const QPointF &shift() const { return m_shift; }

protected:
    QPointF m_shift;
};

class Q_QUICK_PRIVATE_EXPORT QSGHiQSubPixelDistanceFieldTextMaterial : public QSGDistanceFieldTextMaterial
{
public:
    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    void setColor(const QColor &color) { m_color = QVector4D(color.redF(), color.greenF(), color.blueF(), color.alphaF()); }
};

class Q_QUICK_PRIVATE_EXPORT QSGLoQSubPixelDistanceFieldTextMaterial : public QSGDistanceFieldTextMaterial
{
public:
    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    void setColor(const QColor &color) { m_color = QVector4D(color.redF(), color.greenF(), color.blueF(), color.alphaF()); }
};

QT_END_NAMESPACE

#endif
