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

#ifndef QSGDEFAULTGLYPHNODE_P_P_H
#define QSGDEFAULTGLYPHNODE_P_P_H

#include <qcolor.h>
#include <QtGui/private/qopengltextureglyphcache_p.h>
#include <QtQuick/qsgmaterial.h>
#include <QtQuick/qsgtexture.h>
#include <QtQuick/qsggeometry.h>
#include <qshareddata.h>
#include <QtQuick/private/qsgtexture_p.h>
#include <qrawfont.h>
#include <qmargins.h>

QT_BEGIN_NAMESPACE

class QFontEngine;
class Geometry;
class QSGTextMaskMaterial: public QSGMaterial
{
public:
    QSGTextMaskMaterial(const QRawFont &font, QFontEngine::GlyphFormat glyphFormat = QFontEngine::Format_None);
    virtual ~QSGTextMaskMaterial();

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;
    virtual int compare(const QSGMaterial *other) const;

    void setColor(const QColor &c) { m_color = QVector4D(c.redF(), c.greenF(), c.blueF(), c.alphaF()); }
    void setColor(const QVector4D &color) { m_color = color; }
    const QVector4D &color() const { return m_color; }

    QSGTexture *texture() const { return m_texture; }

    int cacheTextureWidth() const;
    int cacheTextureHeight() const;

    bool ensureUpToDate();

    QOpenGLTextureGlyphCache *glyphCache() const;
    void populate(const QPointF &position,
                  const QVector<quint32> &glyphIndexes, const QVector<QPointF> &glyphPositions,
                  QSGGeometry *geometry, QRectF *boundingRect, QPointF *baseLine,
                  const QMargins &margins = QMargins(0, 0, 0, 0));

private:
    void init(QFontEngine::GlyphFormat glyphFormat);

    QSGPlainTexture *m_texture;
    QExplicitlySharedDataPointer<QFontEngineGlyphCache> m_glyphCache;
    QRawFont m_font;
    QVector4D m_color;
    QSize m_size;
};

class QSGStyledTextMaterial : public QSGTextMaskMaterial
{
public:
    QSGStyledTextMaterial(const QRawFont &font);
    virtual ~QSGStyledTextMaterial() { }

    void setStyleShift(const QVector2D &shift) { m_styleShift = shift; }
    const QVector2D &styleShift() const { return m_styleShift; }

    void setStyleColor(const QColor &c) { m_styleColor = QVector4D(c.redF(), c.greenF(), c.blueF(), c.alphaF()); }
    void setStyleColor(const QVector4D &color) { m_styleColor = color; }
    const QVector4D &styleColor() const { return m_styleColor; }

    virtual QSGMaterialType *type() const;
    virtual QSGMaterialShader *createShader() const;

    int compare(const QSGMaterial *other) const;

private:
    QVector2D m_styleShift;
    QVector4D m_styleColor;
};

class QSGOutlinedTextMaterial : public QSGStyledTextMaterial
{
public:
    QSGOutlinedTextMaterial(const QRawFont &font);
    ~QSGOutlinedTextMaterial() { }

    QSGMaterialType *type() const;
    QSGMaterialShader *createShader() const;
};

QT_END_NAMESPACE

#endif
