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

#ifndef QSGTEXTURE_H
#define QSGTEXTURE_H

#include <QtQuick/qtquickglobal.h>
#include <QtCore/QObject>
#include <QtGui/QImage>

QT_BEGIN_NAMESPACE

class QSGTexturePrivate;
class Q_QUICK_EXPORT QSGTexture : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QSGTexture)

public:
    QSGTexture();
    ~QSGTexture();

    enum WrapMode {
        Repeat,
        ClampToEdge
    };

    enum Filtering {
        None,
        Nearest,
        Linear
    };

    virtual int textureId() const = 0;
    virtual QSize textureSize() const = 0;
    virtual bool hasAlphaChannel() const = 0;
    virtual bool hasMipmaps() const = 0;

    virtual QRectF normalizedTextureSubRect() const;

    virtual bool isAtlasTexture() const;

    virtual QSGTexture *removedFromAtlas() const;

    virtual void bind() = 0;
    void updateBindOptions(bool force = false);

    void setMipmapFiltering(Filtering filter);
    QSGTexture::Filtering mipmapFiltering() const;

    void setFiltering(Filtering filter);
    QSGTexture::Filtering filtering() const;

    void setHorizontalWrapMode(WrapMode hwrap);
    QSGTexture::WrapMode horizontalWrapMode() const;

    void setVerticalWrapMode(WrapMode vwrap);
    QSGTexture::WrapMode verticalWrapMode() const;

    inline QRectF convertToNormalizedSourceRect(const QRectF &rect) const;

protected:
    QSGTexture(QSGTexturePrivate &dd);
};

QRectF QSGTexture::convertToNormalizedSourceRect(const QRectF &rect) const
{
    QSize s = textureSize();
    QRectF r = normalizedTextureSubRect();

    qreal sx = r.width() / s.width();
    qreal sy = r.height() / s.height();

    return QRectF(r.x() + rect.x() * sx,
                  r.y() + rect.y() * sy,
                  rect.width() * sx,
                  rect.height() * sy);
}


class Q_QUICK_EXPORT QSGDynamicTexture : public QSGTexture
{
    Q_OBJECT
public:
    virtual bool updateTexture() = 0;
};

QT_END_NAMESPACE

#endif
