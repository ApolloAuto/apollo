/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QPIXMAP_RASTER_P_H
#define QPIXMAP_RASTER_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <qpa/qplatformpixmap.h>


QT_BEGIN_NAMESPACE

class Q_GUI_EXPORT QRasterPlatformPixmap : public QPlatformPixmap
{
public:
    QRasterPlatformPixmap(PixelType type);
    ~QRasterPlatformPixmap();

    QPlatformPixmap *createCompatiblePlatformPixmap() const Q_DECL_OVERRIDE;

    void resize(int width, int height) Q_DECL_OVERRIDE;
    bool fromData(const uchar *buffer, uint len, const char *format, Qt::ImageConversionFlags flags) Q_DECL_OVERRIDE;
    void fromImage(const QImage &image, Qt::ImageConversionFlags flags) Q_DECL_OVERRIDE;
    void fromImageInPlace(QImage &image, Qt::ImageConversionFlags flags) Q_DECL_OVERRIDE;
    void fromImageReader(QImageReader *imageReader, Qt::ImageConversionFlags flags) Q_DECL_OVERRIDE;

    void copy(const QPlatformPixmap *data, const QRect &rect) Q_DECL_OVERRIDE;
    bool scroll(int dx, int dy, const QRect &rect) Q_DECL_OVERRIDE;
    void fill(const QColor &color) Q_DECL_OVERRIDE;
    bool hasAlphaChannel() const Q_DECL_OVERRIDE;
    QImage toImage() const Q_DECL_OVERRIDE;
    QImage toImage(const QRect &rect) const Q_DECL_OVERRIDE;
    QPaintEngine* paintEngine() const Q_DECL_OVERRIDE;
    QImage* buffer() Q_DECL_OVERRIDE;
    qreal devicePixelRatio() const Q_DECL_OVERRIDE;
    void setDevicePixelRatio(qreal scaleFactor) Q_DECL_OVERRIDE;


protected:
    int metric(QPaintDevice::PaintDeviceMetric metric) const Q_DECL_OVERRIDE;
    void createPixmapForImage(QImage &sourceImage, Qt::ImageConversionFlags flags, bool inPlace);
    void setImage(const QImage &image);
    QImage image;

private:
    friend class QPixmap;
    friend class QBitmap;
    friend class QPixmapCacheEntry;
    friend class QRasterPaintEngine;
};

QT_END_NAMESPACE

#endif // QPIXMAP_RASTER_P_H


