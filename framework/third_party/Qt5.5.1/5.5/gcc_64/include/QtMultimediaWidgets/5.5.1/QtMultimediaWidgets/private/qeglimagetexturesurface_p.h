/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QEGLIMAGETEXTURESURFACE_P_H
#define QEGLIMAGETEXTURESURFACE_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists purely as an
// implementation detail. This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <qtmultimediawidgetdefs.h>
#include <QtCore/qsize.h>
#include <QtGui/qimage.h>
#include <QtGui/qmatrix4x4.h>
#include <QtGui/qpaintengine.h>

#include <QtOpenGL/qglshaderprogram.h>

#include <qabstractvideosurface.h>
#include <qvideosurfaceformat.h>
#include <qvideoframe.h>

QT_BEGIN_NAMESPACE

class QGLContext;
class QGLShaderProgram;
class QPainterVideoSurface;

class QEglImageTextureSurface : public QAbstractVideoSurface
{
    Q_OBJECT
public:
    explicit QEglImageTextureSurface(QObject *parent = 0);
    ~QEglImageTextureSurface();

    QList<QVideoFrame::PixelFormat> supportedPixelFormats(
            QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const;

    bool start(const QVideoSurfaceFormat &format);
    void stop();

    bool present(const QVideoFrame &frame);

    int brightness() const;
    void setBrightness(int brightness);

    int contrast() const;
    void setContrast(int contrast);

    int hue() const;
    void setHue(int hue);

    int saturation() const;
    void setSaturation(int saturation);

    bool isReady() const;
    void setReady(bool ready);

    void paint(QPainter *painter, const QRectF &target, const QRectF &source = QRectF(0, 0, 1, 1));

    const QGLContext *glContext() const;
    void setGLContext(QGLContext *context);

    bool isOverlayEnabled() const;
    void setOverlayEnabled(bool enabled);

    QRect displayRect() const;
    void setDisplayRect(const QRect &rect);

public Q_SLOTS:
    void viewportDestroyed();

Q_SIGNALS:
    void frameChanged();

private:
    QGLContext *m_context;
    QGLShaderProgram *m_program;

    QVideoFrame m_frame;

    QVideoFrame::PixelFormat m_pixelFormat;
    QVideoSurfaceFormat::Direction m_scanLineDirection;
    QSize m_frameSize;
    QRect m_sourceRect;
    bool m_ready;

    QRect m_viewport;
    QRect m_displayRect;
    QColor m_colorKey;

    QPainterVideoSurface *m_fallbackSurface;
    bool m_fallbackSurfaceActive;
};

QT_END_NAMESPACE


#endif
