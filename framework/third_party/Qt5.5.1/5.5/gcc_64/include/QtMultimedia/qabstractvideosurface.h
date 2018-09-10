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

#ifndef QABSTRACTVIDEOSURFACE_H
#define QABSTRACTVIDEOSURFACE_H

#include <QtCore/qobject.h>
#include <QtMultimedia/qvideoframe.h>

QT_BEGIN_NAMESPACE

class QRectF;
class QVideoSurfaceFormat;

class QAbstractVideoSurfacePrivate;

class Q_MULTIMEDIA_EXPORT QAbstractVideoSurface : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QSize nativeResolution READ nativeResolution NOTIFY nativeResolutionChanged)
public:
    enum Error
    {
        NoError,
        UnsupportedFormatError,
        IncorrectFormatError,
        StoppedError,
        ResourceError
    };

    explicit QAbstractVideoSurface(QObject *parent = 0);
    ~QAbstractVideoSurface();

    virtual QList<QVideoFrame::PixelFormat> supportedPixelFormats(
            QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const = 0;
    virtual bool isFormatSupported(const QVideoSurfaceFormat &format) const;
    virtual QVideoSurfaceFormat nearestFormat(const QVideoSurfaceFormat &format) const;

    QVideoSurfaceFormat surfaceFormat() const;

    QSize nativeResolution() const;

    virtual bool start(const QVideoSurfaceFormat &format);
    virtual void stop();

    bool isActive() const;

    virtual bool present(const QVideoFrame &frame) = 0;

    Error error() const;

Q_SIGNALS:
    void activeChanged(bool active);
    void surfaceFormatChanged(const QVideoSurfaceFormat &format);
    void supportedFormatsChanged();
    void nativeResolutionChanged(const QSize &);

protected:
    void setError(Error error);
    void setNativeResolution(const QSize &resolution);

private:
    Q_DECLARE_PRIVATE(QAbstractVideoSurface)
    QScopedPointer<QAbstractVideoSurfacePrivate> d_ptr;
};

#ifndef QT_NO_DEBUG_STREAM
Q_MULTIMEDIA_EXPORT QDebug operator<<(QDebug, const QAbstractVideoSurface::Error &);
#endif

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QAbstractVideoSurface*)
Q_DECLARE_METATYPE(QAbstractVideoSurface::Error)

#endif
