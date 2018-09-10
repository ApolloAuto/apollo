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

#ifndef QABSTRACTVIDEOBUFFER_H
#define QABSTRACTVIDEOBUFFER_H

#include <QtMultimedia/qtmultimediadefs.h>
#include <QtMultimedia/qmultimedia.h>


#include <QtCore/qmetatype.h>

QT_BEGIN_NAMESPACE


class QVariant;

class QAbstractVideoBufferPrivate;

class Q_MULTIMEDIA_EXPORT QAbstractVideoBuffer
{
public:
    enum HandleType
    {
        NoHandle,
        GLTextureHandle,
        XvShmImageHandle,
        CoreImageHandle,
        QPixmapHandle,
        EGLImageHandle,
        UserHandle = 1000
    };

    enum MapMode
    {
        NotMapped = 0x00,
        ReadOnly  = 0x01,
        WriteOnly = 0x02,
        ReadWrite = ReadOnly | WriteOnly
    };

    QAbstractVideoBuffer(HandleType type);
    virtual ~QAbstractVideoBuffer();
    virtual void release();

    HandleType handleType() const;

    virtual MapMode mapMode() const = 0;

    virtual uchar *map(MapMode mode, int *numBytes, int *bytesPerLine) = 0;
    int mapPlanes(MapMode mode, int *numBytes, int bytesPerLine[4], uchar *data[4]);
    virtual void unmap() = 0;

    virtual QVariant handle() const;

protected:
    QAbstractVideoBuffer(QAbstractVideoBufferPrivate &dd, HandleType type);

    QAbstractVideoBufferPrivate *d_ptr;  // For expansion, not used currently
    HandleType m_type;

private:
    Q_DECLARE_PRIVATE(QAbstractVideoBuffer)
    Q_DISABLE_COPY(QAbstractVideoBuffer)
};

class QAbstractPlanarVideoBufferPrivate;
class Q_MULTIMEDIA_EXPORT QAbstractPlanarVideoBuffer : public QAbstractVideoBuffer
{
public:
    QAbstractPlanarVideoBuffer(HandleType type);
    virtual ~QAbstractPlanarVideoBuffer();

    uchar *map(MapMode mode, int *numBytes, int *bytesPerLine);
    virtual int map(MapMode mode, int *numBytes, int bytesPerLine[4], uchar *data[4]) = 0;

protected:
    QAbstractPlanarVideoBuffer(QAbstractPlanarVideoBufferPrivate &dd, HandleType type);

private:
    Q_DISABLE_COPY(QAbstractPlanarVideoBuffer)
};

#ifndef QT_NO_DEBUG_STREAM
Q_MULTIMEDIA_EXPORT QDebug operator<<(QDebug, QAbstractVideoBuffer::HandleType);
Q_MULTIMEDIA_EXPORT QDebug operator<<(QDebug, QAbstractVideoBuffer::MapMode);
#endif

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QAbstractVideoBuffer::HandleType)
Q_DECLARE_METATYPE(QAbstractVideoBuffer::MapMode)

#endif
