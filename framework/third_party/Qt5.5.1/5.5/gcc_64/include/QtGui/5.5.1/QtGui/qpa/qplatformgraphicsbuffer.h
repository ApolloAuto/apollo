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

#ifndef QPLATFORMGRAPHICSBUFFER_H
#define QPLATFORMGRAPHICSBUFFER_H

//
//  W A R N I N G
//  -------------
//
// This file is part of the QPA API and is not meant to be used
// in applications. Usage of this API may make your code
// source and binary incompatible with future versions of Qt.
//


#include <QtCore/QSize>
#include <QtCore/QRect>
#include <QtGui/QPixelFormat>
#include <QtCore/qflags.h>
#include <QtCore/QObject>

QT_BEGIN_NAMESPACE

class Q_GUI_EXPORT QPlatformGraphicsBuffer : public QObject
{
Q_OBJECT
public:
    enum AccessType
    {
        None                = 0x00,
        SWReadAccess        = 0x01,
        SWWriteAccess       = 0x02,
        TextureAccess       = 0x04,
        HWCompositor        = 0x08
    };
    Q_DECLARE_FLAGS(AccessTypes, AccessType);

    enum Origin {
        OriginBottomLeft,
        OriginTopLeft
    };

    virtual ~QPlatformGraphicsBuffer();

    AccessTypes isLocked() const { return m_lock_access; }
    bool lock(AccessTypes access, const QRect &rect = QRect());
    void unlock();

    virtual bool bindToTexture(const QRect &rect = QRect()) const;

    virtual const uchar *data() const;
    virtual uchar *data();
    virtual int bytesPerLine() const;
    int byteCount() const;

    virtual Origin origin() const;

    QSize size() const { return m_size; }
    QPixelFormat format() const { return m_format; }

Q_SIGNALS:
    void unlocked(AccessTypes previousAccessTypes);

protected:
    QPlatformGraphicsBuffer(const QSize &size, const QPixelFormat &format);

    virtual bool doLock(AccessTypes access, const QRect &rect = QRect()) = 0;
    virtual void doUnlock() = 0;

private:
    QSize m_size;
    QPixelFormat m_format;
    AccessTypes m_lock_access;
};

QT_END_NAMESPACE

#endif //QPLATFORMGRAPHICSBUFFER_H
