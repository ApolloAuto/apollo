/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#ifndef QNONCONTIGUOUSBYTEDEVICE_P_H
#define QNONCONTIGUOUSBYTEDEVICE_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of a number of Qt sources files.  This header file may change from
// version to version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qobject.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qbuffer.h>
#include <QtCore/qiodevice.h>
#include <QtCore/QSharedPointer>
#include "private/qringbuffer_p.h"

QT_BEGIN_NAMESPACE

class Q_CORE_EXPORT QNonContiguousByteDevice : public QObject
{
    Q_OBJECT
public:
    virtual const char* readPointer(qint64 maximumLength, qint64 &len) = 0;
    virtual bool advanceReadPointer(qint64 amount) = 0;
    virtual bool atEnd() = 0;
    virtual qint64 pos() { return -1; }
    virtual bool reset() = 0;
    virtual qint64 size() = 0;

    virtual ~QNonContiguousByteDevice();

protected:
    QNonContiguousByteDevice();


Q_SIGNALS:
    void readyRead();
    void readProgress(qint64 current, qint64 total);
};

class Q_CORE_EXPORT QNonContiguousByteDeviceFactory
{
public:
    static QNonContiguousByteDevice* create(QIODevice *device);
    static QSharedPointer<QNonContiguousByteDevice> createShared(QIODevice *device);

    static QNonContiguousByteDevice* create(QByteArray *byteArray);
    static QSharedPointer<QNonContiguousByteDevice> createShared(QByteArray *byteArray);

    static QNonContiguousByteDevice* create(QSharedPointer<QRingBuffer> ringBuffer);
    static QSharedPointer<QNonContiguousByteDevice> createShared(QSharedPointer<QRingBuffer> ringBuffer);

    static QIODevice* wrap(QNonContiguousByteDevice* byteDevice);
};

// the actual implementations
//

class QNonContiguousByteDeviceByteArrayImpl : public QNonContiguousByteDevice
{
public:
    QNonContiguousByteDeviceByteArrayImpl(QByteArray *ba);
    ~QNonContiguousByteDeviceByteArrayImpl();
    const char* readPointer(qint64 maximumLength, qint64 &len) Q_DECL_OVERRIDE;
    bool advanceReadPointer(qint64 amount) Q_DECL_OVERRIDE;
    bool atEnd() Q_DECL_OVERRIDE;
    bool reset() Q_DECL_OVERRIDE;
    qint64 size() Q_DECL_OVERRIDE;
    qint64 pos() Q_DECL_OVERRIDE;
protected:
    QByteArray* byteArray;
    qint64 currentPosition;
};

class QNonContiguousByteDeviceRingBufferImpl : public QNonContiguousByteDevice
{
public:
    QNonContiguousByteDeviceRingBufferImpl(QSharedPointer<QRingBuffer> rb);
    ~QNonContiguousByteDeviceRingBufferImpl();
    const char* readPointer(qint64 maximumLength, qint64 &len) Q_DECL_OVERRIDE;
    bool advanceReadPointer(qint64 amount) Q_DECL_OVERRIDE;
    bool atEnd() Q_DECL_OVERRIDE;
    bool reset() Q_DECL_OVERRIDE;
    qint64 size() Q_DECL_OVERRIDE;
    qint64 pos() Q_DECL_OVERRIDE;
protected:
    QSharedPointer<QRingBuffer> ringBuffer;
    qint64 currentPosition;
};


class QNonContiguousByteDeviceIoDeviceImpl : public QNonContiguousByteDevice
{
    Q_OBJECT
public:
    QNonContiguousByteDeviceIoDeviceImpl(QIODevice *d);
    ~QNonContiguousByteDeviceIoDeviceImpl();
    const char* readPointer(qint64 maximumLength, qint64 &len) Q_DECL_OVERRIDE;
    bool advanceReadPointer(qint64 amount) Q_DECL_OVERRIDE;
    bool atEnd() Q_DECL_OVERRIDE;
    bool reset() Q_DECL_OVERRIDE;
    qint64 size() Q_DECL_OVERRIDE;
    qint64 pos() Q_DECL_OVERRIDE;
protected:
    QIODevice* device;
    QByteArray* currentReadBuffer;
    qint64 currentReadBufferSize;
    qint64 currentReadBufferAmount;
    qint64 currentReadBufferPosition;
    qint64 totalAdvancements;
    bool eof;
    qint64 initialPosition;
};

class QNonContiguousByteDeviceBufferImpl : public QNonContiguousByteDevice
{
    Q_OBJECT
public:
    QNonContiguousByteDeviceBufferImpl(QBuffer *b);
    ~QNonContiguousByteDeviceBufferImpl();
    const char* readPointer(qint64 maximumLength, qint64 &len) Q_DECL_OVERRIDE;
    bool advanceReadPointer(qint64 amount) Q_DECL_OVERRIDE;
    bool atEnd() Q_DECL_OVERRIDE;
    bool reset() Q_DECL_OVERRIDE;
    qint64 size() Q_DECL_OVERRIDE;
protected:
    QBuffer* buffer;
    QByteArray byteArray;
    QNonContiguousByteDeviceByteArrayImpl* arrayImpl;
};

// ... and the reverse thing
class QByteDeviceWrappingIoDevice : public QIODevice
{
public:
    QByteDeviceWrappingIoDevice (QNonContiguousByteDevice *bd);
    ~QByteDeviceWrappingIoDevice ();
    virtual bool isSequential () const Q_DECL_OVERRIDE;
    virtual bool atEnd () const Q_DECL_OVERRIDE;
    virtual bool reset () Q_DECL_OVERRIDE;
    virtual qint64 size () const Q_DECL_OVERRIDE;
protected:
     virtual qint64 readData ( char * data, qint64 maxSize ) Q_DECL_OVERRIDE;
     virtual qint64 writeData ( const char * data, qint64 maxSize ) Q_DECL_OVERRIDE;

     QNonContiguousByteDevice *byteDevice;
};

QT_END_NAMESPACE

#endif
