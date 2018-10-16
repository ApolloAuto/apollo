/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtBluetooth module of the Qt Toolkit.
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

#ifndef OSXBTUTILITY_P_H
#define OSXBTUTILITY_P_H

#include <QtCore/qloggingcategory.h>
#include <QtCore/qscopedpointer.h>
#include <QtCore/qsysinfo.h>
#include <QtCore/qglobal.h>

#include <Foundation/Foundation.h>
// Only after Foundation.h!
#include "corebluetoothwrapper_p.h"

@class CBUUID;

QT_BEGIN_NAMESPACE

class QBluetoothUuid;

namespace OSXBluetooth {

struct NSObjectDeleter {
    static void cleanup(NSObject *obj)
    {
        [obj release];
    }
};

template<class T>
class ObjCScopedPointer : public QScopedPointer<NSObject, NSObjectDeleter>
{
public:
    // TODO: remove default argument, add 'retain' parameter,
    // add a default ctor??? This will make the semantics more
    // transparent + will simplify the future transition to ARC
    // (if it will ever happen).
    explicit ObjCScopedPointer(T *ptr = Q_NULLPTR) : QScopedPointer(ptr){}
    operator T*() const
    {
        return data();
    }

    T *data()const
    {
        return static_cast<T *>(QScopedPointer::data());
    }

    T *take()
    {
        return static_cast<T *>(QScopedPointer::take());
    }
};

typedef ObjCScopedPointer<NSAutoreleasePool> AutoreleasePool;
#define QT_BT_MAC_AUTORELEASEPOOL const OSXBluetooth::AutoreleasePool pool([[NSAutoreleasePool alloc] init])

template<class T>
class ObjCStrongReference {
public:
    ObjCStrongReference()
        : m_ptr(nil)
    {
    }
    ObjCStrongReference(T *obj, bool retain)
    {
        if (retain)
            m_ptr = [obj retain];
        else
            m_ptr = obj; // For example, created with initWithXXXX.
    }
    ObjCStrongReference(const ObjCStrongReference &rhs)
    {
        m_ptr = [rhs.m_ptr retain];
    }
    ObjCStrongReference &operator = (const ObjCStrongReference &rhs)
    {
        // "Old-style" implementation:
        if (this != &rhs && m_ptr != rhs.m_ptr) {
            [m_ptr release];
            m_ptr = [rhs.m_ptr retain];
        }

        return *this;
    }

#ifdef Q_COMPILER_RVALUE_REFS
    ObjCStrongReference(ObjCStrongReference &&xval)
    {
        m_ptr = xval.m_ptr;
        xval.m_ptr = nil;
    }

    ObjCStrongReference &operator = (ObjCStrongReference &&xval)
    {
        m_ptr = xval.m_ptr;
        xval.m_ptr = nil;
        return *this;
    }
#endif

    ~ObjCStrongReference()
    {
        [m_ptr release];
    }

    void reset(T *newVal)
    {
        if (m_ptr != newVal) {
            [m_ptr release];
            m_ptr = [newVal retain];
        }
    }

    void resetWithoutRetain(T *newVal)
    {
        if (m_ptr != newVal) {
            [m_ptr release];
            m_ptr = newVal;
        }
    }

    operator T *() const
    {
        return m_ptr;
    }

    T *data() const
    {
        return m_ptr;
    }

    T *take()
    {
        T * p = m_ptr;
        m_ptr = nil;
        return p;
    }
private:
    T *m_ptr;
};

// The type 'T' is some XXXRef from CoreFoundation and co.
// In principle, we can do a trick removing a pointer from a type
// when template is instantiated, but it's quite a lot of ugly pp-tokens
// like OSXBluetooth::CFStrongReference<OSXBluetooth::remove_pointer<CFUUIDRref> > strongReference;
// so instead we use 'T' everywhere, not 'T *' as can expected
// from a smart pointer.
template<class T>
class CFStrongReference {
public:
    CFStrongReference()
        : m_ptr(Q_NULLPTR)
    {
    }

    CFStrongReference(T obj, bool retain)
        : m_ptr(obj)
    {
        if (m_ptr && retain)
            CFRetain(m_ptr);
    }

    CFStrongReference(const CFStrongReference &rhs)
    {
        if ((m_ptr = rhs.m_ptr))
            CFRetain(m_ptr);
    }

    CFStrongReference &operator = (const CFStrongReference &rhs)
    {
        // "Old-style" implementation:
        if (this != &rhs && m_ptr != rhs.m_ptr) {
            if (m_ptr)
                CFRelease(m_ptr);
            if ((m_ptr = rhs.m_ptr))
                CFRetain(m_ptr);
        }

        return *this;
    }

#ifdef Q_COMPILER_RVALUE_REFS
    CFStrongReference(CFStrongReference &&xval)
    {
        m_ptr = xval.m_ptr;
        xval.m_ptr = Q_NULLPTR;
    }

    CFStrongReference &operator = (CFStrongReference &&xval)
    {
        m_ptr = xval.m_ptr;
        xval.m_ptr = Q_NULLPTR;
        return *this;
    }
#endif

    ~CFStrongReference()
    {
        if (m_ptr)
            CFRelease(m_ptr);
    }

    void reset(T newVal)
    {
        if (m_ptr != newVal) {
            if (m_ptr)
                CFRelease(m_ptr);
            if ((m_ptr = newVal))
                CFRetain(m_ptr);
        }
    }

    operator T() const
    {
        return m_ptr;
    }

    T data() const
    {
        return m_ptr;
    }

    T take()
    {
        T p = m_ptr;
        m_ptr = Q_NULLPTR;
        return p;
    }
private:
    T m_ptr;
};

QString qt_address(NSString *address);

#ifndef QT_IOS_BLUETOOTH

class QBluetoothAddress qt_address(const BluetoothDeviceAddress *address);
BluetoothDeviceAddress iobluetooth_address(const QBluetoothAddress &address);

ObjCStrongReference<IOBluetoothSDPUUID> iobluetooth_uuid(const QBluetoothUuid &uuid);
QBluetoothUuid qt_uuid(IOBluetoothSDPUUID *uuid);
QString qt_error_string(IOReturn errorCode);

#endif

QBluetoothUuid qt_uuid(CBUUID *uuid);
CFStrongReference<CFUUIDRef> cf_uuid(const QBluetoothUuid &qtUuid);
ObjCStrongReference<CBUUID> cb_uuid(const QBluetoothUuid &qtUuid);
bool equal_uuids(const QBluetoothUuid &qtUuid, CBUUID *cbUuid);
bool equal_uuids(CBUUID *cbUuid, const QBluetoothUuid &qtUuid);
QByteArray qt_bytearray(NSData *data);
QByteArray qt_bytearray(NSObject *data);
ObjCStrongReference<NSData> data_from_bytearray(const QByteArray & qtData);

inline QSysInfo::MacVersion qt_OS_limit(QSysInfo::MacVersion osxVersion, QSysInfo::MacVersion iosVersion)
{
#ifdef Q_OS_OSX
    Q_UNUSED(iosVersion)
    return osxVersion;
#else
    Q_UNUSED(osxVersion)
    return iosVersion;
#endif
}

} // namespace OSXBluetooth

// Logging category for both OS X and iOS.
Q_DECLARE_LOGGING_CATEGORY(QT_BT_OSX)

QT_END_NAMESPACE

#endif
