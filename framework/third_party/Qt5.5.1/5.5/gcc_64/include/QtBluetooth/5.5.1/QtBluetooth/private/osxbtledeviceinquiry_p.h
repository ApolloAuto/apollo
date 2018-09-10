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

#ifndef OSXBTLEDEVICEINQUIRY_P_H
#define OSXBTLEDEVICEINQUIRY_P_H

#include "qbluetoothdevicediscoveryagent.h"

#include <QtCore/qdatetime.h>
#include <QtCore/qglobal.h>
#include <QtCore/qlist.h>

#include <Foundation/Foundation.h>

@class QT_MANGLE_NAMESPACE(OSXBTLEDeviceInquiry);

@class CBCentralManager;
@class CBPeripheral;

QT_BEGIN_NAMESPACE

class QBluetoothDeviceInfo;
class QBluetoothUuid;

namespace OSXBluetooth {

class LEDeviceInquiryDelegate
{
public:
    typedef QT_MANGLE_NAMESPACE(OSXBTLEDeviceInquiry) LEDeviceInquiryObjC;

    virtual ~LEDeviceInquiryDelegate();

    // At the moment the only error we're reporting is PoweredOffError!
    virtual void LEdeviceInquiryError(QBluetoothDeviceDiscoveryAgent::Error error) = 0;

    virtual void LEnotSupported() = 0;
    virtual void LEdeviceFound(CBPeripheral *peripheral, const QBluetoothUuid &uuid,
                               NSDictionary *advertisementData, NSNumber *RSSI) = 0;
    virtual void LEdeviceInquiryFinished() = 0;
};

}

QT_END_NAMESPACE

// Bluetooth Low Energy scan for iOS and OS X.

@interface QT_MANGLE_NAMESPACE(OSXBTLEDeviceInquiry) : NSObject
{// Protocols are adopted in the mm file.
    QT_PREPEND_NAMESPACE(OSXBluetooth)::LEDeviceInquiryDelegate *delegate;

    // TODO: scoped pointers/shared pointers?
    NSMutableDictionary *peripherals; // Found devices.
    CBCentralManager *manager;

    // pending - waiting for a status update first.
    bool pendingStart;
    bool cancelled;
    // scan actually started.
    bool isActive;
    QTime startTime;
}

// Inquiry length in milliseconds.
+ (int)inquiryLength;

- (id)initWithDelegate:(QT_PREPEND_NAMESPACE(OSXBluetooth)::LEDeviceInquiryDelegate *)aDelegate;
- (void)dealloc;

// Actual scan can be delayed - we have to wait for a status update first.
- (bool)start;
// Stop can be delayed - if we're waiting for a status update.
- (void)stop;

- (bool)isActive;
- (const QTime &)startTime;

@end

#endif
