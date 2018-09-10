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

#ifndef OSXBTL2CAPCHANNEL_P_H
#define OSXBTL2CAPCHANNEL_P_H

#include <QtCore/qglobal.h>

#include <Foundation/Foundation.h>
// Only after Foundation.h:
#include "corebluetoothwrapper_p.h"

#include <cstddef>

QT_BEGIN_NAMESPACE

class QBluetoothAddress;

namespace OSXBluetooth {

class ChannelDelegate;

}

QT_END_NAMESPACE

@class IOBluetoothDevice;

@interface QT_MANGLE_NAMESPACE(OSXBTL2CAPChannel) : NSObject<IOBluetoothL2CAPChannelDelegate>
{
    QT_PREPEND_NAMESPACE(OSXBluetooth)::ChannelDelegate *delegate;
    IOBluetoothDevice *device;
    IOBluetoothL2CAPChannel *channel;
    bool connected;
}

- (id)initWithDelegate:(QT_PREPEND_NAMESPACE(OSXBluetooth)::ChannelDelegate *)aDelegate;
- (id)initWithDelegate:(QT_PREPEND_NAMESPACE(OSXBluetooth)::ChannelDelegate *)aDelegate
      channel:(IOBluetoothL2CAPChannel *)aChannel;

- (void)dealloc;

// Async. connection (connect can be called only once).
- (IOReturn)connectAsyncToDevice:(const QT_PREPEND_NAMESPACE(QBluetoothAddress) &)address
            withPSM:(BluetoothL2CAPChannelID)psm;

// IOBluetoothL2CAPChannelDelegate:
- (void)l2capChannelData:(IOBluetoothL2CAPChannel*)l2capChannel
        data:(void *)dataPointer length:(size_t)dataLength;
- (void)l2capChannelOpenComplete:(IOBluetoothL2CAPChannel*)
        l2capChannel status:(IOReturn)error;
- (void)l2capChannelClosed:(IOBluetoothL2CAPChannel*)l2capChannel;
- (void)l2capChannelReconfigured:(IOBluetoothL2CAPChannel*)l2capChannel;
- (void)l2capChannelWriteComplete:(IOBluetoothL2CAPChannel*)l2capChannel
        refcon:(void*)refcon status:(IOReturn)error;
- (void)l2capChannelQueueSpaceAvailable:(IOBluetoothL2CAPChannel*)l2capChannel;

//
- (BluetoothL2CAPPSM)getPSM;
- (BluetoothDeviceAddress)peerAddress;
- (NSString *)peerName;
- (bool)isConnected;

// Writes the given data synchronously over the target L2CAP channel to the remote
// device.
// The length of the data may not exceed the L2CAP channel's outgoing MTU.
// This method will block until the data has been successfully sent to the
// hardware for transmission (or an error occurs).
- (IOReturn) writeSync:(void*)data length:(UInt16)length;

// The length of the data may not exceed the L2CAP channel's outgoing MTU.
// When the data has been successfully passed to the hardware to be transmitted,
// the delegate method -l2capChannelWriteComplete:refcon:status: will be called.
// Returns kIOReturnSuccess if the data was buffered successfully.
- (IOReturn) writeAsync:(void*)data length:(UInt16)length;

@end

#endif
