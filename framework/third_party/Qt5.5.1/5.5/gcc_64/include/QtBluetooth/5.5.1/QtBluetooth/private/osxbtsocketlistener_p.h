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

#ifndef OSXBTSOCKETLISTENER_P_H
#define OSXBTSOCKETLISTENER_P_H

#include <QtCore/qglobal.h>

#include <Foundation/Foundation.h>
#include <IOBluetooth/Bluetooth.h>

@class IOBluetoothUserNotification;
@class IOBluetoothRFCOMMChannel;
@class IOBluetoothL2CAPChannel;
@class QT_MANGLE_NAMESPACE(OSXBTSocketListener);

QT_BEGIN_NAMESPACE

namespace OSXBluetooth {

class SocketListener
{
public:
    typedef QT_MANGLE_NAMESPACE(OSXBTSocketListener) ObjCListener;

    virtual ~SocketListener();

    virtual void openNotify(IOBluetoothRFCOMMChannel *channel) = 0;
    virtual void openNotify(IOBluetoothL2CAPChannel *channel) = 0;
};

}

QT_END_NAMESPACE

// A single OSXBTSocketListener can be started only once with
// RFCOMM or L2CAP protocol. It must be deleted to stop listening.

@interface QT_MANGLE_NAMESPACE(OSXBTSocketListener) : NSObject
{
    IOBluetoothUserNotification *connectionNotification;
    QT_PREPEND_NAMESPACE(OSXBluetooth::SocketListener) *delegate;
    quint16 port;
}

- (id)initWithListener:(QT_PREPEND_NAMESPACE(OSXBluetooth::SocketListener) *)aDelegate;
- (void)dealloc;

- (bool)listenRFCOMMConnectionsWithChannelID:(BluetoothRFCOMMChannelID)channelID;
- (bool)listenL2CAPConnectionsWithPSM:(BluetoothL2CAPPSM)psm;

- (void)rfcommOpenNotification:(IOBluetoothUserNotification *)notification
        channel:(IOBluetoothRFCOMMChannel *)newChannel;

- (void)l2capOpenNotification:(IOBluetoothUserNotification *)notification
        channel:(IOBluetoothL2CAPChannel *)newChannel;

- (quint16)port;

@end

#endif
