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

#include <QtCore/qvariant.h>
#include <QtCore/qglobal.h>

#include <Foundation/Foundation.h>
// Only after Foundation.h:
#include "corebluetoothwrapper_p.h"

@class IOBluetoothOBEXSession;
@class IOBluetoothDevice;

@class QT_MANGLE_NAMESPACE(OSXBTOBEXSession);

QT_BEGIN_NAMESPACE

class QBluetoothAddress;
class QIODevice;
class QString;

namespace OSXBluetooth
{

class OBEXSessionDelegate
{
public:
    typedef QT_MANGLE_NAMESPACE(OSXBTOBEXSession) ObjCOBEXSession;

    virtual ~OBEXSessionDelegate();

    virtual void OBEXConnectError(OBEXError error, OBEXOpCode responseCode) = 0;
    virtual void OBEXConnectSuccess() = 0;

    virtual void OBEXAbortSuccess() = 0;

    virtual void OBEXPutDataSent(quint32 current, quint32 total) = 0;
    virtual void OBEXPutSuccess() = 0;
    virtual void OBEXPutError(OBEXError error, OBEXOpCode responseCode) = 0;
};

enum OBEXRequest {
    OBEXNoop,
    OBEXConnect,
    OBEXDisconnect,
    OBEXPut,
    OBEXGet,
    OBEXSetPath,
    OBEXAbort
};

}

QT_END_NAMESPACE

// OBEX Session, it's a "single-shot" operation as our QBluetoothTransferReply is
// (it does not have an interface to re-send data or re-use the same transfer reply).
// It either succeeds or fails and tries to cleanup in any case.
@interface QT_MANGLE_NAMESPACE(OSXBTOBEXSession) : NSObject
{
    QT_PREPEND_NAMESPACE(OSXBluetooth)::OBEXSessionDelegate *delegate;
    IOBluetoothDevice *device;
    quint16 channelID;
    IOBluetoothOBEXSession *session;

    QT_PREPEND_NAMESPACE(OSXBluetooth)::OBEXRequest currentRequest;

    bool connected;
    bool connectionIDFound;
    quint32 connectionID;

    QT_PREPEND_NAMESPACE(QIODevice) *inputStream;

    // TODO: switch to scoped pointers or strong reference objects instead.
    NSMutableData *headersData;
    NSMutableData *bodyData;

    quint32 bytesSent;
    bool pendingAbort;
}

- (id)initWithDelegate:(QT_PREPEND_NAMESPACE(OSXBluetooth::OBEXSessionDelegate) *)aDelegate
      remoteDevice:(const QBluetoothAddress &)deviceAddress channelID:(quint16)port;

- (void)dealloc;

// Below I have pairs: OBEX operation and its callback method.
- (OBEXError)OBEXConnect;
- (void)OBEXConnectHandler:(const OBEXSessionEvent*)event;

- (OBEXError)OBEXAbort;
- (void)OBEXAbortHandler:(const OBEXSessionEvent*)event;

- (OBEXError)OBEXPutFile:(QT_PREPEND_NAMESPACE(QIODevice) *)inputStream withName:(const QString &)name;
- (void)OBEXPutHandler:(const OBEXSessionEvent*)event;

// Aux. methods.
- (bool)isConnected;

// To be called from C++ destructors. OBEXSession is not
// valid anymore after this call (no more OBEX operations
// can be executed). It's an ABORT/DISCONNECT sequence.
// It also resets a delegate to null.
- (void)closeSession;
//
- (bool)hasActiveRequest;

@end
