/****************************************************************************
**
** Copyright (C) 2014 Kurt Pattyn <pattyn.kurt@gmail.com>.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWebSockets module of the Qt Toolkit.
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

#ifndef QWEBSOCKETPROTOCOL_P_H
#define QWEBSOCKETPROTOCOL_P_H

#include <QtCore/qglobal.h>
#include "QtWebSockets/qwebsocketprotocol.h"

QT_BEGIN_NAMESPACE

class QString;
class QByteArray;

namespace QWebSocketProtocol
{
enum OpCode
{
    OpCodeContinue    = 0x0,
    OpCodeText        = 0x1,
    OpCodeBinary      = 0x2,
    OpCodeReserved3   = 0x3,
    OpCodeReserved4   = 0x4,
    OpCodeReserved5   = 0x5,
    OpCodeReserved6   = 0x6,
    OpCodeReserved7   = 0x7,
    OpCodeClose       = 0x8,
    OpCodePing        = 0x9,
    OpCodePong        = 0xA,
    OpCodeReservedB   = 0xB,
    OpCodeReservedC   = 0xC,
    OpCodeReservedD   = 0xD,
    OpCodeReservedE   = 0xE,
    OpCodeReservedF   = 0xF
};

inline bool isOpCodeReserved(OpCode code)
{
    return ((code > OpCodeBinary) && (code < OpCodeClose)) || (code > OpCodePong);
}

inline bool isCloseCodeValid(int closeCode)
{
    return  (closeCode > 999) && (closeCode < 5000) &&
            (closeCode != CloseCodeReserved1004) &&          //see RFC6455 7.4.1
            (closeCode != CloseCodeMissingStatusCode) &&
            (closeCode != CloseCodeAbnormalDisconnection) &&
            ((closeCode >= 3000) || (closeCode < 1012));
}

inline Version currentVersion() { return VersionLatest; }
Version Q_AUTOTEST_EXPORT versionFromString(const QString &versionString);

void Q_AUTOTEST_EXPORT mask(QByteArray *payload, quint32 maskingKey);
void Q_AUTOTEST_EXPORT mask(char *payload, quint64 size, quint32 maskingKey);
}	//end namespace QWebSocketProtocol

QT_END_NAMESPACE

#endif // QWEBSOCKETPROTOCOL_P_H
