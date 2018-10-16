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

#ifndef QWEBSOCKETFRAME_P_H
#define QWEBSOCKETFRAME_P_H

#include <QtCore/QString>
#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <limits.h>

#include "qwebsockets_global.h"
#include "qwebsocketprotocol.h"
#include "qwebsocketprotocol_p.h"

QT_BEGIN_NAMESPACE

class QIODevice;

const quint64 MAX_FRAME_SIZE_IN_BYTES = INT_MAX - 1;
const quint64 MAX_MESSAGE_SIZE_IN_BYTES = INT_MAX - 1;

class Q_AUTOTEST_EXPORT QWebSocketFrame
{
    Q_DECLARE_TR_FUNCTIONS(QWebSocketFrame)

public:
    QWebSocketFrame();
    QWebSocketFrame(const QWebSocketFrame &other);

    QWebSocketFrame &operator =(const QWebSocketFrame &other);

#ifdef Q_COMPILER_RVALUE_REFS
    QWebSocketFrame(QWebSocketFrame &&other);
    QWebSocketFrame &operator =(QWebSocketFrame &&other);
#endif

    void swap(QWebSocketFrame &other);

    QWebSocketProtocol::CloseCode closeCode() const;
    QString closeReason() const;
    bool isFinalFrame() const;
    bool isControlFrame() const;
    bool isDataFrame() const;
    bool isContinuationFrame() const;
    bool hasMask() const;
    quint32 mask() const;    //returns 0 if no mask
    int rsv1() const;
    int rsv2() const;
    int rsv3() const;
    QWebSocketProtocol::OpCode opCode() const;
    QByteArray payload() const;

    void clear();

    bool isValid() const;

    static QWebSocketFrame readFrame(QIODevice *pIoDevice);

private:
    QWebSocketProtocol::CloseCode m_closeCode;
    QString m_closeReason;
    bool m_isFinalFrame;
    quint32 m_mask;
    int m_rsv1;
    int m_rsv2;
    int m_rsv3;
    QWebSocketProtocol::OpCode m_opCode;

    quint8 m_length;
    QByteArray m_payload;

    bool m_isValid;

    enum ProcessingState
    {
        PS_READ_HEADER,
        PS_READ_PAYLOAD_LENGTH,
        PS_READ_BIG_PAYLOAD_LENGTH,
        PS_READ_MASK,
        PS_READ_PAYLOAD,
        PS_DISPATCH_RESULT,
        PS_WAIT_FOR_MORE_DATA
    };

    void setError(QWebSocketProtocol::CloseCode code, const QString &closeReason);
    bool checkValidity();
};

QT_END_NAMESPACE

#endif // QWEBSOCKETFRAME_P_H
