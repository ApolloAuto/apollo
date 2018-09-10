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

#ifndef QWEBSOCKETDATAPROCESSOR_P_H
#define QWEBSOCKETDATAPROCESSOR_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/QObject>
#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <QtCore/QTextCodec>
#include "qwebsocketprotocol.h"
#include "qwebsocketprotocol_p.h"

QT_BEGIN_NAMESPACE

class QIODevice;
class QWebSocketFrame;

class Q_AUTOTEST_EXPORT QWebSocketDataProcessor : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(QWebSocketDataProcessor)

public:
    explicit QWebSocketDataProcessor(QObject *parent = Q_NULLPTR);
    virtual ~QWebSocketDataProcessor();

    static quint64 maxMessageSize();
    static quint64 maxFrameSize();

Q_SIGNALS:
    void pingReceived(const QByteArray &data);
    void pongReceived(const QByteArray &data);
    void closeReceived(QWebSocketProtocol::CloseCode closeCode, const QString &closeReason);
    void textFrameReceived(const QString &frame, bool lastFrame);
    void binaryFrameReceived(const QByteArray &frame, bool lastFrame);
    void textMessageReceived(const QString &message);
    void binaryMessageReceived(const QByteArray &message);
    void errorEncountered(QWebSocketProtocol::CloseCode code, const QString &description);

public Q_SLOTS:
    void process(QIODevice *pIoDevice);
    void clear();

private:
    enum
    {
        PS_READ_HEADER,
        PS_READ_PAYLOAD_LENGTH,
        PS_READ_BIG_PAYLOAD_LENGTH,
        PS_READ_MASK,
        PS_READ_PAYLOAD,
        PS_DISPATCH_RESULT
    } m_processingState;

    bool m_isFinalFrame;
    bool m_isFragmented;
    QWebSocketProtocol::OpCode m_opCode;
    bool m_isControlFrame;
    bool m_hasMask;
    quint32 m_mask;
    QByteArray m_binaryMessage;
    QString m_textMessage;
    quint64 m_payloadLength;
    QTextCodec::ConverterState *m_pConverterState;
    QTextCodec *m_pTextCodec;

    bool processControlFrame(const QWebSocketFrame &frame);
};

QT_END_NAMESPACE

#endif // QWEBSOCKETDATAPROCESSOR_P_H
