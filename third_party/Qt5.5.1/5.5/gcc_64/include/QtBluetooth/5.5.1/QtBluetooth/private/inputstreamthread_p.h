/****************************************************************************
**
** Copyright (C) 2013 Lauri Laanmets (Proekspert AS) <lauri.laanmets@eesti.ee>
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

#ifndef INPUTSTREAMTHREAD_H
#define INPUTSTREAMTHREAD_H

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
#include <QtCore/QMutex>
#include <QtAndroidExtras/QAndroidJniObject>
#include <jni.h>

QT_BEGIN_NAMESPACE

class QBluetoothSocketPrivate;

class InputStreamThread : public QObject
{
    Q_OBJECT
public:
    explicit InputStreamThread(QBluetoothSocketPrivate *socket_p);

    bool bytesAvailable() const;
    bool run();

    qint64 readData(char *data, qint64 maxSize);
    void javaThreadErrorOccurred(int errorCode);
    void javaReadyRead(jbyteArray buffer, int bufferLength);

    void prepareForClosure();

signals:
    void dataAvailable();
    void error(int errorCode);

private:
    QBluetoothSocketPrivate *m_socket_p;
    QAndroidJniObject javaInputStreamThread;
    mutable QMutex m_mutex;
    bool expectClosure;
};

QT_END_NAMESPACE

#endif // INPUTSTREAMTHREAD_H
