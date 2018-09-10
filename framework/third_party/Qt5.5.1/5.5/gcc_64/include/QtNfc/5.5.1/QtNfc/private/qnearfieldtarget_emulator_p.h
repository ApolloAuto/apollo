/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtNfc module of the Qt Toolkit.
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

#ifndef QNEARFIELDTARGET_EMULATOR_P_H
#define QNEARFIELDTARGET_EMULATOR_P_H

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

#include "qnearfieldtagtype1_p.h"
#include "qnearfieldtagtype2_p.h"
#include "targetemulator_p.h"

#include <QtCore/QMap>

QT_BEGIN_NAMESPACE

class TagType1 : public QNearFieldTagType1
{
    Q_OBJECT

public:
    TagType1(TagBase *tag, QObject *parent);
    ~TagType1();

    QByteArray uid() const;

    AccessMethods accessMethods() const;

    RequestId sendCommand(const QByteArray &command);
    bool waitForRequestCompleted(const RequestId &id, int msecs = 5000);

private:
    TagBase *m_tag;
};

class TagType2 : public QNearFieldTagType2
{
    Q_OBJECT

public:
    TagType2(TagBase *tag, QObject *parent);
    ~TagType2();

    QByteArray uid() const;

    AccessMethods accessMethods() const;

    RequestId sendCommand(const QByteArray &command);
    bool waitForRequestCompleted(const RequestId &id, int msecs = 5000);

private:
    TagBase *m_tag;
};

class TagActivator : public QObject
{
    Q_OBJECT

public:
    TagActivator();
    ~TagActivator();

    void initialize();
    void reset();

    static TagActivator *instance();

protected:
    void timerEvent(QTimerEvent *e);

signals:
    void tagActivated(TagBase *tag);
    void tagDeactivated(TagBase *tag);

private:
    QMap<TagBase *, bool>::Iterator m_current;
    int timerId;
};

QT_END_NAMESPACE

#endif // QNEARFIELDTARGET_EMULATOR_P_H
