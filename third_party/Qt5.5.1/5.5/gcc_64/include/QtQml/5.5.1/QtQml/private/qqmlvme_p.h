/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQMLVME_P_H
#define QQMLVME_P_H

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

#include "qqmlerror.h"
#include <private/qbitfield_p.h>
#include <private/qrecursionwatcher_p.h>

#include <QtCore/QStack>
#include <QtCore/QString>
#include <QtCore/qelapsedtimer.h>
#include <QtCore/qcoreapplication.h>
#include <QtCore/qtypeinfo.h>

#include <private/qqmlengine_p.h>
#include <private/qfinitestack_p.h>

#include <private/qqmlprofiler_p.h>

QT_BEGIN_NAMESPACE

class QObject;
class QJSValue;
class QQmlScriptData;
class QQmlCompiledData;
class QQmlContextData;

namespace QQmlVMETypes {
    struct List
    {
        List() : type(0) {}
        List(int t) : type(t) {}

        int type;
        QQmlListProperty<void> qListProperty;
    };
    struct State {
        enum Flag { Deferred = 0x00000001 };

        State() : flags(0), context(0), compiledData(0), instructionStream(0) {}
        quint32 flags;
        QQmlContextData *context;
        QQmlCompiledData *compiledData;
        const char *instructionStream;
        QBitField bindingSkipList;
    };
}
Q_DECLARE_TYPEINFO(QQmlVMETypes::List, Q_PRIMITIVE_TYPE  | Q_MOVABLE_TYPE);
template<>
class QTypeInfo<QQmlVMETypes::State> : public QTypeInfoMerger<QQmlVMETypes::State, QBitField> {}; //Q_DECLARE_TYPEINFO

class QQmlInstantiationInterrupt {
public:
    inline QQmlInstantiationInterrupt();
    inline QQmlInstantiationInterrupt(volatile bool *runWhile, int nsecs=0);
    inline QQmlInstantiationInterrupt(int nsecs);

    inline void reset();
    inline bool shouldInterrupt() const;
private:
    enum Mode { None, Time, Flag };
    Mode mode;
    QElapsedTimer timer;
    int nsecs;
    volatile bool *runWhile;
};

class Q_QML_PRIVATE_EXPORT QQmlVME
{
public:
    static void enableComponentComplete();
    static void disableComponentComplete();
    static bool componentCompleteEnabled();

private:
    static bool s_enableComponentComplete;
};

// Used to check that a QQmlVME that is interrupted mid-execution
// is still valid.  Checks all the objects and contexts have not been
// deleted.
class QQmlVMEGuard
{
public:
    QQmlVMEGuard();
    ~QQmlVMEGuard();

    void guard(QQmlObjectCreator *);
    void clear();

    bool isOK() const;

private:
    int m_objectCount;
    QPointer<QObject> *m_objects;
    int m_contextCount;
    QQmlGuardedContextData *m_contexts;
};

QQmlInstantiationInterrupt::QQmlInstantiationInterrupt()
    : mode(None), nsecs(0), runWhile(0)
{
}

QQmlInstantiationInterrupt::QQmlInstantiationInterrupt(volatile bool *runWhile, int nsecs)
    : mode(Flag), nsecs(nsecs), runWhile(runWhile)
{
}

QQmlInstantiationInterrupt::QQmlInstantiationInterrupt(int nsecs)
    : mode(Time), nsecs(nsecs), runWhile(0)
{
}

void QQmlInstantiationInterrupt::reset()
{
    if (mode == Time || nsecs)
        timer.start();
}

bool QQmlInstantiationInterrupt::shouldInterrupt() const
{
    if (mode == None) {
        return false;
    } else if (mode == Time) {
        return timer.nsecsElapsed() > nsecs;
    } else if (mode == Flag) {
        return !*runWhile || (nsecs && timer.nsecsElapsed() > nsecs);
    } else {
        return false;
    }
}

QT_END_NAMESPACE

#endif // QQMLVME_P_H
