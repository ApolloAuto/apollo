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

#ifndef QQMLINCUBATOR_P_H
#define QQMLINCUBATOR_P_H

#include <private/qintrusivelist_p.h>
#include <private/qqmlvme_p.h>
#include <private/qrecursionwatcher_p.h>
#include <private/qqmlengine_p.h>
#include <private/qqmlcontext_p.h>

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

QT_BEGIN_NAMESPACE

class QQmlCompiledData;
class QQmlIncubator;
class QQmlIncubatorPrivate : public QQmlEnginePrivate::Incubator
{
public:
    QQmlIncubatorPrivate(QQmlIncubator *q, QQmlIncubator::IncubationMode m);
    ~QQmlIncubatorPrivate();

    inline static QQmlIncubatorPrivate *get(QQmlIncubator *incubator) { return incubator->d; }

    QQmlIncubator *q;

    QQmlIncubator::Status calculateStatus() const;
    void changeStatus(QQmlIncubator::Status);
    QQmlIncubator::Status status;

    QQmlIncubator::IncubationMode mode;
    bool isAsynchronous;

    QList<QQmlError> errors;

    enum Progress { Execute, Completing, Completed };
    Progress progress;

    QPointer<QObject> result;
    QQmlGuardedContextData rootContext;
    QQmlCompiledData *compiledData;
    QScopedPointer<QQmlObjectCreator> creator;
    int subComponentToCreate;
    QQmlVMEGuard vmeGuard;

    QExplicitlySharedDataPointer<QQmlIncubatorPrivate> waitingOnMe;
    typedef QQmlEnginePrivate::Incubator QIPBase;
    QIntrusiveList<QIPBase, &QIPBase::nextWaitingFor> waitingFor;

    QRecursionNode recursion;

    void clear();

    void forceCompletion(QQmlInstantiationInterrupt &i);
    void incubate(QQmlInstantiationInterrupt &i);
};

QT_END_NAMESPACE

#endif // QQMLINCUBATOR_P_H

