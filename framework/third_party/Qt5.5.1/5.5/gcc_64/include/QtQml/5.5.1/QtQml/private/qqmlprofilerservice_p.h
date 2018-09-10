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

#ifndef QQMLPROFILERSERVICE_P_H
#define QQMLPROFILERSERVICE_P_H

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

#include "qqmlconfigurabledebugservice_p.h"
#include "qqmlprofilerdefinitions_p.h"
#include "qqmlabstractprofileradapter_p.h"

#include <private/qqmlboundsignal_p.h>

#include <QtCore/qelapsedtimer.h>
#include <QtCore/qmetaobject.h>
#include <QtCore/qmutex.h>
#include <QtCore/qvector.h>
#include <QtCore/qstringbuilder.h>
#include <QtCore/qwaitcondition.h>

#include <limits>

QT_BEGIN_NAMESPACE

class QUrl;
class QQmlEngine;


class Q_QML_PRIVATE_EXPORT QQmlProfilerService : public QQmlConfigurableDebugService, public QQmlProfilerDefinitions
{
    Q_OBJECT
public:

    static QQmlProfilerService *instance();
    void engineAboutToBeAdded(QQmlEngine *engine);
    void engineAboutToBeRemoved(QQmlEngine *engine);
    void engineAdded(QQmlEngine *engine);
    void engineRemoved(QQmlEngine *engine);

    void addGlobalProfiler(QQmlAbstractProfilerAdapter *profiler);
    void removeGlobalProfiler(QQmlAbstractProfilerAdapter *profiler);

    void startProfiling(QQmlEngine *engine, quint64 features = std::numeric_limits<quint64>::max());
    void stopProfiling(QQmlEngine *engine);

    QQmlProfilerService();
    ~QQmlProfilerService();

    void dataReady(QQmlAbstractProfilerAdapter *profiler);

protected:
    virtual void stateAboutToBeChanged(State state);
    virtual void messageReceived(const QByteArray &);

private:

    void sendMessages();
    void addEngineProfiler(QQmlAbstractProfilerAdapter *profiler, QQmlEngine *engine);
    void removeProfilerFromStartTimes(const QQmlAbstractProfilerAdapter *profiler);

    QElapsedTimer m_timer;

    QList<QQmlAbstractProfilerAdapter *> m_globalProfilers;
    QMultiHash<QQmlEngine *, QQmlAbstractProfilerAdapter *> m_engineProfilers;
    QList<QQmlEngine *> m_stoppingEngines;
    QMultiMap<qint64, QQmlAbstractProfilerAdapter *> m_startTimes;
};

QT_END_NAMESPACE

#endif // QQMLPROFILERSERVICE_P_H

