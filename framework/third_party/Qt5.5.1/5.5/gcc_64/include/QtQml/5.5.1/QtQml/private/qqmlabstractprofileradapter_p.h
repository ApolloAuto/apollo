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

#ifndef QQMLABSTRACTPROFILERADAPTER_P_H
#define QQMLABSTRACTPROFILERADAPTER_P_H

#include <private/qtqmlglobal_p.h>
#include <private/qqmlprofilerdefinitions_p.h>

#include <QtCore/QObject>
#include <QtCore/QElapsedTimer>

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

class QQmlProfilerService;
class Q_QML_PRIVATE_EXPORT QQmlAbstractProfilerAdapter : public QObject, public QQmlProfilerDefinitions {
    Q_OBJECT

public:
    QQmlAbstractProfilerAdapter(QQmlProfilerService *service) :
        service(service), waiting(true), featuresEnabled(0) {}
    virtual ~QQmlAbstractProfilerAdapter() {}

    virtual qint64 sendMessages(qint64 until, QList<QByteArray> &messages) = 0;

    void startProfiling(quint64 features);

    void stopProfiling();

    void reportData() { emit dataRequested(); }

    void stopWaiting() { waiting = false; }
    void startWaiting() { waiting = true; }

    bool isRunning() const { return featuresEnabled != 0; }
    quint64 features() const { return featuresEnabled; }

    void synchronize(const QElapsedTimer &t) { emit referenceTimeKnown(t); }

signals:
    void profilingEnabled(quint64 features);
    void profilingEnabledWhileWaiting(quint64 features);

    void profilingDisabled();
    void profilingDisabledWhileWaiting();

    void dataRequested();
    void referenceTimeKnown(const QElapsedTimer &timer);

protected:
    QQmlProfilerService *service;

private:
    bool waiting;
    quint64 featuresEnabled;
};

QT_END_NAMESPACE

#endif // QQMLABSTRACTPROFILERADAPTER_P_H
