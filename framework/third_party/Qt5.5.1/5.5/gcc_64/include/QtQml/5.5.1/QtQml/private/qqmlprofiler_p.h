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

#ifndef QQMLPROFILER_P_H
#define QQMLPROFILER_P_H

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

#include <private/qv4function_p.h>
#include <private/qqmlboundsignal_p.h>
#include <private/qfinitestack_p.h>
#include "qqmlprofilerdefinitions_p.h"
#include "qqmlabstractprofileradapter_p.h"

#include <QUrl>
#include <QString>

QT_BEGIN_NAMESPACE

#define Q_QML_PROFILE_IF_ENABLED(feature, profiler, Code)\
    if (profiler && (profiler->featuresEnabled & (1 << feature))) {\
        Code;\
    } else\
        (void)0

#define Q_QML_PROFILE(feature, profiler, Method)\
    Q_QML_PROFILE_IF_ENABLED(feature, profiler, profiler->Method)

// This struct is somewhat dangerous to use:
// The messageType is a bit field. You can pack multiple messages into
// one object, e.g. RangeStart and RangeLocation. Each one will be read
// independently by toByteArrays. Thus you can only pack messages if their data
// doesn't overlap. It's up to you to figure that out.
struct Q_AUTOTEST_EXPORT QQmlProfilerData
{
    QQmlProfilerData() {}

    QQmlProfilerData(qint64 time, int messageType, int detailType, const QUrl &url,
                     int x = 0, int y = 0) :
        time(time), messageType(messageType), detailType(detailType), detailUrl(url),
        x(x), y(y) {}

    QQmlProfilerData(qint64 time, int messageType, int detailType, const QString &str,
                     int x = 0, int y = 0) :
        time(time), messageType(messageType), detailType(detailType),detailString(str),
        x(x), y(y) {}

    QQmlProfilerData(qint64 time, int messageType, int detailType, const QString &str,
                     const QUrl &url, int x = 0, int y = 0) :
        time(time), messageType(messageType), detailType(detailType), detailString(str),
        detailUrl(url), x(x), y(y) {}


    QQmlProfilerData(qint64 time, int messageType, int detailType) :
        time(time), messageType(messageType), detailType(detailType) {}


    qint64 time;
    int messageType;        //bit field of QQmlProfilerService::Message
    int detailType;

    QString detailString;   //used by RangeData and possibly by RangeLocation
    QUrl detailUrl;         //used by RangeLocation, overrides detailString

    int x;                  //used by RangeLocation
    int y;                  //used by RangeLocation

    void toByteArrays(QList<QByteArray> &messages) const;
};

Q_DECLARE_TYPEINFO(QQmlProfilerData, Q_MOVABLE_TYPE);

class QQmlProfiler : public QObject, public QQmlProfilerDefinitions {
    Q_OBJECT
public:
    void startBinding(const QQmlSourceLocation &location)
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(),
                                       (1 << RangeStart | 1 << RangeLocation), 1 << Binding,
                                       location.sourceFile, qmlSourceCoordinate(location.line), qmlSourceCoordinate(location.column)));
    }

    // Have toByteArrays() construct another RangeData event from the same QString later.
    // This is somewhat pointless but important for backwards compatibility.
    void startCompiling(const QString &name)
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(),
                                       (1 << RangeStart | 1 << RangeLocation | 1 << RangeData),
                                       1 << Compiling, name, 1, 1));
    }

    void startHandlingSignal(const QQmlSourceLocation &location)
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(),
                                       (1 << RangeStart | 1 << RangeLocation), 1 << HandlingSignal,
                                       location.sourceFile, location.line, location.column));
    }

    void startCreating()
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(), 1 << RangeStart, 1 << Creating));
    }

    void startCreating(const QString &typeName, const QUrl &fileName, int line, int column)
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(),
                                       (1 << RangeStart | 1 << RangeLocation | 1 << RangeData),
                                       1 << Creating, typeName, fileName, line, column));
    }

    void updateCreating(const QString &typeName, const QUrl &fileName, int line, int column)
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(),
                                       (1 << RangeLocation | 1 << RangeData),
                                       1 << Creating, typeName, fileName, line, column));
    }

    template<RangeType Range>
    void endRange()
    {
        m_data.append(QQmlProfilerData(m_timer.nsecsElapsed(), 1 << RangeEnd, 1 << Range));
    }

    QQmlProfiler();

    quint64 featuresEnabled;

public slots:
    void startProfiling(quint64 features);
    void stopProfiling();
    void reportData();
    void setTimer(const QElapsedTimer &timer) { m_timer = timer; }

signals:
    void dataReady(const QList<QQmlProfilerData> &);

protected:
    QElapsedTimer m_timer;
    QVarLengthArray<QQmlProfilerData> m_data;
};

class QQmlProfilerAdapter : public QQmlAbstractProfilerAdapter {
    Q_OBJECT
public:
    QQmlProfilerAdapter(QQmlProfilerService *service, QQmlEnginePrivate *engine);
    qint64 sendMessages(qint64 until, QList<QByteArray> &messages);

public slots:
    void receiveData(const QList<QQmlProfilerData> &new_data);

private:
    QList<QQmlProfilerData> data;
};

//
// RAII helper structs
//

struct QQmlProfilerHelper : public QQmlProfilerDefinitions {
    QQmlProfiler *profiler;
    QQmlProfilerHelper(QQmlProfiler *profiler) : profiler(profiler) {}
};

struct QQmlBindingProfiler : public QQmlProfilerHelper {
    QQmlBindingProfiler(QQmlProfiler *profiler, const QV4::FunctionObject *function) :
        QQmlProfilerHelper(profiler)
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileBinding, profiler,
                      startBinding(function->sourceLocation()));
    }

    ~QQmlBindingProfiler()
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileBinding, profiler,
                      endRange<Binding>());
    }
};

struct QQmlHandlingSignalProfiler : public QQmlProfilerHelper {
    QQmlHandlingSignalProfiler(QQmlProfiler *profiler, QQmlBoundSignalExpression *expression) :
        QQmlProfilerHelper(profiler)
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileHandlingSignal, profiler,
                      startHandlingSignal(expression->sourceLocation()));
    }

    ~QQmlHandlingSignalProfiler()
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileHandlingSignal, profiler,
                      endRange<QQmlProfiler::HandlingSignal>());
    }
};

struct QQmlCompilingProfiler : public QQmlProfilerHelper {
    QQmlCompilingProfiler(QQmlProfiler *profiler, const QString &name) :
        QQmlProfilerHelper(profiler)
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileCompiling, profiler, startCompiling(name));
    }

    ~QQmlCompilingProfiler()
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileCompiling, profiler, endRange<Compiling>());
    }
};

struct QQmlVmeProfiler : public QQmlProfilerDefinitions {
public:

    struct Data {
        Data() : m_line(0), m_column(0) {}
        QUrl m_url;
        int m_line;
        int m_column;
        QString m_typeName;
    };

    QQmlVmeProfiler() : profiler(0) {}

    void init(QQmlProfiler *p, int maxDepth)
    {
        profiler = p;
        ranges.allocate(maxDepth);
    }

    Data pop()
    {
        if (ranges.count() > 0)
            return ranges.pop();
        else
            return Data();
    }

    void push(const Data &data)
    {
        if (ranges.capacity() > ranges.count())
            ranges.push(data);
    }

    QQmlProfiler *profiler;

private:
    QFiniteStack<Data> ranges;
};

#define Q_QML_OC_PROFILE(member, Code)\
    Q_QML_PROFILE_IF_ENABLED(QQmlProfilerDefinitions::ProfileCreating, member.profiler, Code)

class QQmlObjectCreationProfiler : public QQmlVmeProfiler::Data {
public:

    QQmlObjectCreationProfiler(QQmlProfiler *profiler) : profiler(profiler)
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileCreating, profiler, startCreating());
    }

    ~QQmlObjectCreationProfiler()
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileCreating, profiler, endRange<QQmlProfilerDefinitions::Creating>());
    }

    void update(const QString &typeName, const QUrl &url, int line, int column)
    {
        profiler->updateCreating(typeName, url, line, column);
        m_typeName = typeName;
        m_url = url;
        m_line = line;
        m_column = column;
    }

private:
    QQmlProfiler *profiler;
};

class QQmlObjectCompletionProfiler {
public:
    QQmlObjectCompletionProfiler(QQmlVmeProfiler *parent) :
        profiler(parent->profiler)
    {
        Q_QML_PROFILE_IF_ENABLED(QQmlProfilerDefinitions::ProfileCreating, profiler, {
            QQmlVmeProfiler::Data data = parent->pop();
            profiler->startCreating(data.m_typeName, data.m_url, data.m_line, data.m_column);
        });
    }

    ~QQmlObjectCompletionProfiler()
    {
        Q_QML_PROFILE(QQmlProfilerDefinitions::ProfileCreating, profiler,
                      endRange<QQmlProfilerDefinitions::Creating>());
    }
private:
    QQmlProfiler *profiler;
};

QT_END_NAMESPACE
Q_DECLARE_METATYPE(QList<QQmlProfilerData>)

#endif // QQMLPROFILER_P_H
