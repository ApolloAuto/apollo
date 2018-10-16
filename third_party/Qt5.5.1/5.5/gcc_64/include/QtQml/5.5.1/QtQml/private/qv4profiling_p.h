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

#ifndef QV4PROFILING_H
#define QV4PROFILING_H

#include "qv4global_p.h"
#include "qv4engine_p.h"
#include "qv4function_p.h"

#include <QElapsedTimer>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Profiling {

enum Features {
    FeatureFunctionCall,
    FeatureMemoryAllocation
};

enum MemoryType {
    HeapPage,
    LargeItem,
    SmallItem
};

struct FunctionCallProperties {
    qint64 start;
    qint64 end;
    QString name;
    QString file;
    int line;
    int column;
};

struct MemoryAllocationProperties {
    qint64 timestamp;
    qint64 size;
    MemoryType type;
};

class FunctionCall {
public:

    FunctionCall() : m_function(0), m_start(0), m_end(0)
    { Q_ASSERT_X(false, Q_FUNC_INFO, "Cannot construct a function call without function"); }

    FunctionCall(Function *function, qint64 start, qint64 end) :
        m_function(function), m_start(start), m_end(end)
    { m_function->compilationUnit->addref(); }

    FunctionCall(const FunctionCall &other) :
        m_function(other.m_function), m_start(other.m_start), m_end(other.m_end)
    { m_function->compilationUnit->addref(); }

    ~FunctionCall()
    { m_function->compilationUnit->release(); }

    FunctionCall &operator=(const FunctionCall &other) {
        if (&other != this) {
            if (m_function)
                m_function->compilationUnit->release();
            m_function = other.m_function;
            m_start = other.m_start;
            m_end = other.m_end;
            m_function->compilationUnit->addref();
        }
        return *this;
    }

    FunctionCallProperties resolve() const;

private:

    Function *m_function;
    qint64 m_start;
    qint64 m_end;
};

#define Q_V4_PROFILE_ALLOC(engine, size, type)\
    (engine->profiler &&\
            (engine->profiler->featuresEnabled & (1 << Profiling::FeatureMemoryAllocation)) ?\
        engine->profiler->trackAlloc(size, type) : size)

#define Q_V4_PROFILE_DEALLOC(engine, pointer, size, type) \
    (engine->profiler &&\
            (engine->profiler->featuresEnabled & (1 << Profiling::FeatureMemoryAllocation)) ?\
        engine->profiler->trackDealloc(pointer, size, type) : pointer)

#define Q_V4_PROFILE(engine, function)\
    (engine->profiler &&\
            (engine->profiler->featuresEnabled & (1 << Profiling::FeatureFunctionCall)) ?\
        Profiling::FunctionCallProfiler::profileCall(engine->profiler, engine, function) :\
        function->code(engine, function->codeData))

class Q_QML_EXPORT Profiler : public QObject {
    Q_OBJECT
    Q_DISABLE_COPY(Profiler)
public:
    Profiler(QV4::ExecutionEngine *engine);

    size_t trackAlloc(size_t size, MemoryType type)
    {
        MemoryAllocationProperties allocation = {m_timer.nsecsElapsed(), (qint64)size, type};
        m_memory_data.append(allocation);
        return size;
    }

    void *trackDealloc(void *pointer, size_t size, MemoryType type)
    {
        MemoryAllocationProperties allocation = {m_timer.nsecsElapsed(), -(qint64)size, type};
        m_memory_data.append(allocation);
        return pointer;
    }

    quint64 featuresEnabled;

public slots:
    void stopProfiling();
    void startProfiling(quint64 features);
    void reportData();
    void setTimer(const QElapsedTimer &timer) { m_timer = timer; }

signals:
    void dataReady(const QList<QV4::Profiling::FunctionCallProperties> &,
                   const QList<QV4::Profiling::MemoryAllocationProperties> &);

private:
    QV4::ExecutionEngine *m_engine;
    QElapsedTimer m_timer;
    QVector<FunctionCall> m_data;
    QList<MemoryAllocationProperties> m_memory_data;

    friend class FunctionCallProfiler;
};

class FunctionCallProfiler {
    Q_DISABLE_COPY(FunctionCallProfiler)
public:

    // It's enough to ref() the function in the destructor as it will probably not disappear while
    // it's executing ...
    FunctionCallProfiler(Profiler *profiler, Function *function) :
        profiler(profiler), function(function), startTime(profiler->m_timer.nsecsElapsed())
    {}

    ~FunctionCallProfiler()
    {
        profiler->m_data.append(FunctionCall(function, startTime, profiler->m_timer.nsecsElapsed()));
    }

    static ReturnedValue profileCall(Profiler *profiler, ExecutionEngine *engine, Function *function)
    {
        FunctionCallProfiler callProfiler(profiler, function);
        return function->code(engine, function->codeData);
    }

    Profiler *profiler;
    Function *function;
    qint64 startTime;
};


} // namespace Profiling
} // namespace QV4

Q_DECLARE_TYPEINFO(QV4::Profiling::MemoryAllocationProperties, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(QV4::Profiling::FunctionCallProperties, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(QV4::Profiling::FunctionCall, Q_MOVABLE_TYPE);

QT_END_NAMESPACE
Q_DECLARE_METATYPE(QList<QV4::Profiling::FunctionCallProperties>)
Q_DECLARE_METATYPE(QList<QV4::Profiling::MemoryAllocationProperties>)

#endif // QV4PROFILING_H
