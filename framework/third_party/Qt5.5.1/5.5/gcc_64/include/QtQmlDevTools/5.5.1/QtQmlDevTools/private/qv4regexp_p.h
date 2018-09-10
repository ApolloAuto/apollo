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
#ifndef QV4REGEXP_H
#define QV4REGEXP_H

#include <QString>
#include <QVector>

#include <wtf/RefPtr.h>
#include <wtf/FastAllocBase.h>
#include <wtf/BumpPointerAllocator.h>

#include <limits.h>

#include <yarr/Yarr.h>
#include <yarr/YarrInterpreter.h>
#include <yarr/YarrJIT.h>

#include "qv4managed_p.h"
#include "qv4engine_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct ExecutionEngine;
struct RegExpCacheKey;

namespace Heap {

struct RegExp : Base {
    RegExp(ExecutionEngine* engine, const QString& pattern, bool ignoreCase, bool multiline);
    ~RegExp();
    QString pattern;
    OwnPtr<JSC::Yarr::BytecodePattern> byteCode;
#if ENABLE(YARR_JIT)
    JSC::Yarr::YarrCodeBlock jitCode;
#endif
    RegExpCache *cache;
    int subPatternCount;
    bool ignoreCase;
    bool multiLine;

    int captureCount() const { return subPatternCount + 1; }
};

}

struct RegExp : public Managed
{
    V4_MANAGED(RegExp, Managed)
    Q_MANAGED_TYPE(RegExp)
    V4_NEEDS_DESTROY

    QString pattern() const { return d()->pattern; }
    OwnPtr<JSC::Yarr::BytecodePattern> &byteCode() { return d()->byteCode; }
#if ENABLE(YARR_JIT)
    JSC::Yarr::YarrCodeBlock jitCode() const { return d()->jitCode; }
#endif
    RegExpCache *cache() const { return d()->cache; }
    int subPatternCount() const { return d()->subPatternCount; }
    bool ignoreCase() const { return d()->ignoreCase; }
    bool multiLine() const { return d()->multiLine; }

    static Heap::RegExp *create(ExecutionEngine* engine, const QString& pattern, bool ignoreCase = false, bool multiline = false);

    bool isValid() const { return d()->byteCode.get(); }

    uint match(const QString& string, int start, uint *matchOffsets);

    int captureCount() const { return subPatternCount() + 1; }

    static void markObjects(Heap::Base *that, QV4::ExecutionEngine *e);

    friend class RegExpCache;
};

struct RegExpCacheKey
{
    RegExpCacheKey(const QString &pattern, bool ignoreCase, bool multiLine)
        : pattern(pattern)
        , ignoreCase(ignoreCase)
        , multiLine(multiLine)
    { }
    explicit inline RegExpCacheKey(const RegExp::Data *re);

    bool operator==(const RegExpCacheKey &other) const
    { return pattern == other.pattern && ignoreCase == other.ignoreCase && multiLine == other.multiLine; }
    bool operator!=(const RegExpCacheKey &other) const
    { return !operator==(other); }

    QString pattern;
    uint ignoreCase : 1;
    uint multiLine : 1;
};

inline RegExpCacheKey::RegExpCacheKey(const RegExp::Data *re)
    : pattern(re->pattern)
    , ignoreCase(re->ignoreCase)
    , multiLine(re->multiLine)
{}

inline uint qHash(const RegExpCacheKey& key, uint seed = 0) Q_DECL_NOTHROW
{ return qHash(key.pattern, seed); }

// ### GC
class RegExpCache : public QHash<RegExpCacheKey, Heap::RegExp*>
{
public:
    ~RegExpCache();
};



}

QT_END_NAMESPACE

#endif // QV4REGEXP_H
