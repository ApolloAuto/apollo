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

#ifndef QV4GC_H
#define QV4GC_H

#include "qv4global_p.h"
#include "qv4value_inl_p.h"
#include "qv4scopedvalue_p.h"

//#define DETAILED_MM_STATS

QT_BEGIN_NAMESPACE

namespace QV4 {

struct GCDeletable;

class Q_QML_EXPORT MemoryManager
{
    Q_DISABLE_COPY(MemoryManager);

public:
    struct Data;

    class GCBlocker
    {
    public:
        GCBlocker(MemoryManager *mm)
            : mm(mm)
            , wasBlocked(mm->isGCBlocked())
        {
            mm->setGCBlocked(true);
        }

        ~GCBlocker()
        {
            mm->setGCBlocked(wasBlocked);
        }

    private:
        MemoryManager *mm;
        bool wasBlocked;
    };

public:
    MemoryManager(ExecutionEngine *engine);
    ~MemoryManager();

    // TODO: this is only for 64bit (and x86 with SSE/AVX), so exend it for other architectures to be slightly more efficient (meaning, align on 8-byte boundaries).
    // Note: all occurrences of "16" in alloc/dealloc are also due to the alignment.
    static inline std::size_t align(std::size_t size)
    { return (size + 15) & ~0xf; }

    template<typename ManagedType>
    inline typename ManagedType::Data *allocManaged(std::size_t size, std::size_t unmanagedSize = 0)
    {
        size = align(size);
        Heap::Base *o = allocData(size, unmanagedSize);
        o->vtable = ManagedType::staticVTable();
        return static_cast<typename ManagedType::Data *>(o);
    }

    template <typename ManagedType>
    typename ManagedType::Data *alloc()
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data();
        return t->d();
    }

    template <typename ManagedType, typename Arg1>
    typename ManagedType::Data *alloc(Arg1 arg1)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data(arg1);
        return t->d();
    }

    template <typename ManagedType, typename Arg1>
    typename ManagedType::Data *allocWithStringData(std::size_t unmanagedSize, Arg1 arg1)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data), unmanagedSize));
        (void)new (t->d()) typename ManagedType::Data(this, arg1);
        return t->d();
    }

    template <typename ManagedType, typename Arg1, typename Arg2>
    typename ManagedType::Data *alloc(Arg1 arg1, Arg2 arg2)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data(arg1, arg2);
        return t->d();
    }

    template <typename ManagedType, typename Arg1, typename Arg2, typename Arg3>
    typename ManagedType::Data *alloc(Arg1 arg1, Arg2 arg2, Arg3 arg3)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data(arg1, arg2, arg3);
        return t->d();
    }

    template <typename ManagedType, typename Arg1, typename Arg2, typename Arg3, typename Arg4>
    typename ManagedType::Data *alloc(Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data(arg1, arg2, arg3, arg4);
        return t->d();
    }

    template <typename ManagedType, typename Arg1, typename Arg2, typename Arg3, typename Arg4, typename Arg5>
    typename ManagedType::Data *alloc(Arg1 arg1, Arg2 arg2, Arg3 arg3, Arg4 arg4, Arg5 arg5)
    {
        Scope scope(engine());
        Scoped<ManagedType> t(scope, allocManaged<ManagedType>(sizeof(typename ManagedType::Data)));
        (void)new (t->d()) typename ManagedType::Data(arg1, arg2, arg3, arg4, arg5);
        return t->d();
    }

    bool isGCBlocked() const;
    void setGCBlocked(bool blockGC);
    void runGC();

    ExecutionEngine *engine() const;

    void dumpStats() const;

    void registerDeletable(GCDeletable *d);

    size_t getUsedMem() const;
    size_t getAllocatedMem() const;
    size_t getLargeItemsMem() const;

    void growUnmanagedHeapSizeUsage(size_t delta); // called when a JS object grows itself. Specifically: Heap::String::append

protected:
    /// expects size to be aligned
    // TODO: try to inline
    Heap::Base *allocData(std::size_t size, std::size_t unmanagedSize);

#ifdef DETAILED_MM_STATS
    void willAllocate(std::size_t size);
#endif // DETAILED_MM_STATS

private:
    void collectFromJSStack() const;
    void mark();
    void sweep(bool lastSweep = false);

protected:
    QScopedPointer<Data> m_d;
public:
    PersistentValueStorage *m_persistentValues;
    PersistentValueStorage *m_weakValues;
};

}

QT_END_NAMESPACE

#endif // QV4GC_H
