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

#ifndef QV4EXECUTABLEALLOCATOR_H
#define QV4EXECUTABLEALLOCATOR_H

#include "qv4global_p.h"

#include <QMultiMap>
#include <QHash>
#include <QVector>
#include <QByteArray>
#include <QMutex>

namespace WTF {
class PageAllocation;
}

QT_BEGIN_NAMESPACE

namespace QV4 {

class Q_AUTOTEST_EXPORT ExecutableAllocator
{
public:
    struct ChunkOfPages;
    struct Allocation;

    ExecutableAllocator();
    ~ExecutableAllocator();

    Allocation *allocate(size_t size);
    void free(Allocation *allocation);

    struct Allocation
    {
        Allocation()
            : addr(0)
            , size(0)
            , free(true)
            , next(0)
            , prev(0)
        {}

        void *start() const;
        void invalidate() { addr = 0; }
        bool isValid() const { return addr != 0; }
        void deallocate(ExecutableAllocator *allocator);

    private:
        ~Allocation() {}

        friend class ExecutableAllocator;

        Allocation *split(size_t dividingSize);
        bool mergeNext(ExecutableAllocator *allocator);
        bool mergePrevious(ExecutableAllocator *allocator);

        quintptr addr;
        uint size : 31; // More than 2GB of function code? nah :)
        uint free : 1;
        Allocation *next;
        Allocation *prev;
    };

    // for debugging / unit-testing
    int freeAllocationCount() const { return freeAllocations.count(); }
    int chunkCount() const { return chunks.count(); }

    struct ChunkOfPages
    {
        ChunkOfPages()
            : pages(0)
            , firstAllocation(0)
        {}
        ~ChunkOfPages();

        WTF::PageAllocation *pages;
        Allocation *firstAllocation;

        bool contains(Allocation *alloc) const;
    };

    ChunkOfPages *chunkForAllocation(Allocation *allocation) const;

private:
    QMultiMap<size_t, Allocation*> freeAllocations;
    QMap<quintptr, ChunkOfPages*> chunks;
    mutable QMutex mutex;
};

}

QT_END_NAMESPACE

#endif // QV4EXECUTABLEALLOCATOR_H
