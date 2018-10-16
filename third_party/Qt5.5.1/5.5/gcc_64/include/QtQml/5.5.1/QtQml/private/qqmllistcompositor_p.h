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

#ifndef QQMLLISTCOMPOSITOR_P_H
#define QQMLLISTCOMPOSITOR_P_H

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

#include <QtCore/qglobal.h>
#include <QtCore/qvector.h>

#include <private/qqmlchangeset_p.h>

#include <QtCore/qdebug.h>

QT_BEGIN_NAMESPACE

class Q_AUTOTEST_EXPORT QQmlListCompositor
{
public:
    enum { MinimumGroupCount = 3, MaximumGroupCount = 11 };

    enum Group
    {
        Cache   = 0,
        Default = 1,
        Persisted = 2
    };

    enum Flag
    {
        CacheFlag       = 1 << Cache,
        DefaultFlag     = 1 << Default,
        PersistedFlag   = 1 << Persisted,
        PrependFlag     = 0x10000000,
        AppendFlag      = 0x20000000,
        UnresolvedFlag  = 0x40000000,
        MovedFlag       = 0x80000000,
        GroupMask       = ~(PrependFlag | AppendFlag | UnresolvedFlag | MovedFlag | CacheFlag)
    };

    class Range
    {
    public:
        Range() : next(this), previous(this), list(0), index(0), count(0), flags(0) {}
        Range(Range *next, void *list, int index, int count, uint flags)
            : next(next), previous(next->previous), list(list), index(index), count(count), flags(flags) {
            next->previous = this; previous->next = this; }

        Range *next;
        Range *previous;
        void *list;
        int index;
        int count;
        uint flags;

        inline int start() const { return index; }
        inline int end() const { return index + count; }

        inline int groups() const { return flags & GroupMask; }

        inline bool inGroup() const { return flags & GroupMask; }
        inline bool inCache() const { return flags & CacheFlag; }
        inline bool inGroup(int group) const { return flags & (1 << group); }
        inline bool isUnresolved() const { return flags & UnresolvedFlag; }

        inline bool prepend() const { return flags & PrependFlag; }
        inline bool append() const { return flags & AppendFlag; }
    };

    class Q_AUTOTEST_EXPORT iterator
    {
    public:
        inline iterator();
        inline iterator(const iterator &it);
        inline iterator(Range *range, int offset, Group group, int groupCount);
        inline ~iterator() {}

        bool operator ==(const iterator &it) const { return range == it.range && offset == it.offset; }
        bool operator !=(const iterator &it) const { return range != it.range || offset != it.offset; }

        bool operator ==(Group group) const { return range->flags & (1 << group); }
        bool operator !=(Group group) const { return !(range->flags & (1 << group)); }

        Range *&operator *() { return range; }
        Range * const &operator *() const { return range; }
        Range *operator ->() { return range; }
        const Range *operator ->() const { return range; }

        iterator &operator +=(int difference);

        template<typename T> T *list() const { return static_cast<T *>(range->list); }
        int modelIndex() const { return range->index + offset; }

        void incrementIndexes(int difference) { incrementIndexes(difference, range->flags); }
        void decrementIndexes(int difference) { decrementIndexes(difference, range->flags); }

        inline void incrementIndexes(int difference, uint flags);
        inline void decrementIndexes(int difference, uint flags);

        void setGroup(Group g) { group = g; groupFlag = 1 << g; }

        Range *range;
        int offset;
        Group group;
        int groupFlag;
        int groupCount;
        union {
            struct {
                int cacheIndex;
            };
            int index[MaximumGroupCount];
        };
    };

    class Q_AUTOTEST_EXPORT insert_iterator : public iterator
    {
    public:
        inline insert_iterator() {}
        inline insert_iterator(const iterator &it) : iterator(it) {}
        inline insert_iterator(Range *, int, Group, int);
        inline ~insert_iterator() {}

        insert_iterator &operator +=(int difference);
    };

    struct Change
    {
        inline Change() {}
        inline Change(iterator it, int count, uint flags, int moveId = -1);
        int count;
        uint flags;
        int moveId;
        union {
            struct {
                int cacheIndex;
            };
            int index[MaximumGroupCount];
        };

        inline bool isMove() const { return moveId >= 0; }
        inline bool inCache() const { return flags & CacheFlag; }
        inline bool inGroup() const { return flags & GroupMask; }
        inline bool inGroup(int group) const { return flags & (CacheFlag << group); }

        inline int groups() const { return flags & GroupMask; }
    };

    struct Insert : public Change
    {
        Insert() {}
        Insert(iterator it, int count, uint flags, int moveId = -1)
            : Change(it, count, flags, moveId) {}
    };

    struct Remove : public Change
    {
        Remove() {}
        Remove(iterator it, int count, uint flags, int moveId = -1)
            : Change(it, count, flags, moveId) {}
    };

    QQmlListCompositor();
    ~QQmlListCompositor();

    int defaultGroups() const { return m_defaultFlags & ~PrependFlag; }
    void setDefaultGroups(int groups) { m_defaultFlags = groups | PrependFlag; }
    void setDefaultGroup(Group group) { m_defaultFlags |= (1 << group); }
    void clearDefaultGroup(Group group) { m_defaultFlags &= ~(1 << group); }
    void setRemoveGroups(int groups) { m_removeFlags = PrependFlag | AppendFlag | groups; }
    void setGroupCount(int count);

    int count(Group group) const;
    iterator find(Group group, int index);
    iterator find(Group group, int index) const;
    insert_iterator findInsertPosition(Group group, int index);

    const iterator &end() { return m_end; }

    void append(void *list, int index, int count, uint flags, QVector<Insert> *inserts = 0);
    void insert(Group group, int before, void *list, int index, int count, uint flags, QVector<Insert> *inserts = 0);
    iterator insert(iterator before, void *list, int index, int count, uint flags, QVector<Insert> *inserts = 0);

    void setFlags(Group fromGroup, int from, int count, Group group, int flags, QVector<Insert> *inserts = 0);
    void setFlags(iterator from, int count, Group group, uint flags, QVector<Insert> *inserts = 0);
    void setFlags(Group fromGroup, int from, int count, uint flags, QVector<Insert> *inserts = 0) {
        setFlags(fromGroup, from, count, fromGroup, flags, inserts); }
    void setFlags(iterator from, int count, uint flags, QVector<Insert> *inserts = 0) {
        setFlags(from, count, from.group, flags, inserts); }

    void clearFlags(Group fromGroup, int from, int count, Group group, uint flags, QVector<Remove> *removals = 0);
    void clearFlags(iterator from, int count, Group group, uint flags, QVector<Remove> *removals = 0);
    void clearFlags(Group fromGroup, int from, int count, uint flags, QVector<Remove> *removals = 0) {
        clearFlags(fromGroup, from, count, fromGroup, flags, removals); }
    void clearFlags(iterator from, int count, uint flags, QVector<Remove> *removals = 0) {
        clearFlags(from, count, from.group, flags, removals); }

    bool verifyMoveTo(Group fromGroup, int from, Group toGroup, int to, int count, Group group) const;

    void move(
            Group fromGroup,
            int from,
            Group toGroup,
            int to,
            int count,
            Group group,
            QVector<Remove> *removals = 0,
            QVector<Insert> *inserts = 0);
    void clear();

    void listItemsInserted(void *list, int index, int count, QVector<Insert> *inserts);
    void listItemsRemoved(void *list, int index, int count, QVector<Remove> *removals);
    void listItemsMoved(void *list, int from, int to, int count, QVector<Remove> *removals, QVector<Insert> *inserts);
    void listItemsChanged(void *list, int index, int count, QVector<Change> *changes);

    void transition(
            Group from,
            Group to,
            QVector<QQmlChangeSet::Change> *removes,
            QVector<QQmlChangeSet::Change> *inserts);

private:
    Range m_ranges;
    iterator m_end;
    iterator m_cacheIt;
    int m_groupCount;
    int m_defaultFlags;
    int m_removeFlags;
    int m_moveId;

    inline Range *insert(Range *before, void *list, int index, int count, uint flags);
    inline Range *erase(Range *range);

    struct MovedFlags
    {
        MovedFlags() {}
        MovedFlags(int moveId, uint flags) : moveId(moveId), flags(flags) {}

        int moveId;
        uint flags;
    };

    void listItemsRemoved(
            QVector<Remove> *translatedRemovals,
            void *list,
            QVector<QQmlChangeSet::Change> *removals,
            QVector<QQmlChangeSet::Change> *insertions = 0,
            QVector<MovedFlags> *movedFlags = 0);
    void listItemsInserted(
            QVector<Insert> *translatedInsertions,
            void *list,
            const QVector<QQmlChangeSet::Change> &insertions,
            const QVector<MovedFlags> *movedFlags = 0);
    void listItemsChanged(
            QVector<Change> *translatedChanges,
            void *list,
            const QVector<QQmlChangeSet::Change> &changes);

    friend Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor &list);
};

Q_DECLARE_TYPEINFO(QQmlListCompositor::Change, Q_PRIMITIVE_TYPE);
Q_DECLARE_TYPEINFO(QQmlListCompositor::Remove, Q_PRIMITIVE_TYPE);
Q_DECLARE_TYPEINFO(QQmlListCompositor::Insert, Q_PRIMITIVE_TYPE);

inline QQmlListCompositor::iterator::iterator()
    : range(0), offset(0), group(Default), groupCount(0) {}
inline QQmlListCompositor::iterator::iterator(const iterator &it)
    : range(it.range)
    , offset(it.offset)
    , group(it.group)
    , groupFlag(it.groupFlag)
    , groupCount(it.groupCount)
{
    for (int i = 0; i < groupCount; ++i)
        index[i] = it.index[i];
}

inline QQmlListCompositor::iterator::iterator(
        Range *range, int offset, Group group, int groupCount)
    : range(range)
    , offset(offset)
    , group(group)
    , groupFlag(1 << group)
    , groupCount(groupCount)
{
    for (int i = 0; i < groupCount; ++i)
        index[i] = 0;
}

inline void QQmlListCompositor::iterator::incrementIndexes(int difference, uint flags)
{
    for (int i = 0; i < groupCount; ++i) {
        if (flags & (1 << i))
            index[i] += difference;
    }
}

inline void QQmlListCompositor::iterator::decrementIndexes(int difference, uint flags)
{
    for (int i = 0; i < groupCount; ++i) {
        if (flags & (1 << i))
            index[i] -= difference;
    }
}

inline QQmlListCompositor::insert_iterator::insert_iterator(
        Range *range, int offset, Group group, int groupCount)
    : iterator(range, offset, group, groupCount) {}

inline QQmlListCompositor::Change::Change(iterator it, int count, uint flags, int moveId)
    : count(count), flags(flags), moveId(moveId)
{
    for (int i = 0; i < MaximumGroupCount; ++i)
        index[i] = it.index[i];
}

Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::Group &group);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::Range &range);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::iterator &it);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::Change &change);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::Remove &remove);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor::Insert &insert);
Q_AUTOTEST_EXPORT QDebug operator <<(QDebug debug, const QQmlListCompositor &list);

QT_END_NAMESPACE

#endif
