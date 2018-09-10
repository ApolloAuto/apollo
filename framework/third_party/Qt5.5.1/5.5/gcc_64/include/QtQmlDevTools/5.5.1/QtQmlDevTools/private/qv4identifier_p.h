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
#ifndef QV4IDENTIFIER_H
#define QV4IDENTIFIER_H

#include <qstring.h>

QT_BEGIN_NAMESPACE

namespace QV4 {

namespace Heap {
    struct String;
}

struct String;
struct IdentifierTable;
struct ExecutionEngine;

struct Identifier
{
    QString string;
    uint hashValue;
};


struct IdentifierHashEntry {
    const Identifier *identifier;
    union {
        int value;
        void *pointer;
    };
    static int get(const IdentifierHashEntry *This, int *) { return This ? This->value : -1; }
    static bool get(const IdentifierHashEntry *This, bool *) { return This != 0; }
    static void *get(const IdentifierHashEntry *This, void **) { return This ? This->pointer : 0; }
};

struct IdentifierHashData
{
    IdentifierHashData(int numBits);
    ~IdentifierHashData() {
        free(entries);
    }

    QBasicAtomicInt refCount;
    int alloc;
    int size;
    int numBits;
    IdentifierTable *identifierTable;
    IdentifierHashEntry *entries;
};

struct IdentifierHashBase
{

    IdentifierHashData *d;

    IdentifierHashBase() : d(0) {}
    IdentifierHashBase(ExecutionEngine *engine);
    inline IdentifierHashBase(const IdentifierHashBase &other);
    inline ~IdentifierHashBase();
    inline IdentifierHashBase &operator=(const IdentifierHashBase &other);

    bool isEmpty() const { return !d; }
    // ###
    void reserve(int) {}

    inline int count() const;
    bool contains(const Identifier *i) const;
    bool contains(const QString &str) const;
    bool contains(String *str) const;

protected:
    IdentifierHashEntry *addEntry(const Identifier *i);
    const IdentifierHashEntry *lookup(const Identifier *identifier) const;
    const IdentifierHashEntry *lookup(const QString &str) const;
    const IdentifierHashEntry *lookup(String *str) const;
    const Identifier *toIdentifier(const QString &str) const;
    const Identifier *toIdentifier(Heap::String *str) const;
};


template<typename T>
struct IdentifierHash : public IdentifierHashBase
{
    IdentifierHash()
        : IdentifierHashBase() {}
    IdentifierHash(ExecutionEngine *engine)
        : IdentifierHashBase(engine) {}
    inline IdentifierHash(const IdentifierHash<T> &other)
        : IdentifierHashBase(other) {}
    inline ~IdentifierHash() {}
    inline IdentifierHash &operator=(const IdentifierHash<T> &other) {
        IdentifierHashBase::operator =(other);
        return *this;
    }

    void add(const QString &str, const T &value);

    inline T value(const QString &str) const;
    inline T value(String *str) const;
    QString findId(T value) const;
};

inline IdentifierHashBase::IdentifierHashBase(const IdentifierHashBase &other)
{
    d = other.d;
    if (d)
        d->refCount.ref();
}

inline IdentifierHashBase::~IdentifierHashBase()
{
    if (d && !d->refCount.deref())
        delete d;
}

IdentifierHashBase &IdentifierHashBase::operator=(const IdentifierHashBase &other)
{
    if (other.d)
        other.d->refCount.ref();
    if (d && !d->refCount.deref())
        delete d;
    d = other.d;
    return *this;
}

inline int IdentifierHashBase::count() const
{
    return d ? d->size : 0;
}

inline bool IdentifierHashBase::contains(const Identifier *i) const
{
    return lookup(i) != 0;
}

inline bool IdentifierHashBase::contains(const QString &str) const
{
    return lookup(str) != 0;
}

inline bool IdentifierHashBase::contains(String *str) const
{
    return lookup(str) != 0;
}

template<typename T>
void IdentifierHash<T>::add(const QString &str, const T &value)
{
    IdentifierHashEntry *e = addEntry(toIdentifier(str));
    e->value = value;
}

template<typename T>
inline T IdentifierHash<T>::value(const QString &str) const
{
    return IdentifierHashEntry::get(lookup(str), (T*)0);
}

template<typename T>
inline T IdentifierHash<T>::value(String *str) const
{
    return IdentifierHashEntry::get(lookup(str), (T*)0);
}


template<typename T>
QString IdentifierHash<T>::findId(T value) const
{
    IdentifierHashEntry *e = d->entries;
    IdentifierHashEntry *end = e + d->alloc;
    while (e < end) {
        if (e->identifier && IdentifierHashEntry::get(e, (T*)0) == value)
            return e->identifier->string;
        ++e;
    }
    return QString();
}


}

QT_END_NAMESPACE

#endif
