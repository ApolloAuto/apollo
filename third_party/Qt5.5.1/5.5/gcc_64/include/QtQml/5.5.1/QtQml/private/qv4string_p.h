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
#ifndef QV4STRING_H
#define QV4STRING_H

#include <QtCore/qstring.h>
#include "qv4managed_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {

struct ExecutionEngine;
struct Identifier;

namespace Heap {

#ifndef V4_BOOTSTRAP
struct Q_QML_PRIVATE_EXPORT String : Base {
    enum StringType {
        StringType_Unknown,
        StringType_Regular,
        StringType_ArrayIndex
    };

    String(MemoryManager *mm, const QString &text);
    String(MemoryManager *mm, String *l, String *n);
    ~String() {
        if (!largestSubLength && !text->ref.deref())
            QStringData::deallocate(text);
    }
    void simplifyString() const;
    int length() const {
        Q_ASSERT((largestSubLength &&
                  (len == left->len + right->len)) ||
                 len == (uint)text->size);
        return len;
    }
    std::size_t retainedTextSize() const {
        return largestSubLength ? 0 : (std::size_t(text->size) * sizeof(QChar));
    }
    void createHashValue() const;
    inline unsigned hashValue() const {
        if (subtype == StringType_Unknown)
            createHashValue();
        Q_ASSERT(!largestSubLength);

        return stringHash;
    }
    inline QString toQString() const {
        if (largestSubLength)
            simplifyString();
        QStringDataPtr ptr = { text };
        text->ref.ref();
        return QString(ptr);
    }
    inline bool isEqualTo(const String *other) const {
        if (this == other)
            return true;
        if (hashValue() != other->hashValue())
            return false;
        Q_ASSERT(!largestSubLength);
        if (identifier && identifier == other->identifier)
            return true;
        if (subtype == Heap::String::StringType_ArrayIndex && other->subtype == Heap::String::StringType_ArrayIndex)
            return true;

        return toQString() == other->toQString();
    }

    union {
        mutable QStringData *text;
        mutable String *left;
    };
    union {
        mutable Identifier *identifier;
        mutable String *right;
    };
    mutable uint subtype;
    mutable uint stringHash;
    mutable uint largestSubLength;
    uint len;
    MemoryManager *mm;
private:
    static void append(const String *data, QChar *ch);
};
#endif

}

struct Q_QML_PRIVATE_EXPORT String : public Managed {
#ifndef V4_BOOTSTRAP
    V4_MANAGED(String, Managed)
    Q_MANAGED_TYPE(String)
    V4_NEEDS_DESTROY
    enum {
        IsString = true
    };

    uchar subtype() const { return d()->subtype; }
    void setSubtype(uchar subtype) const { d()->subtype = subtype; }

    bool equals(String *other) const {
        return d()->isEqualTo(other->d());
    }
    inline bool isEqualTo(const String *other) const {
        return d()->isEqualTo(other->d());
    }

    inline bool compare(const String *other) {
        return toQString() < other->toQString();
    }

    inline QString toQString() const {
        return d()->toQString();
    }

    inline unsigned hashValue() const {
        return d()->hashValue();
    }
    uint asArrayIndex() const {
        if (subtype() == Heap::String::StringType_Unknown)
            d()->createHashValue();
        Q_ASSERT(!d()->largestSubLength);
        if (subtype() == Heap::String::StringType_ArrayIndex)
            return d()->stringHash;
        return UINT_MAX;
    }
    uint toUInt(bool *ok) const;

    void makeIdentifier(ExecutionEngine *e) const {
        if (d()->identifier)
            return;
        makeIdentifierImpl(e);
    }

    void makeIdentifierImpl(ExecutionEngine *e) const;

    static uint createHashValue(const QChar *ch, int length);
    static uint createHashValue(const char *ch, int length);

    bool startsWithUpper() const {
        const String::Data *l = d();
        while (l->largestSubLength)
            l = l->left;
        return l->text->size && QChar::isUpper(l->text->data()[0]);
    }

    Identifier *identifier() const { return d()->identifier; }

protected:
    static void markObjects(Heap::Base *that, ExecutionEngine *e);
    static bool isEqualTo(Managed *that, Managed *o);
    static uint getLength(const Managed *m);
#endif

public:
    static uint toArrayIndex(const QString &str);
};

}

QT_END_NAMESPACE

#endif
