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

#ifndef QHASHEDSTRING_P_H
#define QHASHEDSTRING_P_H

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
#include <QtCore/qstring.h>
#include <private/qv4string_p.h>
#include <private/qv4scopedvalue_p.h>

#include <private/qflagpointer_p.h>

#if defined(Q_OS_QNX)
#include <stdlib.h>
#endif

QT_BEGIN_NAMESPACE

// Enable this to debug hash linking assumptions.
// #define QSTRINGHASH_LINK_DEBUG

class QHashedStringRef;
class Q_AUTOTEST_EXPORT QHashedString : public QString
{
public:
    inline QHashedString();
    inline QHashedString(const QString &string);
    inline QHashedString(const QString &string, quint32);
    inline QHashedString(const QHashedString &string);

    inline QHashedString &operator=(const QHashedString &string);
    inline bool operator==(const QHashedString &string) const;
    inline bool operator==(const QHashedStringRef &string) const;

    inline quint32 hash() const;
    inline quint32 existingHash() const;

    static bool compare(const QChar *lhs, const QChar *rhs, int length);
    static inline bool compare(const QChar *lhs, const char *rhs, int length);
    static inline bool compare(const char *lhs, const char *rhs, int length);
private:
    friend class QHashedStringRef;
    friend class QStringHashNode;

    void computeHash() const;
    mutable quint32 m_hash;
};

class QHashedCStringRef;
class Q_AUTOTEST_EXPORT QHashedStringRef
{
public:
    inline QHashedStringRef();
    inline QHashedStringRef(const QString &);
    inline QHashedStringRef(const QStringRef &);
    inline QHashedStringRef(const QChar *, int);
    inline QHashedStringRef(const QChar *, int, quint32);
    inline QHashedStringRef(const QHashedString &);
    inline QHashedStringRef(const QHashedStringRef &);
    inline QHashedStringRef &operator=(const QHashedStringRef &);

    inline bool operator==(const QString &string) const;
    inline bool operator==(const QHashedString &string) const;
    inline bool operator==(const QHashedStringRef &string) const;
    inline bool operator==(const QHashedCStringRef &string) const;
    inline bool operator!=(const QString &string) const;
    inline bool operator!=(const QHashedString &string) const;
    inline bool operator!=(const QHashedStringRef &string) const;
    inline bool operator!=(const QHashedCStringRef &string) const;

    inline quint32 hash() const;

    inline QChar *data();
    inline const QChar &at(int) const;
    inline const QChar *constData() const;
    bool startsWith(const QString &) const;
    bool endsWith(const QString &) const;
    int indexOf(const QChar &, int from=0) const;
    QHashedStringRef mid(int, int) const;

    inline bool isEmpty() const;
    inline int length() const;
    inline bool startsWithUpper() const;

    QString toString() const;

    inline bool isLatin1() const;

private:
    friend class QHashedString;

    void computeHash() const;

    const QChar *m_data;
    int m_length;
    mutable quint32 m_hash;
};

class Q_AUTOTEST_EXPORT QHashedCStringRef
{
public:
    inline QHashedCStringRef();
    inline QHashedCStringRef(const char *, int);
    inline QHashedCStringRef(const char *, int, quint32);
    inline QHashedCStringRef(const QHashedCStringRef &);

    inline quint32 hash() const;

    inline const char *constData() const;
    inline int length() const;

    QString toUtf16() const;
    inline int utf16length() const;
    inline void writeUtf16(QChar *) const;
    inline void writeUtf16(quint16 *) const;
private:
    friend class QHashedStringRef;

    void computeHash() const;

    const char *m_data;
    int m_length;
    mutable quint32 m_hash;
};

class QStringHashData;
class Q_AUTOTEST_EXPORT QStringHashNode
{
public:
    QStringHashNode()
    : length(0), hash(0), symbolId(0), ckey(0)
    {
    }

    QStringHashNode(const QHashedString &key)
    : length(key.length()), hash(key.hash()), symbolId(0)
    {
        strData = const_cast<QHashedString &>(key).data_ptr();
        setQString(true);
        strData->ref.ref();
    }

    QStringHashNode(const QHashedCStringRef &key)
    : length(key.length()), hash(key.hash()), symbolId(0), ckey(key.constData())
    {
    }

    QStringHashNode(const QStringHashNode &o)
    : length(o.length), hash(o.hash), symbolId(o.symbolId), ckey(o.ckey)
    {
        setQString(o.isQString());
        if (isQString()) { strData->ref.ref(); }
    }

    ~QStringHashNode()
    {
        if (isQString()) { if (!strData->ref.deref()) free(strData); }
    }

    QFlagPointer<QStringHashNode> next;

    qint32 length;
    quint32 hash;
    quint32 symbolId;

    union {
        const char *ckey;
        QStringData *strData;
    };

    inline QHashedString key() const
    {
        if (isQString())
            return QHashedString(QString((QChar *)strData->data(), length), hash);

        return QHashedString(QString::fromLatin1(ckey, length), hash);
    }

    bool isQString() const { return next.flag(); }
    void setQString(bool v) { if (v) next.setFlag(); else next.clearFlag(); }

    inline char *cStrData() const { return (char *)ckey; }
    inline quint16 *utf16Data() const { return (quint16 *)strData->data(); }

    inline bool equals(const QV4::Value &string) const {
        QString s = string.toQStringNoThrow();
        if (isQString()) {
            QStringDataPtr dd;
            dd.ptr = strData;
            strData->ref.ref();
            return QString(dd) == s;
        } else {
            return QLatin1String(cStrData(), length) == s;
        }
    }

    inline bool equals(const QV4::String *string) const {
        if (length != string->d()->length() || hash != string->hashValue())
                return false;
        if (isQString()) {
            QStringDataPtr dd;
            dd.ptr = strData;
            strData->ref.ref();
            return QString(dd) == string->toQString();
        } else {
            return QLatin1String(cStrData(), length) == string->toQString();
        }
    }

    inline bool equals(const QHashedStringRef &string) const {
        return length == string.length() &&
               hash == string.hash() &&
               (isQString()?QHashedString::compare(string.constData(), (const QChar *)utf16Data(), length):
                            QHashedString::compare(string.constData(), cStrData(), length));
    }

    inline bool equals(const QHashedCStringRef &string) const {
        return length == string.length() &&
               hash == string.hash() &&
               (isQString()?QHashedString::compare((const QChar *)utf16Data(), string.constData(), length):
                            QHashedString::compare(string.constData(), cStrData(), length));
    }
};

class Q_AUTOTEST_EXPORT QStringHashData
{
public:
    QStringHashData()
    : buckets(0), numBuckets(0), size(0), numBits(0)
#ifdef QSTRINGHASH_LINK_DEBUG
      , linkCount(0)
#endif
    {}

    QStringHashNode **buckets;
    int numBuckets;
    int size;
    short numBits;
#ifdef QSTRINGHASH_LINK_DEBUG
    int linkCount;
#endif

    struct IteratorData {
        IteratorData() : n(0), p(0) {}
        QStringHashNode *n;
        void *p;
    };
    void rehashToBits(short);
    void rehashToSize(int);
    void rehashNode(QStringHashNode **newBuckets, int nb, QStringHashNode *node);

private:
    QStringHashData(const QStringHashData &);
    QStringHashData &operator=(const QStringHashData &);
};

// For a supplied key type, in what form do we need to keep a hashed version?
template<typename T>
struct HashedForm {};

template<> struct HashedForm<QString> { typedef QHashedString Type; };
template<> struct HashedForm<QStringRef> { typedef QHashedStringRef Type; };
template<> struct HashedForm<QHashedString> { typedef const QHashedString &Type; };
template<> struct HashedForm<QV4::String *> { typedef const QV4::String *Type; };
template<> struct HashedForm<const QV4::String *> { typedef const QV4::String *Type; };
template<> struct HashedForm<QHashedStringRef> { typedef const QHashedStringRef &Type; };
template<> struct HashedForm<QLatin1String> { typedef QHashedCStringRef Type; };
template<> struct HashedForm<QHashedCStringRef> { typedef const QHashedCStringRef &Type; };

class QStringHashBase
{
public:
    static HashedForm<QString>::Type hashedString(const QString &s) { return QHashedString(s);}
    static HashedForm<QStringRef>::Type hashedString(const QStringRef &s) { return QHashedStringRef(s.constData(), s.size());}
    static HashedForm<QHashedString>::Type hashedString(const QHashedString &s) { return s; }
    static HashedForm<QV4::String *>::Type hashedString(QV4::String *s) { return s; }
    static HashedForm<const QV4::String *>::Type hashedString(const QV4::String *s) { return s; }
    static HashedForm<QHashedStringRef>::Type hashedString(const QHashedStringRef &s) { return s; }

    static HashedForm<QLatin1String>::Type hashedString(const QLatin1String &s) { return QHashedCStringRef(s.data(), s.size()); }
    static HashedForm<QHashedCStringRef>::Type hashedString(const QHashedCStringRef &s) { return s; }

    static const QString &toQString(const QString &s) { return s; }
    static const QString &toQString(const QHashedString &s) { return s; }
    static QString toQString(const QV4::String *s) { return s->toQString(); }
    static QString toQString(const QHashedStringRef &s) { return s.toString(); }

    static QString toQString(const QLatin1String &s) { return QString(s); }
    static QString toQString(const QHashedCStringRef &s) { return s.toUtf16(); }

    static inline quint32 hashOf(const QHashedStringRef &s) { return s.hash(); }
    static inline quint32 hashOf(QV4::String *s) { return s->hashValue(); }
    static inline quint32 hashOf(const QV4::String *s) { return s->hashValue(); }

    template<typename K>
    static inline quint32 hashOf(const K &key) { return hashedString(key).hash(); }
};

template<class T>
class QStringHash : public QStringHashBase
{
public:
    typedef QHashedString key_type;
    typedef T mapped_type;

    struct Node : public QStringHashNode {
        Node(const QHashedString &key, const T &value) : QStringHashNode(key), value(value) {}
        Node(const QHashedCStringRef &key, const T &value) : QStringHashNode(key), value(value) {}
        Node(const Node &o) : QStringHashNode(o), value(o.value) {}
        Node() {}
        T value;
    };
    struct NewedNode : public Node {
        NewedNode(const QHashedString &key, const T &value) : Node(key, value), nextNewed(0) {}
        NewedNode(const QHashedCStringRef &key, const T &value) : Node(key, value), nextNewed(0) {}
        NewedNode(const Node &o) : Node(o), nextNewed(0) {}
        NewedNode *nextNewed;
    };
    struct ReservedNodePool
    {
        ReservedNodePool() : count(0), used(0), nodes(0) {}
        ~ReservedNodePool() { delete [] nodes; }
        int count;
        int used;
        Node *nodes;
    };

    QStringHashData data;
    NewedNode *newedNodes;
    ReservedNodePool *nodePool;
    const QStringHash<T> *link;

    template<typename K>
    inline Node *findNode(const K &) const;

    inline Node *createNode(const Node &o);

    template<typename K>
    inline Node *createNode(const K &, const T &);

    inline Node *insertNode(Node *, quint32);

    inline void initializeNode(Node *, const QHashedString &key);
    inline void initializeNode(Node *, const QHashedCStringRef &key);

    template<typename K>
    inline Node *takeNode(const K &key, const T &value);

    inline Node *takeNode(const Node &o);

    inline void copy(const QStringHash<T> &);

    void copyNode(const QStringHashNode *otherNode);

    inline QStringHashData::IteratorData iterateFirst() const;
    static inline QStringHashData::IteratorData iterateNext(const QStringHashData::IteratorData &);

public:
    inline QStringHash();
    inline QStringHash(const QStringHash &);
    inline ~QStringHash();

    QStringHash &operator=(const QStringHash<T> &);

    void copyAndReserve(const QStringHash<T> &other, int additionalReserve);
    void linkAndReserve(const QStringHash<T> &other, int additionalReserve);

    inline bool isEmpty() const;
    inline void clear();
    inline int count() const;

    inline int numBuckets() const;
    inline bool isLinked() const;

    class ConstIterator {
    public:
        inline ConstIterator();
        inline ConstIterator(const QStringHashData::IteratorData &);

        inline ConstIterator &operator++();

        inline bool operator==(const ConstIterator &o) const;
        inline bool operator!=(const ConstIterator &o) const;

        template<typename K>
        inline bool equals(const K &) const;

        inline QHashedString key() const;
        inline const T &value() const;
        inline const T &operator*() const;

        inline Node *node() const;
    private:
        QStringHashData::IteratorData d;
    };

    template<typename K>
    inline void insert(const K &, const T &);

    inline void insert(const ConstIterator  &);

    template<typename K>
    inline T *value(const K &) const;

    inline T *value(const QV4::String *string) const;
    inline T *value(const ConstIterator &) const;

    template<typename K>
    inline bool contains(const K &) const;

    template<typename K>
    inline T &operator[](const K &);

    inline ConstIterator begin() const;
    inline ConstIterator end() const;

    inline ConstIterator iterator(Node *n) const;

    template<typename K>
    inline ConstIterator find(const K &) const;

    inline void reserve(int);
};

template<class T>
QStringHash<T>::QStringHash()
: newedNodes(0), nodePool(0), link(0)
{
}

template<class T>
QStringHash<T>::QStringHash(const QStringHash<T> &other)
: newedNodes(0), nodePool(0), link(0)
{
    data.numBits = other.data.numBits;
    data.size = other.data.size;
    reserve(other.count());
    copy(other);
}

template<class T>
QStringHash<T> &QStringHash<T>::operator=(const QStringHash<T> &other)
{
    if (&other == this)
        return *this;

    clear();

    data.numBits = other.data.numBits;
    data.size = other.data.size;
    reserve(other.count());
    copy(other);

    return *this;
}

template<class T>
void QStringHash<T>::copyAndReserve(const QStringHash<T> &other, int additionalReserve)
{
    clear();
    data.numBits = other.data.numBits;
    reserve(other.count() + additionalReserve);
    copy(other);
}

template<class T>
void QStringHash<T>::linkAndReserve(const QStringHash<T> &other, int additionalReserve)
{
    clear();

    if (other.count()) {
        data.size = other.data.size;
        data.rehashToSize(other.count() + additionalReserve);

        if (data.numBuckets == other.data.numBuckets) {
            nodePool = new ReservedNodePool;
            nodePool->count = additionalReserve;
            nodePool->used = 0;
            nodePool->nodes = new Node[additionalReserve];

#ifdef QSTRINGHASH_LINK_DEBUG
            data.linkCount++;
            const_cast<QStringHash<T>&>(other).data.linkCount++;
#endif

            for (int ii = 0; ii < data.numBuckets; ++ii)
                data.buckets[ii] = (Node *)other.data.buckets[ii];

            link = &other;
            return;
        }

        data.size = 0;
    }

    data.numBits = other.data.numBits;
    reserve(other.count() + additionalReserve);
    copy(other);
}

template<class T>
QStringHash<T>::~QStringHash()
{
    clear();
}

template<class T>
void QStringHash<T>::clear()
{
#ifdef QSTRINGHASH_LINK_DEBUG
    if (link) {
        data.linkCount--;
        const_cast<QStringHash<T> *>(link)->data.linkCount--;
    }

    if (data.linkCount)
        qFatal("QStringHash: Illegal attempt to clear a linked hash.");
#endif

    // Delete the individually allocated nodes
    NewedNode *n = newedNodes;
    while (n) {
        NewedNode *c = n;
        n = c->nextNewed;
        delete c;
    }
    // Delete the pool allocated nodes
    if (nodePool) delete nodePool;
    delete [] data.buckets;

    data.buckets = 0;
    data.numBuckets = 0;
    data.numBits = 0;
    data.size = 0;

    newedNodes = 0;
    nodePool = 0;
    link = 0;
}

template<class T>
bool QStringHash<T>::isEmpty() const
{
    return data.size== 0;
}

template<class T>
int QStringHash<T>::count() const
{
    return data.size;
}

template<class T>
int QStringHash<T>::numBuckets() const
{
    return data.numBuckets;
}

template<class T>
bool QStringHash<T>::isLinked() const
{
    return link != 0;
}

template<class T>
void QStringHash<T>::initializeNode(Node *node, const QHashedString &key)
{
    node->length = key.length();
    node->hash = key.hash();
    node->strData = const_cast<QHashedString &>(key).data_ptr();
    node->strData->ref.ref();
    node->setQString(true);
}

template<class T>
void QStringHash<T>::initializeNode(Node *node, const QHashedCStringRef &key)
{
    node->length = key.length();
    node->hash = key.hash();
    node->ckey = key.constData();
}

template<class T>
template<class K>
typename QStringHash<T>::Node *QStringHash<T>::takeNode(const K &key, const T &value)
{
    if (nodePool && nodePool->used != nodePool->count) {
        Node *rv = nodePool->nodes + nodePool->used++;
        initializeNode(rv, hashedString(key));
        rv->value = value;
        return rv;
    } else {
        NewedNode *rv = new NewedNode(hashedString(key), value);
        rv->nextNewed = newedNodes;
        newedNodes = rv;
        return rv;
    }
}

template<class T>
typename QStringHash<T>::Node *QStringHash<T>::takeNode(const Node &o)
{
    if (nodePool && nodePool->used != nodePool->count) {
        Node *rv = nodePool->nodes + nodePool->used++;
        rv->length = o.length;
        rv->hash = o.hash;
        if (o.isQString()) {
            rv->strData = o.strData;
            rv->strData->ref.ref();
            rv->setQString(true);
        } else {
            rv->ckey = o.ckey;
        }
        rv->symbolId = o.symbolId;
        rv->value = o.value;
        return rv;
    } else {
        NewedNode *rv = new NewedNode(o);
        rv->nextNewed = newedNodes;
        newedNodes = rv;
        return rv;
    }
}

template<class T>
void QStringHash<T>::copyNode(const QStringHashNode *otherNode)
{
    // Copy the predecessor before the successor
    QStringHashNode *next = otherNode->next.data();
    if (next)
        copyNode(next);

    Node *mynode = takeNode(*(const Node *)otherNode);
    int bucket = mynode->hash % data.numBuckets;
    mynode->next = data.buckets[bucket];
    data.buckets[bucket] = mynode;
}

template<class T>
void QStringHash<T>::copy(const QStringHash<T> &other)
{
    Q_ASSERT(data.size == 0);

    data.size = other.data.size;

    // Ensure buckets array is created
    data.rehashToBits(data.numBits);

    // Preserve the existing order within buckets
    for (int i = 0; i < other.data.numBuckets; ++i) {
        QStringHashNode *bucket = other.data.buckets[i];
        if (bucket)
            copyNode(bucket);
    }
}

template<class T>
QStringHashData::IteratorData
QStringHash<T>::iterateNext(const QStringHashData::IteratorData &d)
{
    QStringHash<T> *This = (QStringHash<T> *)d.p;
    Node *node = (Node *)d.n;

    if (This->nodePool && node >= This->nodePool->nodes &&
        node < (This->nodePool->nodes + This->nodePool->used)) {
        node--;
        if (node < This->nodePool->nodes)
            node = 0;
    } else {
        NewedNode *nn = (NewedNode *)node;
        node = nn->nextNewed;

        if (node == 0 && This->nodePool && This->nodePool->used)
            node = This->nodePool->nodes + This->nodePool->used - 1;
    }

    if (node == 0 && This->link)
        return This->link->iterateFirst();

    QStringHashData::IteratorData rv;
    rv.n = node;
    rv.p = d.p;
    return rv;
}

template<class T>
QStringHashData::IteratorData QStringHash<T>::iterateFirst() const
{
    Node *n = 0;
    if (newedNodes)
        n = newedNodes;
    else if (nodePool && nodePool->used)
        n = nodePool->nodes + nodePool->used - 1;

    if (n == 0 && link)
        return link->iterateFirst();

    QStringHashData::IteratorData rv;
    rv.n = n;
    rv.p = const_cast<QStringHash<T> *>(this);
    return rv;
}

template<class T>
typename QStringHash<T>::ConstIterator QStringHash<T>::iterator(Node *n) const
{
    if (!n)
        return ConstIterator();

    const QStringHash<T> *container = this;

    if (link) {
        // This node could be in the linked hash
        if ((n >= nodePool->nodes) && (n < (nodePool->nodes + nodePool->used))) {
            // The node is in this hash
        } else if ((n >= link->nodePool->nodes) && (n < (link->nodePool->nodes + link->nodePool->used))) {
            // The node is in the linked hash
            container = link;
        } else {
            const NewedNode *ln = link->newedNodes;
            while (ln) {
                if (ln == n) {
                    // This node is in the linked hash's newed list
                    container = link;
                    break;
                }
                ln = ln->nextNewed;
            }
        }
    }

    QStringHashData::IteratorData rv;
    rv.n = n;
    rv.p = const_cast<QStringHash<T> *>(container);
    return ConstIterator(rv);
}

template<class T>
typename QStringHash<T>::Node *QStringHash<T>::createNode(const Node &o)
{
    Node *n = takeNode(o);
    return insertNode(n, n->hash);
}

template<class T>
template<class K>
typename QStringHash<T>::Node *QStringHash<T>::createNode(const K &key, const T &value)
{
    Node *n = takeNode(key, value);
    return insertNode(n, hashOf(key));
}

template<class T>
typename QStringHash<T>::Node *QStringHash<T>::insertNode(Node *n, quint32 hash)
{
    if (data.size >= data.numBuckets)
        data.rehashToBits(data.numBits + 1);

    int bucket = hash % data.numBuckets;
    n->next = data.buckets[bucket];
    data.buckets[bucket] = n;

    data.size++;

    return n;
}

template<class T>
template<class K>
void QStringHash<T>::insert(const K &key, const T &value)
{
    // If this is a linked hash, we can't rely on owning the node, so we always
    // create a new one.
    Node *n = link?0:findNode(key);
    if (n) n->value = value;
    else createNode(key, value);
}

template<class T>
void QStringHash<T>::insert(const ConstIterator &iter)
{
    insert(iter.key(), iter.value());
}

template<class T>
template<class K>
typename QStringHash<T>::Node *QStringHash<T>::findNode(const K &key) const
{
    QStringHashNode *node = data.numBuckets?data.buckets[hashOf(key) % data.numBuckets]:0;

    typename HashedForm<K>::Type hashedKey(hashedString(key));
    while (node && !node->equals(hashedKey))
        node = (*node->next);

    return (Node *)node;
}

template<class T>
template<class K>
T *QStringHash<T>::value(const K &key) const
{
    Node *n = findNode(key);
    return n?&n->value:0;
}

template<class T>
T *QStringHash<T>::value(const ConstIterator &iter) const
{
    Node *n = iter.node();
    return value(n->key());
}

template<class T>
T *QStringHash<T>::value(const QV4::String *string) const
{
    Node *n = findNode(string);
    return n?&n->value:0;
}

template<class T>
template<class K>
bool QStringHash<T>::contains(const K &key) const
{
    return 0 != value(key);
}

template<class T>
template<class K>
T &QStringHash<T>::operator[](const K &key)
{
    Node *n = findNode(key);
    if (n) return n->value;
    else return createNode(key, T())->value;
}

template<class T>
void QStringHash<T>::reserve(int n)
{
    if (nodePool || 0 == n)
        return;

    nodePool = new ReservedNodePool;
    nodePool->count = n;
    nodePool->used = 0;
    nodePool->nodes = new Node[n];

    data.rehashToSize(n);
}

template<class T>
QStringHash<T>::ConstIterator::ConstIterator()
{
}

template<class T>
QStringHash<T>::ConstIterator::ConstIterator(const QStringHashData::IteratorData &d)
: d(d)
{
}

template<class T>
typename QStringHash<T>::ConstIterator &QStringHash<T>::ConstIterator::operator++()
{
    d = QStringHash<T>::iterateNext(d);
    return *this;
}

template<class T>
bool QStringHash<T>::ConstIterator::operator==(const ConstIterator &o) const
{
    return d.n == o.d.n;
}

template<class T>
bool QStringHash<T>::ConstIterator::operator!=(const ConstIterator &o) const
{
    return d.n != o.d.n;
}

template<class T>
template<typename K>
bool QStringHash<T>::ConstIterator::equals(const K &key) const
{
    return d.n->equals(key);
}

template<class T>
QHashedString QStringHash<T>::ConstIterator::key() const
{
    Node *n = (Node *)d.n;
    return n->key();
}
template<class T>
const T &QStringHash<T>::ConstIterator::value() const
{
    Node *n = (Node *)d.n;
    return n->value;
}

template<class T>
const T &QStringHash<T>::ConstIterator::operator*() const
{
    Node *n = (Node *)d.n;
    return n->value;
}

template<class T>
typename QStringHash<T>::Node *QStringHash<T>::ConstIterator::node() const
{
    Node *n = (Node *)d.n;
    return n;
}

template<class T>
typename QStringHash<T>::ConstIterator QStringHash<T>::begin() const
{
    return ConstIterator(iterateFirst());
}

template<class T>
typename QStringHash<T>::ConstIterator QStringHash<T>::end() const
{
    return ConstIterator();
}

template<class T>
template<class K>
typename QStringHash<T>::ConstIterator QStringHash<T>::find(const K &key) const
{
    return iterator(findNode(key));
}

template<class T>
class QStringMultiHash : public QStringHash<T>
{
public:
    typedef typename QStringHash<T>::ConstIterator ConstIterator;

    template<typename K>
    inline void insert(const K &, const T &);

    inline void insert(const ConstIterator &);

    inline ConstIterator findNext(const ConstIterator &) const;
};

template<class T>
template<class K>
void QStringMultiHash<T>::insert(const K &key, const T &value)
{
    // Always create a new node
    QStringHash<T>::createNode(key, value);
}

template<class T>
void QStringMultiHash<T>::insert(const ConstIterator &iter)
{
    // Always create a new node
    QStringHash<T>::createNode(iter.key(), iter.value());
}

template<class T>
typename QStringHash<T>::ConstIterator QStringMultiHash<T>::findNext(const ConstIterator &iter) const
{
    QStringHashNode *node = iter.node();
    if (node) {
        QHashedString key(node->key());

        while ((node = *node->next)) {
            if (node->equals(key)) {
                return QStringHash<T>::iterator(static_cast<typename QStringHash<T>::Node *>(node));
            }
        }
    }

    return ConstIterator();
}

inline uint qHash(const QHashedString &string)
{
    return uint(string.hash());
}

inline uint qHash(const QHashedStringRef &string)
{
    return uint(string.hash());
}

QHashedString::QHashedString()
: QString(), m_hash(0)
{
}

QHashedString::QHashedString(const QString &string)
: QString(string), m_hash(0)
{
}

QHashedString::QHashedString(const QString &string, quint32 hash)
: QString(string), m_hash(hash)
{
}

QHashedString::QHashedString(const QHashedString &string)
: QString(string), m_hash(string.m_hash)
{
}

QHashedString &QHashedString::operator=(const QHashedString &string)
{
    static_cast<QString &>(*this) = string;
    m_hash = string.m_hash;
    return *this;
}

bool QHashedString::operator==(const QHashedString &string) const
{
    return (string.m_hash == m_hash || !string.m_hash || !m_hash) &&
           static_cast<const QString &>(*this) == static_cast<const QString &>(string);
}

bool QHashedString::operator==(const QHashedStringRef &string) const
{
    return length() == string.m_length &&
           (string.m_hash == m_hash || !string.m_hash || !m_hash) &&
           QHashedString::compare(constData(), string.m_data, string.m_length);
}

quint32 QHashedString::hash() const
{
    if (!m_hash) computeHash();
    return m_hash;
}

quint32 QHashedString::existingHash() const
{
    return m_hash;
}

QHashedStringRef::QHashedStringRef()
: m_data(0), m_length(0), m_hash(0)
{
}

QHashedStringRef::QHashedStringRef(const QString &str)
: m_data(str.constData()), m_length(str.length()), m_hash(0)
{
}

QHashedStringRef::QHashedStringRef(const QStringRef &str)
: m_data(str.constData()), m_length(str.length()), m_hash(0)
{
}

QHashedStringRef::QHashedStringRef(const QChar *data, int length)
: m_data(data), m_length(length), m_hash(0)
{
}

QHashedStringRef::QHashedStringRef(const QChar *data, int length, quint32 hash)
: m_data(data), m_length(length), m_hash(hash)
{
}

QHashedStringRef::QHashedStringRef(const QHashedString &string)
: m_data(string.constData()), m_length(string.length()), m_hash(string.m_hash)
{
}

QHashedStringRef::QHashedStringRef(const QHashedStringRef &string)
: m_data(string.m_data), m_length(string.m_length), m_hash(string.m_hash)
{
}

QHashedStringRef &QHashedStringRef::operator=(const QHashedStringRef &o)
{
    m_data = o.m_data;
    m_length = o.m_length;
    m_hash = o.m_hash;
    return *this;
}

bool QHashedStringRef::operator==(const QString &string) const
{
    return m_length == string.length() &&
           QHashedString::compare(string.constData(), m_data, m_length);
}

bool QHashedStringRef::operator==(const QHashedString &string) const
{
    return m_length == string.length() &&
           (m_hash == string.m_hash || !m_hash || !string.m_hash) &&
           QHashedString::compare(string.constData(), m_data, m_length);
}

bool QHashedStringRef::operator==(const QHashedStringRef &string) const
{
    return m_length == string.m_length &&
           (m_hash == string.m_hash || !m_hash || !string.m_hash) &&
           QHashedString::compare(string.m_data, m_data, m_length);
}

bool QHashedStringRef::operator==(const QHashedCStringRef &string) const
{
    return m_length == string.m_length &&
           (m_hash == string.m_hash || !m_hash || !string.m_hash) &&
           QHashedString::compare(m_data, string.m_data, m_length);
}

bool QHashedStringRef::operator!=(const QString &string) const
{
    return m_length != string.length() ||
           !QHashedString::compare(string.constData(), m_data, m_length);
}

bool QHashedStringRef::operator!=(const QHashedString &string) const
{
    return m_length != string.length() ||
           (m_hash != string.m_hash && m_hash && string.m_hash) ||
           !QHashedString::compare(string.constData(), m_data, m_length);
}

bool QHashedStringRef::operator!=(const QHashedStringRef &string) const
{
    return m_length != string.m_length ||
           (m_hash != string.m_hash && m_hash && string.m_hash) ||
           QHashedString::compare(string.m_data, m_data, m_length);
}

bool QHashedStringRef::operator!=(const QHashedCStringRef &string) const
{
    return m_length != string.m_length ||
           (m_hash != string.m_hash && m_hash && string.m_hash) ||
           QHashedString::compare(m_data, string.m_data, m_length);
}

QChar *QHashedStringRef::data()
{
    return const_cast<QChar *>(m_data);
}

const QChar &QHashedStringRef::at(int index) const
{
    Q_ASSERT(index < m_length);
    return m_data[index];
}

const QChar *QHashedStringRef::constData() const
{
    return m_data;
}

bool QHashedStringRef::isEmpty() const
{
    return m_length == 0;
}

int QHashedStringRef::length() const
{
    return m_length;
}

bool QHashedStringRef::isLatin1() const
{
    for (int ii = 0; ii < m_length; ++ii)
        if (m_data[ii].unicode() > 127) return false;
    return true;
}

bool QHashedStringRef::startsWithUpper() const
{
    if (m_length < 1) return false;
    return m_data[0].isUpper();
}

quint32 QHashedStringRef::hash() const
{
    if (!m_hash) computeHash();
    return m_hash;
}

QHashedCStringRef::QHashedCStringRef()
: m_data(0), m_length(0), m_hash(0)
{
}

QHashedCStringRef::QHashedCStringRef(const char *data, int length)
: m_data(data), m_length(length), m_hash(0)
{
}

QHashedCStringRef::QHashedCStringRef(const char *data, int length, quint32 hash)
: m_data(data), m_length(length), m_hash(hash)
{
}

QHashedCStringRef::QHashedCStringRef(const QHashedCStringRef &o)
: m_data(o.m_data), m_length(o.m_length), m_hash(o.m_hash)
{
}

quint32 QHashedCStringRef::hash() const
{
    if (!m_hash) computeHash();
    return m_hash;
}

const char *QHashedCStringRef::constData() const
{
    return m_data;
}

int QHashedCStringRef::length() const
{
    return m_length;
}

int QHashedCStringRef::utf16length() const
{
    return m_length;
}

void QHashedCStringRef::writeUtf16(QChar *output) const
{
    writeUtf16((quint16 *)output);
}

void QHashedCStringRef::writeUtf16(quint16 *output) const
{
    int l = m_length;
    const char *d = m_data;
    while (l--)
        *output++ = *d++;
}

bool QHashedString::compare(const QChar *lhs, const char *rhs, int length)
{
    Q_ASSERT(lhs && rhs);
    const quint16 *l = (const quint16*)lhs;
    while (length--)
        if (*l++ != *rhs++) return false;
    return true;
}

bool QHashedString::compare(const char *lhs, const char *rhs, int length)
{
    Q_ASSERT(lhs && rhs);
    return 0 == ::memcmp(lhs, rhs, length);
}

QT_END_NAMESPACE

#endif // QHASHEDSTRING_P_H
