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

#ifndef QQMLPOOL_P_H
#define QQMLPOOL_P_H

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

#include <private/qv4global_p.h>
#include <QtCore/qstring.h>
#include <QtCore/qurl.h>

QT_BEGIN_NAMESPACE

class Q_QML_PRIVATE_EXPORT QQmlPool
{
public:
    // The class has a destructor that needs to be called
    class Class {
    public:
        inline QQmlPool *pool() const;

    private:
        void *operator new(size_t);
        void *operator new(size_t, void *m) { return m; }
        friend class QQmlPool;

        QQmlPool *_pool;
        Class *_next;
        void (*_destroy)(Class *);
    };

    // The class is plain old data and no destructor needs to
    // be called
    class POD {
    public:
        inline QQmlPool *pool() const;

    private:
        void *operator new(size_t);
        void *operator new(size_t, void *m) { return m; }
        friend class QQmlPool;

        QQmlPool *_pool;
    };

    inline QQmlPool();
    inline ~QQmlPool();

    void clear();

    template<typename T>
    inline T *New();
    template<typename T>
    inline T *NewRaw();
    template<typename T>
    inline T *NewRawArray(int length);

    inline QString *NewString(const QString &);
    inline QByteArray *NewByteArray(const QByteArray &);
    inline QUrl *NewUrl(const QUrl &);

    template<typename T>
    struct List {
        List() : m_length(0), m_data(0) {}
        List(const List &o) : m_length(o.m_length), m_data(o.m_data) {}
        List &operator=(const List &o) {
            m_length = o.m_length;
            m_data = o.m_data;
            return *this;
        }

        int count() const {
            return m_length;
        }
        int length() const {
            return m_length;
        }
        const T &at(int index) const {
            Q_ASSERT(index < m_length);
            return m_data[index];
        };
        T &operator[](int index) {
            Q_ASSERT(index < m_length);
            return m_data[index];
        };
        const T *data() const { return m_data; }
    private:
        friend class QQmlPool;
        List(T *d, int l) : m_length(l), m_data(d) {}
        int m_length;
        T *m_data;
    };

    template<typename T>
    inline List<T> NewRawList(int length);

private:
    struct StringClass : public QString, public Class {
    };
    struct ByteArrayClass : public QByteArray, public Class {
    };
    struct UrlClass : public QUrl, public Class {
    };

    inline void *allocate(int size);
    void newpage();

    template<typename T>
    inline void initialize(POD *);
    template<typename T>
    inline void initialize(Class *);
    template<typename T>
    static void destroy(Class *c);

    struct Page {
        struct Header {
            Page *next;
            char *free;
        } header;

        static const int pageSize = 4 * 4096 - sizeof(Header);

        char memory[pageSize];
    };

    Page *_page;
    Class *_classList;
};

QQmlPool::QQmlPool()
: _page(0), _classList(0)
{
}

QQmlPool::~QQmlPool()
{
    clear();
}

template<typename T>
T *QQmlPool::New()
{
    T *rv = new (allocate(sizeof(T))) T;
    initialize<T>(rv);
    rv->_pool = this;
    return rv;
}

template<typename T>
T *QQmlPool::NewRaw()
{
    return (T*)allocate(sizeof(T));
}

template<typename T>
T *QQmlPool::NewRawArray(int length)
{
    return (T*)allocate(length * sizeof(T));
}

template<typename T>
QQmlPool::List<T> QQmlPool::NewRawList(int length)
{
    return List<T>(NewRawArray<T>(length), length);
}

QString *QQmlPool::NewString(const QString &s)
{
    QString *rv = New<StringClass>();
    *rv = s;
    return rv;
}

QByteArray *QQmlPool::NewByteArray(const QByteArray &s)
{
    QByteArray *rv = New<ByteArrayClass>();
    *rv = s;
    return rv;
}

QUrl *QQmlPool::NewUrl(const QUrl &s)
{
    QUrl *rv = New<UrlClass>();
    *rv = s;
    return rv;
}

void *QQmlPool::allocate(int size)
{
    if (!_page || (_page->header.free + size) > (_page->memory + Page::pageSize))
        newpage();

    void *rv = _page->header.free;
    _page->header.free += size + ((8 - size) & 7); // ensure 8 byte alignment;
    return rv;
}

template<typename T>
void QQmlPool::initialize(QQmlPool::POD *)
{
}

template<typename T>
void QQmlPool::initialize(QQmlPool::Class *c)
{
    c->_next = _classList;
    c->_destroy = &destroy<T>;
    _classList = c;
}

template<typename T>
void QQmlPool::destroy(Class *c)
{
    static_cast<T *>(c)->~T();
}

QQmlPool *QQmlPool::Class::pool() const
{
    return _pool;
}

QQmlPool *QQmlPool::POD::pool() const
{
    return _pool;
}

QT_END_NAMESPACE

#endif // QQMLPOOL_P_H

