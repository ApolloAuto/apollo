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

#ifndef QFIELDLIST_P_H
#define QFIELDLIST_P_H

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

#include <private/qflagpointer_p.h>

// QForwardFieldList is a super simple linked list that can only prepend
template<class N, N *N::*nextMember>
class QForwardFieldList
{
public:
    inline QForwardFieldList();
    inline N *first() const;
    inline N *takeFirst();

    inline void prepend(N *);

    inline bool isEmpty() const;
    inline bool isOne() const;
    inline bool isMany() const;

    static inline N *next(N *v);

    inline bool flag() const;
    inline void setFlag();
    inline void clearFlag();
    inline void setFlagValue(bool);

    inline bool flag2() const;
    inline void setFlag2();
    inline void clearFlag2();
    inline void setFlag2Value(bool);
private:
    QFlagPointer<N> _first;
};

// QFieldList is a simple linked list, that can append and prepend and also
// maintains a count
template<class N, N *N::*nextMember>
class QFieldList
{
public:
    inline QFieldList();
    inline N *first() const;
    inline N *takeFirst();

    inline void append(N *);
    inline void prepend(N *);

    inline bool isEmpty() const;
    inline bool isOne() const;
    inline bool isMany() const;
    inline int count() const;

    inline void append(QFieldList<N, nextMember> &);
    inline void prepend(QFieldList<N, nextMember> &);
    inline void insertAfter(N *, QFieldList<N, nextMember> &);

    inline void copyAndClear(QFieldList<N, nextMember> &);
    inline void copyAndClearAppend(QForwardFieldList<N, nextMember> &);
    inline void copyAndClearPrepend(QForwardFieldList<N, nextMember> &);

    static inline N *next(N *v);

    inline bool flag() const;
    inline void setFlag();
    inline void clearFlag();
    inline void setFlagValue(bool);
private:
    N *_first;
    N *_last;
    quint32 _flag:1;
    quint32 _count:31;
};

template<class N, N *N::*nextMember>
QForwardFieldList<N, nextMember>::QForwardFieldList()
{
}

template<class N, N *N::*nextMember>
N *QForwardFieldList<N, nextMember>::first() const
{
    return *_first;
}

template<class N, N *N::*nextMember>
N *QForwardFieldList<N, nextMember>::takeFirst()
{
    N *value = *_first;
    if (value) {
        _first = next(value);
        value->*nextMember = 0;
    }
    return value;
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::prepend(N *v)
{
    Q_ASSERT(v->*nextMember == 0);
    v->*nextMember = *_first;
    _first = v;
}

template<class N, N *N::*nextMember>
bool QForwardFieldList<N, nextMember>::isEmpty() const
{
    return _first.isNull();
}

template<class N, N *N::*nextMember>
bool QForwardFieldList<N, nextMember>::isOne() const
{
    return *_first && _first->*nextMember == 0;
}

template<class N, N *N::*nextMember>
bool QForwardFieldList<N, nextMember>::isMany() const
{
    return *_first && _first->*nextMember != 0;
}

template<class N, N *N::*nextMember>
N *QForwardFieldList<N, nextMember>::next(N *v)
{
    Q_ASSERT(v);
    return v->*nextMember;
}

template<class N, N *N::*nextMember>
bool QForwardFieldList<N, nextMember>::flag() const
{
    return _first.flag();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::setFlag()
{
    _first.setFlag();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::clearFlag()
{
    _first.clearFlag();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::setFlagValue(bool v)
{
    _first.setFlagValue(v);
}

template<class N, N *N::*nextMember>
bool QForwardFieldList<N, nextMember>::flag2() const
{
    return _first.flag2();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::setFlag2()
{
    _first.setFlag2();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::clearFlag2()
{
    _first.clearFlag2();
}

template<class N, N *N::*nextMember>
void QForwardFieldList<N, nextMember>::setFlag2Value(bool v)
{
    _first.setFlag2Value(v);
}

template<class N, N *N::*nextMember>
QFieldList<N, nextMember>::QFieldList()
: _first(0), _last(0), _flag(0), _count(0)
{
}

template<class N, N *N::*nextMember>
N *QFieldList<N, nextMember>::first() const
{
    return _first;
}

template<class N, N *N::*nextMember>
N *QFieldList<N, nextMember>::takeFirst()
{
    N *value = _first;
    if (value) {
        _first = next(value);
        if (_last == value) {
            Q_ASSERT(_first == 0);
            _last = 0;
        }
        value->*nextMember = 0;
        --_count;
    }
    return value;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::append(N *v)
{
    Q_ASSERT(v->*nextMember == 0);
    if (isEmpty()) {
        _first = v;
        _last = v;
    } else {
        _last->*nextMember = v;
        _last = v;
    }
    ++_count;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::prepend(N *v)
{
    Q_ASSERT(v->*nextMember == 0);
    if (isEmpty()) {
        _first = v;
        _last = v;
    } else {
        v->*nextMember = _first;
        _first = v;
    }
    ++_count;
}

template<class N, N *N::*nextMember>
bool QFieldList<N, nextMember>::isEmpty() const
{
    return _count == 0;
}

template<class N, N *N::*nextMember>
bool QFieldList<N, nextMember>::isOne() const
{
    return _count == 1;
}

template<class N, N *N::*nextMember>
bool QFieldList<N, nextMember>::isMany() const
{
    return _count > 1;
}

template<class N, N *N::*nextMember>
int QFieldList<N, nextMember>::count() const
{
    return _count;
}

template<class N, N *N::*nextMember>
N *QFieldList<N, nextMember>::next(N *v)
{
    Q_ASSERT(v);
    return v->*nextMember;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::append(QFieldList<N, nextMember> &o)
{
    if (!o.isEmpty()) {
        if (isEmpty()) {
            _first = o._first;
            _last = o._last;
            _count = o._count;
        } else {
            _last->*nextMember = o._first;
            _last = o._last;
            _count += o._count;
        }
        o._first = o._last = 0; o._count = 0;
    }
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::prepend(QFieldList<N, nextMember> &o)
{
    if (!o.isEmpty()) {
        if (isEmpty()) {
            _first = o._first;
            _last = o._last;
            _count = o._count;
        } else {
            o._last->*nextMember = _first;
            _first = o._first;
            _count += o._count;
        }
        o._first = o._last = 0; o._count = 0;
    }
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::insertAfter(N *after, QFieldList<N, nextMember> &o)
{
    if (after == 0) {
        prepend(o);
    } else if (after == _last) {
        append(o);
    } else if (!o.isEmpty()) {
        if (isEmpty()) {
            _first = o._first;
            _last = o._last;
            _count = o._count;
        } else {
            o._last->*nextMember = after->*nextMember;
            after->*nextMember = o._first;
            _count += o._count;
        }
        o._first = o._last = 0; o._count = 0;
    }
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::copyAndClear(QFieldList<N, nextMember> &o)
{
    _first = o._first;
    _last = o._last;
    _count = o._count;
    o._first = o._last = 0;
    o._count = 0;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::copyAndClearAppend(QForwardFieldList<N, nextMember> &o)
{
    _first = 0;
    _last = 0;
    _count = 0;
    while (N *n = o.takeFirst()) append(n);
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::copyAndClearPrepend(QForwardFieldList<N, nextMember> &o)
{
    _first = 0;
    _last = 0;
    _count = 0;
    while (N *n = o.takeFirst()) prepend(n);
}

template<class N, N *N::*nextMember>
bool QFieldList<N, nextMember>::flag() const
{
    return _flag;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::setFlag()
{
    _flag = true;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::clearFlag()
{
    _flag = false;
}

template<class N, N *N::*nextMember>
void QFieldList<N, nextMember>::setFlagValue(bool v)
{
    _flag = v;
}

#endif // QFIELDLIST_P_H
