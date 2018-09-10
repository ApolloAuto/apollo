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

#ifndef QINTRUSIVELIST_P_H
#define QINTRUSIVELIST_P_H

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

QT_BEGIN_NAMESPACE

class QIntrusiveListNode;
template<class N, QIntrusiveListNode N::*member>
class QIntrusiveList
{
public:
    inline QIntrusiveList();
    inline ~QIntrusiveList();

    inline bool isEmpty() const;
    inline void insert(N *n);
    inline void remove(N *n);
    inline bool contains(N *) const;

    class iterator {
    public:
        inline iterator();
        inline iterator(N *value);

        inline N *operator*() const;
        inline N *operator->() const;
        inline bool operator==(const iterator &other) const;
        inline bool operator!=(const iterator &other) const;
        inline iterator &operator++();

        inline iterator &erase();

    private:
        N *_value;
    };
    typedef iterator Iterator;

    inline N *first() const;
    static inline N *next(N *current);

    inline iterator begin();
    inline iterator end();

private:
    static inline N *nodeToN(QIntrusiveListNode *node);

    QIntrusiveListNode *__first;
};

class QIntrusiveListNode
{
public:
    inline QIntrusiveListNode();
    inline ~QIntrusiveListNode();

    inline void remove();
    inline bool isInList() const;

    QIntrusiveListNode *_next;
    QIntrusiveListNode**_prev;
};

template<class N, QIntrusiveListNode N::*member>
QIntrusiveList<N, member>::iterator::iterator()
: _value(0)
{
}

template<class N, QIntrusiveListNode N::*member>
QIntrusiveList<N, member>::iterator::iterator(N *value)
: _value(value)
{
}

template<class N, QIntrusiveListNode N::*member>
N *QIntrusiveList<N, member>::iterator::operator*() const
{
    return _value;
}

template<class N, QIntrusiveListNode N::*member>
N *QIntrusiveList<N, member>::iterator::operator->() const
{
    return _value;
}

template<class N, QIntrusiveListNode N::*member>
bool QIntrusiveList<N, member>::iterator::operator==(const iterator &other) const
{
    return other._value == _value;
}

template<class N, QIntrusiveListNode N::*member>
bool QIntrusiveList<N, member>::iterator::operator!=(const iterator &other) const
{
    return other._value != _value;
}

template<class N, QIntrusiveListNode N::*member>
typename QIntrusiveList<N, member>::iterator &QIntrusiveList<N, member>::iterator::operator++()
{
    _value = QIntrusiveList<N, member>::next(_value);
    return *this;
}

template<class N, QIntrusiveListNode N::*member>
typename QIntrusiveList<N, member>::iterator &QIntrusiveList<N, member>::iterator::erase()
{
    N *old = _value;
    _value = QIntrusiveList<N, member>::next(_value);
    (old->*member).remove();
    return *this;
}

template<class N, QIntrusiveListNode N::*member>
QIntrusiveList<N, member>::QIntrusiveList()
: __first(0)
{
}

template<class N, QIntrusiveListNode N::*member>
QIntrusiveList<N, member>::~QIntrusiveList()
{
    while (__first) __first->remove();
}

template<class N, QIntrusiveListNode N::*member>
bool QIntrusiveList<N, member>::isEmpty() const
{
    return __first == 0;
}

template<class N, QIntrusiveListNode N::*member>
void QIntrusiveList<N, member>::insert(N *n)
{
    QIntrusiveListNode *nnode = &(n->*member);
    nnode->remove();

    nnode->_next = __first;
    if (nnode->_next) nnode->_next->_prev = &nnode->_next;
    __first = nnode;
    nnode->_prev = &__first;
}

template<class N, QIntrusiveListNode N::*member>
void QIntrusiveList<N, member>::remove(N *n)
{
    QIntrusiveListNode *nnode = &(n->*member);
    nnode->remove();
}

template<class N, QIntrusiveListNode N::*member>
bool QIntrusiveList<N, member>::contains(N *n) const
{
    QIntrusiveListNode *nnode = __first;
    while (nnode) {
        if (nodeToN(nnode) == n)
            return true;
        nnode = nnode->_next;
    }
    return false;
}

template<class N, QIntrusiveListNode N::*member>
N *QIntrusiveList<N, member>::first() const
{
    return __first?nodeToN(__first):0;
}

template<class N, QIntrusiveListNode N::*member>
N *QIntrusiveList<N, member>::next(N *current)
{
    QIntrusiveListNode *nextnode = (current->*member)._next;
    N *nextstruct = nextnode?nodeToN(nextnode):0;
    return nextstruct;
}

template<class N, QIntrusiveListNode N::*member>
typename QIntrusiveList<N, member>::iterator QIntrusiveList<N, member>::begin()
{
    return __first?iterator(nodeToN(__first)):iterator();
}

template<class N, QIntrusiveListNode N::*member>
typename QIntrusiveList<N, member>::iterator QIntrusiveList<N, member>::end()
{
    return iterator();
}

template<class N, QIntrusiveListNode N::*member>
N *QIntrusiveList<N, member>::nodeToN(QIntrusiveListNode *node)
{
    return (N *)((char *)node - ((char *)&(((N *)0)->*member) - (char *)0));
}

QIntrusiveListNode::QIntrusiveListNode()
: _next(0), _prev(0)
{
}

QIntrusiveListNode::~QIntrusiveListNode()
{
    remove();
}

void QIntrusiveListNode::remove()
{
    if (_prev) *_prev = _next;
    if (_next) _next->_prev = _prev;
    _prev = 0;
    _next = 0;
}

bool QIntrusiveListNode::isInList() const
{
    return _prev != 0;
}

QT_END_NAMESPACE

#endif // QINTRUSIVELIST_P_H
