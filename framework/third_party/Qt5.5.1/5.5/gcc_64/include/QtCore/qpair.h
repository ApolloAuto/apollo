/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#ifndef QPAIR_H
#define QPAIR_H

#include <QtCore/qglobal.h>

QT_BEGIN_NAMESPACE


template <class T1, class T2>
struct QPair
{
    typedef T1 first_type;
    typedef T2 second_type;

    Q_DECL_CONSTEXPR QPair()
        Q_DECL_NOEXCEPT_EXPR(noexcept(T1()) && noexcept(T2()))
        : first(), second() {}
    Q_DECL_CONSTEXPR QPair(const T1 &t1, const T2 &t2)
        Q_DECL_NOEXCEPT_EXPR(noexcept(T1(t1)) && noexcept(T2(t2)))
        : first(t1), second(t2) {}
    // compiler-generated copy/move ctor/assignment operators are fine!

    template <typename TT1, typename TT2>
    Q_DECL_CONSTEXPR QPair(const QPair<TT1, TT2> &p)
        Q_DECL_NOEXCEPT_EXPR(noexcept(T1(p.first)) && noexcept(T2(p.second)))
        : first(p.first), second(p.second) {}
    template <typename TT1, typename TT2>
    Q_DECL_RELAXED_CONSTEXPR QPair &operator=(const QPair<TT1, TT2> &p)
        Q_DECL_NOEXCEPT_EXPR(noexcept(std::declval<T1&>() = p.first) && noexcept(std::declval<T2&>() = p.second))
    { first = p.first; second = p.second; return *this; }
#ifdef Q_COMPILER_RVALUE_REFS
    template <typename TT1, typename TT2>
    Q_DECL_CONSTEXPR QPair(QPair<TT1, TT2> &&p)
        // can't use std::move here as it's not constexpr in C++11:
        Q_DECL_NOEXCEPT_EXPR(noexcept(T1(static_cast<TT1 &&>(p.first))) && noexcept(T2(static_cast<TT2 &&>(p.second))))
        : first(static_cast<TT1 &&>(p.first)), second(static_cast<TT2 &&>(p.second)) {}
    template <typename TT1, typename TT2>
    Q_DECL_RELAXED_CONSTEXPR QPair &operator=(QPair<TT1, TT2> &&p)
        Q_DECL_NOEXCEPT_EXPR(noexcept(std::declval<T1&>() = std::move(p.first)) && noexcept(std::declval<T2&>() = std::move(p.second)))
    { first = std::move(p.first); second = std::move(p.second); return *this; }
#endif

    Q_DECL_RELAXED_CONSTEXPR void swap(QPair &other)
        Q_DECL_NOEXCEPT_EXPR(noexcept(qSwap(other.first, other.first)) && noexcept(qSwap(other.second, other.second)))
    {
        // use qSwap() to pick up ADL swaps automatically:
        qSwap(first, other.first);
        qSwap(second, other.second);
    }

    T1 first;
    T2 second;
};

template <typename T1, typename T2>
void swap(QPair<T1, T2> &lhs, QPair<T1, T2> &rhs) Q_DECL_NOEXCEPT_EXPR(noexcept(lhs.swap(rhs)))
{ lhs.swap(rhs); }

// mark QPair<T1,T2> as complex/movable/primitive depending on the
// typeinfos of the constituents:
template<class T1, class T2>
class QTypeInfo<QPair<T1, T2> > : public QTypeInfoMerger<QPair<T1, T2>, T1, T2> {}; // Q_DECLARE_TYPEINFO

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator==(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(p1.first == p2.first && p1.second == p2.second))
{ return p1.first == p2.first && p1.second == p2.second; }

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator!=(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(!(p1 == p2)))
{ return !(p1 == p2); }

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator<(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(p1.first < p2.first || (!(p2.first < p1.first) && p1.second < p2.second)))
{
    return p1.first < p2.first || (!(p2.first < p1.first) && p1.second < p2.second);
}

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator>(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(p2 < p1))
{
    return p2 < p1;
}

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator<=(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(!(p2 < p1)))
{
    return !(p2 < p1);
}

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_INLINE_TEMPLATE bool operator>=(const QPair<T1, T2> &p1, const QPair<T1, T2> &p2)
    Q_DECL_NOEXCEPT_EXPR(noexcept(!(p1 < p2)))
{
    return !(p1 < p2);
}

template <class T1, class T2>
Q_DECL_CONSTEXPR Q_OUTOFLINE_TEMPLATE QPair<T1, T2> qMakePair(const T1 &x, const T2 &y)
    Q_DECL_NOEXCEPT_EXPR(noexcept(QPair<T1, T2>(x, y)))
{
    return QPair<T1, T2>(x, y);
}

QT_END_NAMESPACE

#endif // QPAIR_H
