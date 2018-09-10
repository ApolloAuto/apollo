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

#ifndef QPOINTERVALUEPAIR_P_H
#define QPOINTERVALUEPAIR_P_H

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

QT_BEGIN_NAMESPACE

// QPointerValuePair is intended to help reduce the memory consumption of a class.
// In the common case, QPointerValuePair behaves like a pointer.  In this mode, it
// consumes the same memory as a regular pointer.
// Additionally, QPointerValuePair can store an arbitrary value type in *addition*
// to the pointer.  In this case, it uses slightly more memory than the pointer and
// value type combined.
// Consequently, this class is most useful in cases where a pointer is always stored
// and a value type is rarely stored.
template<typename P, typename V>
class QPointerValuePair {
public:
    inline QPointerValuePair();
    inline QPointerValuePair(P *);
    inline ~QPointerValuePair();

    inline bool isNull() const;

    inline bool flag() const;
    inline void setFlag();
    inline void clearFlag();
    inline void setFlagValue(bool);

    inline QPointerValuePair<P, V> &operator=(P *);

    inline P *operator->() const;
    inline P *operator*() const;

    inline bool hasValue() const;
    inline V &value();
    inline const V *constValue() const;

private:
    struct Value { P *pointer; V value; };
    QBiPointer<P, Value> d;
};

template<typename P, typename V>
QPointerValuePair<P, V>::QPointerValuePair()
{
}

template<typename P, typename V>
QPointerValuePair<P, V>::QPointerValuePair(P *p)
: d(p)
{
}

template<typename P, typename V>
QPointerValuePair<P, V>::~QPointerValuePair()
{
    if (d.isT2()) delete d.asT2();
}

template<typename P, typename V>
bool QPointerValuePair<P, V>::isNull() const
{
    if (d.isT1()) return 0 == d.asT1();
    else return d.asT2()->pointer == 0;
}

template<typename P, typename V>
bool QPointerValuePair<P, V>::flag() const
{
    return d.flag();
}

template<typename P, typename V>
void QPointerValuePair<P, V>::setFlag()
{
    d.setFlag();
}

template<typename P, typename V>
void QPointerValuePair<P, V>::clearFlag()
{
    d.clearFlag();
}

template<typename P, typename V>
void QPointerValuePair<P, V>::setFlagValue(bool v)
{
    d.setFlagValue(v);
}

template<typename P, typename V>
QPointerValuePair<P, V> &QPointerValuePair<P, V>::operator=(P *o)
{
    if (d.isT1()) d = o;
    else d.asT2()->pointer = o;
    return *this;
}

template<typename P, typename V>
P *QPointerValuePair<P, V>::operator->() const
{
    if (d.isT1()) return d.asT1();
    else return d.asT2()->pointer;
}

template<typename P, typename V>
P *QPointerValuePair<P, V>::operator*() const
{
    if (d.isT1()) return d.asT1();
    else return d.asT2()->pointer;
}

template<typename P, typename V>
bool QPointerValuePair<P, V>::hasValue() const
{
    return d.isT2();
}

template<typename P, typename V>
V &QPointerValuePair<P, V>::value()
{
    if (d.isT1()) {
        P *p = d.asT1();
        Value *value = new Value;
        value->pointer = p;
        d = value;
    }

    return d.asT2()->value;
}

// Will return null if hasValue() == false
template<typename P, typename V>
const V *QPointerValuePair<P, V>::constValue() const
{
    if (d.isT2()) return &d.asT2()->value;
    else return 0;
}

QT_END_NAMESPACE

#endif // QPOINTERVALUEPAIR_P_H
