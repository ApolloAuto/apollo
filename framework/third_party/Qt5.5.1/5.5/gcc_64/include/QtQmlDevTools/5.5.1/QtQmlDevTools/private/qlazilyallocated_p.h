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

#ifndef QLAZILYALLOCATED_P_H
#define QLAZILYALLOCATED_P_H

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

template<typename T>
class QLazilyAllocated {
public:
    inline QLazilyAllocated();
    inline ~QLazilyAllocated();

    inline bool isAllocated() const;

    inline T *operator->() const;

    inline T &value();
    inline const T &value() const;

    inline bool flag() const;
    inline void setFlag();
    inline void clearFlag();
    inline void setFlagValue(bool);
private:
    mutable QFlagPointer<T> d;
};

template<typename T>
QLazilyAllocated<T>::QLazilyAllocated()
{
}

template<typename T>
QLazilyAllocated<T>::~QLazilyAllocated()
{
    delete *d;
}

template<typename T>
bool QLazilyAllocated<T>::isAllocated() const
{
    return !d.isNull();
}

template<typename T>
T &QLazilyAllocated<T>::value()
{
    if (d.isNull()) d = new T;
    return *(*d);
}

template<typename T>
const T &QLazilyAllocated<T>::value() const
{
    if (d.isNull()) d = new T;
    return *(*d);
}

template<typename T>
T *QLazilyAllocated<T>::operator->() const
{
    return *d;
}

template<typename T>
bool QLazilyAllocated<T>::flag() const
{
    return d.flag();
}

template<typename T>
void QLazilyAllocated<T>::setFlag()
{
    d.setFlag();
}

template<typename T>
void QLazilyAllocated<T>::clearFlag()
{
    d.clearFlag();
}

template<typename T>
void QLazilyAllocated<T>::setFlagValue(bool v)
{
    d.setFlagValue(v);
}

QT_END_NAMESPACE

#endif // QLAZILYALLOCATED_P_H
