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
#ifndef QMLJS_MATH_H
#define QMLJS_MATH_H

#include <qglobal.h>

#include <QtCore/qnumeric.h>
#include <cmath>

#if defined(Q_CC_GNU)
#define QMLJS_READONLY __attribute((const))
#else
#define QMLJS_READONLY
#endif

QT_BEGIN_NAMESPACE

namespace QV4 {

#if defined(Q_CC_GNU) && defined(Q_PROCESSOR_X86)

static inline QMLJS_READONLY ReturnedValue add_int32(int a, int b)
{
    quint8 overflow = 0;
    int aa = a;

    asm ("addl %2, %1\n"
         "seto %0"
    : "=q" (overflow), "=r" (aa)
         : "r" (b), "1" (aa)
         : "cc"
    );
    if (Q_UNLIKELY(overflow))
        return Primitive::fromDouble(static_cast<double>(a) + b).asReturnedValue();
    return Primitive::fromInt32(aa).asReturnedValue();
}

static inline QMLJS_READONLY ReturnedValue sub_int32(int a, int b)
{
    quint8 overflow = 0;
    int aa = a;

    asm ("subl %2, %1\n"
         "seto %0"
    : "=q" (overflow), "=r" (aa)
         : "r" (b), "1" (aa)
         : "cc"
    );
    if (Q_UNLIKELY(overflow))
        return Primitive::fromDouble(static_cast<double>(a) - b).asReturnedValue();
    return Primitive::fromInt32(aa).asReturnedValue();
}

static inline QMLJS_READONLY ReturnedValue mul_int32(int a, int b)
{
    quint8 overflow = 0;
    int aa = a;

    asm ("imul %2, %1\n"
         "setc %0"
         : "=q" (overflow), "=r" (aa)
         : "r" (b), "1" (aa)
         : "cc"
    );
    if (Q_UNLIKELY(overflow))
        return Primitive::fromDouble(static_cast<double>(a) * b).asReturnedValue();
    return Primitive::fromInt32(aa).asReturnedValue();
}

#else

static inline QMLJS_READONLY ReturnedValue add_int32(int a, int b)
{
    qint64 result = static_cast<qint64>(a) + b;
    if (Q_UNLIKELY(result > INT_MAX || result < INT_MIN))
        return Primitive::fromDouble(static_cast<double>(a) + b).asReturnedValue();
    return Primitive::fromInt32(static_cast<int>(result)).asReturnedValue();
}

static inline QMLJS_READONLY ReturnedValue sub_int32(int a, int b)
{
    qint64 result = static_cast<qint64>(a) - b;
    if (Q_UNLIKELY(result > INT_MAX || result < INT_MIN))
        return Primitive::fromDouble(static_cast<double>(a) - b).asReturnedValue();
    return Primitive::fromInt32(static_cast<int>(result)).asReturnedValue();
}

static inline QMLJS_READONLY ReturnedValue mul_int32(int a, int b)
{
    qint64 result = static_cast<qint64>(a) * b;
    if (Q_UNLIKELY(result > INT_MAX || result < INT_MIN))
        return Primitive::fromDouble(static_cast<double>(a) * b).asReturnedValue();
    return Primitive::fromInt32(static_cast<int>(result)).asReturnedValue();
}

#endif

}

QT_END_NAMESPACE

#ifdef QMLJS_READONLY
#undef QMLJS_READONLY
#endif

#endif // QMLJS_MATH_H
