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

#ifndef QENDIAN_H
#define QENDIAN_H

#include <QtCore/qglobal.h>

// include stdlib.h and hope that it defines __GLIBC__ for glibc-based systems
#include <stdlib.h>
#include <string.h>

QT_BEGIN_NAMESPACE


/*
 * ENDIAN FUNCTIONS
*/
inline void qbswap_helper(const uchar *src, uchar *dest, int size)
{
    for (int i = 0; i < size ; ++i) dest[i] = src[size - 1 - i];
}

/*
 * qbswap(const T src, const uchar *dest);
 * Changes the byte order of \a src from big endian to little endian or vice versa
 * and stores the result in \a dest.
 * There is no alignment requirements for \a dest.
*/
template <typename T> inline void qbswap(const T src, uchar *dest)
{
    qbswap_helper(reinterpret_cast<const uchar *>(&src), dest, sizeof(T));
}

// Used to implement a type-safe and alignment-safe copy operation
// If you want to avoid the memcpy, you must write specializations for these functions
template <typename T> inline void qToUnaligned(const T src, uchar *dest)
{
    // Using sizeof(T) inside memcpy function produces internal compiler error with
    // MSVC2008/ARM in tst_endian -> use extra indirection to resolve size of T.
    const size_t size = sizeof(T);
    memcpy(dest, &src, size);
}
template <typename T> inline T qFromUnaligned(const uchar *src)
{
    T dest;
    const size_t size = sizeof(T);
    memcpy(&dest, src, size);
    return dest;
}

/*
 * T qbswap(T source).
 * Changes the byte order of a value from big endian to little endian or vice versa.
 * This function can be used if you are not concerned about alignment issues,
 * and it is therefore a bit more convenient and in most cases more efficient.
*/
template <typename T> T qbswap(T source);

#ifdef __has_builtin
#  define QT_HAS_BUILTIN(x)     __has_builtin(x)
#else
#  define QT_HAS_BUILTIN(x)     0
#endif

// GCC 4.3 implemented all the intrinsics, but the 16-bit one only got implemented in 4.8;
// Clang 2.6 implemented the 32- and 64-bit but waited until 3.2 to implement the 16-bit one
#if (defined(Q_CC_GNU) && Q_CC_GNU >= 403) || QT_HAS_BUILTIN(__builtin_bswap32)
template <> inline quint64 qbswap<quint64>(quint64 source)
{
    return __builtin_bswap64(source);
}
template <> inline quint32 qbswap<quint32>(quint32 source)
{
    return __builtin_bswap32(source);
}

template <> inline void qbswap<quint64>(quint64 source, uchar *dest)
{
    qToUnaligned<quint64>(__builtin_bswap64(source), dest);
}
template <> inline void qbswap<quint32>(quint32 source, uchar *dest)
{
    qToUnaligned<quint32>(__builtin_bswap32(source), dest);
}
#else
template <> inline quint64 qbswap<quint64>(quint64 source)
{
    return 0
        | ((source & Q_UINT64_C(0x00000000000000ff)) << 56)
        | ((source & Q_UINT64_C(0x000000000000ff00)) << 40)
        | ((source & Q_UINT64_C(0x0000000000ff0000)) << 24)
        | ((source & Q_UINT64_C(0x00000000ff000000)) << 8)
        | ((source & Q_UINT64_C(0x000000ff00000000)) >> 8)
        | ((source & Q_UINT64_C(0x0000ff0000000000)) >> 24)
        | ((source & Q_UINT64_C(0x00ff000000000000)) >> 40)
        | ((source & Q_UINT64_C(0xff00000000000000)) >> 56);
}

template <> inline quint32 qbswap<quint32>(quint32 source)
{
    return 0
        | ((source & 0x000000ff) << 24)
        | ((source & 0x0000ff00) << 8)
        | ((source & 0x00ff0000) >> 8)
        | ((source & 0xff000000) >> 24);
}
#endif // GCC & Clang intrinsics
#if (defined(Q_CC_GNU) && Q_CC_GNU >= 408) || QT_HAS_BUILTIN(__builtin_bswap16)
template <> inline quint16 qbswap<quint16>(quint16 source)
{
    return __builtin_bswap16(source);
}
template <> inline void qbswap<quint16>(quint16 source, uchar *dest)
{
    qToUnaligned<quint16>(__builtin_bswap16(source), dest);
}
#else
template <> inline quint16 qbswap<quint16>(quint16 source)
{
    return quint16( 0
                    | ((source & 0x00ff) << 8)
                    | ((source & 0xff00) >> 8) );
}
#endif // GCC & Clang intrinsics

#undef QT_HAS_BUILTIN

// signed specializations
template <> inline qint64 qbswap<qint64>(qint64 source)
{
    return qbswap<quint64>(quint64(source));
}

template <> inline qint32 qbswap<qint32>(qint32 source)
{
    return qbswap<quint32>(quint32(source));
}

template <> inline qint16 qbswap<qint16>(qint16 source)
{
    return qbswap<quint16>(quint16(source));
}

template <> inline void qbswap<qint64>(qint64 source, uchar *dest)
{
    qbswap<quint64>(quint64(source), dest);
}

template <> inline void qbswap<qint32>(qint32 source, uchar *dest)
{
    qbswap<quint32>(quint32(source), dest);
}

template <> inline void qbswap<qint16>(qint16 source, uchar *dest)
{
    qbswap<quint16>(quint16(source), dest);
}

#if Q_BYTE_ORDER == Q_BIG_ENDIAN

template <typename T> inline T qToBigEndian(T source)
{ return source; }
template <typename T> inline T qFromBigEndian(T source)
{ return source; }
template <typename T> inline T qToLittleEndian(T source)
{ return qbswap<T>(source); }
template <typename T> inline T qFromLittleEndian(T source)
{ return qbswap<T>(source); }
template <typename T> inline void qToBigEndian(T src, uchar *dest)
{ qToUnaligned<T>(src, dest); }
template <typename T> inline void qToLittleEndian(T src, uchar *dest)
{ qbswap<T>(src, dest); }
#else // Q_LITTLE_ENDIAN

template <typename T> inline T qToBigEndian(T source)
{ return qbswap<T>(source); }
template <typename T> inline T qFromBigEndian(T source)
{ return qbswap<T>(source); }
template <typename T> inline T qToLittleEndian(T source)
{ return source; }
template <typename T> inline T qFromLittleEndian(T source)
{ return source; }
template <typename T> inline void qToBigEndian(T src, uchar *dest)
{ qbswap<T>(src, dest); }
template <typename T> inline void qToLittleEndian(T src, uchar *dest)
{ qToUnaligned<T>(src, dest); }

#endif // Q_BYTE_ORDER == Q_BIG_ENDIAN

template <> inline quint8 qbswap<quint8>(quint8 source)
{
    return source;
}

template <> inline qint8 qbswap<qint8>(qint8 source)
{
    return source;
}

/* T qFromLittleEndian(const uchar *src)
 * This function will read a little-endian encoded value from \a src
 * and return the value in host-endian encoding.
 * There is no requirement that \a src must be aligned.
*/
template <typename T> inline T qFromLittleEndian(const uchar *src)
{
    return qFromLittleEndian(qFromUnaligned<T>(src));
}

template <> inline quint8 qFromLittleEndian<quint8>(const uchar *src)
{ return static_cast<quint8>(src[0]); }
template <> inline qint8 qFromLittleEndian<qint8>(const uchar *src)
{ return static_cast<qint8>(src[0]); }

/* This function will read a big-endian (also known as network order) encoded value from \a src
 * and return the value in host-endian encoding.
 * There is no requirement that \a src must be aligned.
*/
template <class T> inline T qFromBigEndian(const uchar *src)
{
    return qFromBigEndian(qFromUnaligned<T>(src));
}

template <> inline quint8 qFromBigEndian<quint8>(const uchar *src)
{ return static_cast<quint8>(src[0]); }
template <> inline qint8 qFromBigEndian<qint8>(const uchar *src)
{ return static_cast<qint8>(src[0]); }

QT_END_NAMESPACE

#endif // QENDIAN_H
