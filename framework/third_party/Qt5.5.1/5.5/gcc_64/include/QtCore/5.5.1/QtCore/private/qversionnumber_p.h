/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2014 Keith Gardner <kreios4004@gmail.com>
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

#ifndef QVERSIONNUMBER_H
#define QVERSIONNUMBER_H

#include <QtCore/qnamespace.h>
#include <QtCore/qstring.h>
#include <QtCore/qvector.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qtypeinfo.h>

QT_BEGIN_NAMESPACE

class QVersionNumber;
Q_CORE_EXPORT uint qHash(const QVersionNumber &key, uint seed = 0);

#ifndef QT_NO_DATASTREAM
Q_CORE_EXPORT QDataStream& operator<<(QDataStream &out, const QVersionNumber &version);
Q_CORE_EXPORT QDataStream& operator>>(QDataStream &in, QVersionNumber &version);
#endif

class QVersionNumber
{
public:
    inline QVersionNumber() Q_DECL_NOTHROW
        : m_segments()
    {}
    // compiler-generated copy/move ctor/assignment operators are ok

    inline explicit QVersionNumber(const QVector<int> &seg) Q_DECL_NOTHROW
        : m_segments(seg)
    {}
#ifdef Q_COMPILER_RVALUE_REFS
    inline explicit QVersionNumber(QVector<int> &&seg) Q_DECL_NOTHROW
        : m_segments(qMove(seg))
    {}
#endif
#ifdef Q_COMPILER_INITIALIZER_LISTS
    inline QVersionNumber(std::initializer_list<int> args)
        : m_segments(args)
    {}
#endif

    inline explicit QVersionNumber(int maj)
    { m_segments.reserve(1); m_segments << maj; }

    inline explicit QVersionNumber(int maj, int min)
    { m_segments.reserve(2); m_segments << maj << min; }

    inline explicit QVersionNumber(int maj, int min, int mic)
    { m_segments.reserve(3); m_segments << maj << min << mic; }

    inline bool isNull() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return m_segments.isEmpty(); }

    inline bool isNormalized() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return isNull() || m_segments.last() != 0; }

    inline int majorVersion() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return segmentAt(0); }

    inline int minorVersion() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return segmentAt(1); }

    inline int microVersion() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return segmentAt(2); }

#if defined(Q_COMPILER_REF_QUALIFIERS)
#  if defined(Q_CC_GNU)
    // required due to https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61941
#    pragma push_macro("Q_REQUIRED_RESULT")
#    undef Q_REQUIRED_RESULT
#    define Q_REQUIRED_RESULT
#    define Q_REQUIRED_RESULT_pushed
#  endif
    inline QVersionNumber normalized() const & Q_REQUIRED_RESULT
    {
        QVector<int> segs(m_segments);
        return normalizedImpl(segs);
    }

    inline QVersionNumber normalized() && Q_REQUIRED_RESULT
    {
        return normalizedImpl(m_segments);
    }

    inline QVector<int> segments() const & Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return m_segments; }

    inline QVector<int> segments() && Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return qMove(m_segments); }

#  ifdef Q_REQUIRED_RESULT_pushed
#    pragma pop_macro("Q_REQUIRED_RESULT")
#  endif
#else
    inline QVersionNumber normalized() const Q_REQUIRED_RESULT
    {
        QVector<int> segs(m_segments);
        return normalizedImpl(segs);
    }

    inline QVector<int> segments() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return m_segments; }
#endif

    inline int segmentAt(int index) const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return (m_segments.size() > index) ? m_segments.at(index) : 0; }

    inline int segmentCount() const Q_DECL_NOTHROW Q_REQUIRED_RESULT
    { return m_segments.size(); }

    Q_CORE_EXPORT bool isPrefixOf(const QVersionNumber &other) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_CORE_EXPORT static int compare(const QVersionNumber &v1, const QVersionNumber &v2) Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_CORE_EXPORT static Q_DECL_PURE_FUNCTION QVersionNumber commonPrefix(const QVersionNumber &v1, const QVersionNumber &v2) Q_REQUIRED_RESULT;

    Q_CORE_EXPORT QString toString() const Q_REQUIRED_RESULT;
    Q_CORE_EXPORT static Q_DECL_PURE_FUNCTION QVersionNumber fromString(const QString &string, int *suffixIndex = 0) Q_REQUIRED_RESULT;

private:
    Q_CORE_EXPORT static QVersionNumber normalizedImpl(QVector<int> &segs) Q_REQUIRED_RESULT;

#ifndef QT_NO_DATASTREAM
    friend Q_CORE_EXPORT QDataStream& operator>>(QDataStream &in, QVersionNumber &version);
#endif
    friend Q_CORE_EXPORT uint qHash(const QVersionNumber &key, uint seed);

    QVector<int> m_segments;
};

Q_DECLARE_TYPEINFO(QVersionNumber, Q_MOVABLE_TYPE);

#ifndef QT_NO_DEBUG_STREAM
Q_CORE_EXPORT QDebug operator<<(QDebug, const QVersionNumber &version);
#endif

Q_REQUIRED_RESULT inline bool operator> (const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) > 0; }

Q_REQUIRED_RESULT inline bool operator>=(const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) >= 0; }

Q_REQUIRED_RESULT inline bool operator< (const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) < 0; }

Q_REQUIRED_RESULT inline bool operator<=(const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) <= 0; }

Q_REQUIRED_RESULT inline bool operator==(const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) == 0; }

Q_REQUIRED_RESULT inline bool operator!=(const QVersionNumber &lhs, const QVersionNumber &rhs) Q_DECL_NOTHROW
{ return QVersionNumber::compare(lhs, rhs) != 0; }

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QVersionNumber)

#endif //QVERSIONNUMBER_H
