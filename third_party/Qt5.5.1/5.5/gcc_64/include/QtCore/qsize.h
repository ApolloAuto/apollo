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

#ifndef QSIZE_H
#define QSIZE_H

#include <QtCore/qnamespace.h>

QT_BEGIN_NAMESPACE


class Q_CORE_EXPORT QSize
{
public:
    Q_DECL_CONSTEXPR QSize() Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QSize(int w, int h) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline bool isNull() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isEmpty() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isValid() const Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline int width() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int height() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setWidth(int w) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setHeight(int h) Q_DECL_NOTHROW;
    void transpose() Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QSize transposed() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    inline void scale(int w, int h, Qt::AspectRatioMode mode) Q_DECL_NOTHROW;
    inline void scale(const QSize &s, Qt::AspectRatioMode mode) Q_DECL_NOTHROW;
    QSize scaled(int w, int h, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    QSize scaled(const QSize &s, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline QSize expandedTo(const QSize &) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    Q_DECL_CONSTEXPR inline QSize boundedTo(const QSize &) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_RELAXED_CONSTEXPR inline int &rwidth() Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline int &rheight() Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline QSize &operator+=(const QSize &) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QSize &operator-=(const QSize &) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QSize &operator*=(qreal c) Q_DECL_NOTHROW;
    inline QSize &operator/=(qreal c);

    friend inline Q_DECL_CONSTEXPR bool operator==(const QSize &, const QSize &) Q_DECL_NOTHROW;
    friend inline Q_DECL_CONSTEXPR bool operator!=(const QSize &, const QSize &) Q_DECL_NOTHROW;
    friend inline Q_DECL_CONSTEXPR const QSize operator+(const QSize &, const QSize &) Q_DECL_NOTHROW;
    friend inline Q_DECL_CONSTEXPR const QSize operator-(const QSize &, const QSize &) Q_DECL_NOTHROW;
    friend inline Q_DECL_CONSTEXPR const QSize operator*(const QSize &, qreal) Q_DECL_NOTHROW;
    friend inline Q_DECL_CONSTEXPR const QSize operator*(qreal, const QSize &) Q_DECL_NOTHROW;
    friend inline const QSize operator/(const QSize &, qreal);

private:
    int wd;
    int ht;
};
Q_DECLARE_TYPEINFO(QSize, Q_MOVABLE_TYPE);

/*****************************************************************************
  QSize stream functions
 *****************************************************************************/

#ifndef QT_NO_DATASTREAM
Q_CORE_EXPORT QDataStream &operator<<(QDataStream &, const QSize &);
Q_CORE_EXPORT QDataStream &operator>>(QDataStream &, QSize &);
#endif


/*****************************************************************************
  QSize inline functions
 *****************************************************************************/

Q_DECL_CONSTEXPR inline QSize::QSize() Q_DECL_NOTHROW : wd(-1), ht(-1) {}

Q_DECL_CONSTEXPR inline QSize::QSize(int w, int h) Q_DECL_NOTHROW : wd(w), ht(h) {}

Q_DECL_CONSTEXPR inline bool QSize::isNull() const Q_DECL_NOTHROW
{ return wd==0 && ht==0; }

Q_DECL_CONSTEXPR inline bool QSize::isEmpty() const Q_DECL_NOTHROW
{ return wd<1 || ht<1; }

Q_DECL_CONSTEXPR inline bool QSize::isValid() const Q_DECL_NOTHROW
{ return wd>=0 && ht>=0; }

Q_DECL_CONSTEXPR inline int QSize::width() const Q_DECL_NOTHROW
{ return wd; }

Q_DECL_CONSTEXPR inline int QSize::height() const Q_DECL_NOTHROW
{ return ht; }

Q_DECL_RELAXED_CONSTEXPR inline void QSize::setWidth(int w) Q_DECL_NOTHROW
{ wd = w; }

Q_DECL_RELAXED_CONSTEXPR inline void QSize::setHeight(int h) Q_DECL_NOTHROW
{ ht = h; }

Q_DECL_CONSTEXPR inline QSize QSize::transposed() const Q_DECL_NOTHROW
{ return QSize(ht, wd); }

inline void QSize::scale(int w, int h, Qt::AspectRatioMode mode) Q_DECL_NOTHROW
{ scale(QSize(w, h), mode); }

inline void QSize::scale(const QSize &s, Qt::AspectRatioMode mode) Q_DECL_NOTHROW
{ *this = scaled(s, mode); }

inline QSize QSize::scaled(int w, int h, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW
{ return scaled(QSize(w, h), mode); }

Q_DECL_RELAXED_CONSTEXPR inline int &QSize::rwidth() Q_DECL_NOTHROW
{ return wd; }

Q_DECL_RELAXED_CONSTEXPR inline int &QSize::rheight() Q_DECL_NOTHROW
{ return ht; }

Q_DECL_RELAXED_CONSTEXPR inline QSize &QSize::operator+=(const QSize &s) Q_DECL_NOTHROW
{ wd+=s.wd; ht+=s.ht; return *this; }

Q_DECL_RELAXED_CONSTEXPR inline QSize &QSize::operator-=(const QSize &s) Q_DECL_NOTHROW
{ wd-=s.wd; ht-=s.ht; return *this; }

Q_DECL_RELAXED_CONSTEXPR inline QSize &QSize::operator*=(qreal c) Q_DECL_NOTHROW
{ wd = qRound(wd*c); ht = qRound(ht*c); return *this; }

Q_DECL_CONSTEXPR inline bool operator==(const QSize &s1, const QSize &s2) Q_DECL_NOTHROW
{ return s1.wd == s2.wd && s1.ht == s2.ht; }

Q_DECL_CONSTEXPR inline bool operator!=(const QSize &s1, const QSize &s2) Q_DECL_NOTHROW
{ return s1.wd != s2.wd || s1.ht != s2.ht; }

Q_DECL_CONSTEXPR inline const QSize operator+(const QSize & s1, const QSize & s2) Q_DECL_NOTHROW
{ return QSize(s1.wd+s2.wd, s1.ht+s2.ht); }

Q_DECL_CONSTEXPR inline const QSize operator-(const QSize &s1, const QSize &s2) Q_DECL_NOTHROW
{ return QSize(s1.wd-s2.wd, s1.ht-s2.ht); }

Q_DECL_CONSTEXPR inline const QSize operator*(const QSize &s, qreal c) Q_DECL_NOTHROW
{ return QSize(qRound(s.wd*c), qRound(s.ht*c)); }

Q_DECL_CONSTEXPR inline const QSize operator*(qreal c, const QSize &s) Q_DECL_NOTHROW
{ return QSize(qRound(s.wd*c), qRound(s.ht*c)); }

inline QSize &QSize::operator/=(qreal c)
{
    Q_ASSERT(!qFuzzyIsNull(c));
    wd = qRound(wd/c); ht = qRound(ht/c);
    return *this;
}

inline const QSize operator/(const QSize &s, qreal c)
{
    Q_ASSERT(!qFuzzyIsNull(c));
    return QSize(qRound(s.wd/c), qRound(s.ht/c));
}

Q_DECL_CONSTEXPR inline QSize QSize::expandedTo(const QSize & otherSize) const Q_DECL_NOTHROW
{
    return QSize(qMax(wd,otherSize.wd), qMax(ht,otherSize.ht));
}

Q_DECL_CONSTEXPR inline QSize QSize::boundedTo(const QSize & otherSize) const Q_DECL_NOTHROW
{
    return QSize(qMin(wd,otherSize.wd), qMin(ht,otherSize.ht));
}

#ifndef QT_NO_DEBUG_STREAM
Q_CORE_EXPORT QDebug operator<<(QDebug, const QSize &);
#endif


class Q_CORE_EXPORT QSizeF
{
public:
    Q_DECL_CONSTEXPR QSizeF() Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QSizeF(const QSize &sz) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QSizeF(qreal w, qreal h) Q_DECL_NOTHROW;

    inline bool isNull() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isEmpty() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isValid() const Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline qreal width() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline qreal height() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setWidth(qreal w) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setHeight(qreal h) Q_DECL_NOTHROW;
    void transpose() Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QSizeF transposed() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    inline void scale(qreal w, qreal h, Qt::AspectRatioMode mode) Q_DECL_NOTHROW;
    inline void scale(const QSizeF &s, Qt::AspectRatioMode mode) Q_DECL_NOTHROW;
    QSizeF scaled(qreal w, qreal h, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    QSizeF scaled(const QSizeF &s, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline QSizeF expandedTo(const QSizeF &) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    Q_DECL_CONSTEXPR inline QSizeF boundedTo(const QSizeF &) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_RELAXED_CONSTEXPR inline qreal &rwidth() Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline qreal &rheight() Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline QSizeF &operator+=(const QSizeF &) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QSizeF &operator-=(const QSizeF &) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QSizeF &operator*=(qreal c) Q_DECL_NOTHROW;
    inline QSizeF &operator/=(qreal c);

    friend Q_DECL_CONSTEXPR inline bool operator==(const QSizeF &, const QSizeF &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline bool operator!=(const QSizeF &, const QSizeF &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline const QSizeF operator+(const QSizeF &, const QSizeF &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline const QSizeF operator-(const QSizeF &, const QSizeF &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline const QSizeF operator*(const QSizeF &, qreal) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline const QSizeF operator*(qreal, const QSizeF &) Q_DECL_NOTHROW;
    friend inline const QSizeF operator/(const QSizeF &, qreal);

    Q_DECL_CONSTEXPR inline QSize toSize() const Q_DECL_NOTHROW;

private:
    qreal wd;
    qreal ht;
};
Q_DECLARE_TYPEINFO(QSizeF, Q_MOVABLE_TYPE);


/*****************************************************************************
  QSizeF stream functions
 *****************************************************************************/

#ifndef QT_NO_DATASTREAM
Q_CORE_EXPORT QDataStream &operator<<(QDataStream &, const QSizeF &);
Q_CORE_EXPORT QDataStream &operator>>(QDataStream &, QSizeF &);
#endif


/*****************************************************************************
  QSizeF inline functions
 *****************************************************************************/

Q_DECL_CONSTEXPR inline QSizeF::QSizeF() Q_DECL_NOTHROW : wd(-1.), ht(-1.) {}

Q_DECL_CONSTEXPR inline QSizeF::QSizeF(const QSize &sz) Q_DECL_NOTHROW : wd(sz.width()), ht(sz.height()) {}

Q_DECL_CONSTEXPR inline QSizeF::QSizeF(qreal w, qreal h) Q_DECL_NOTHROW : wd(w), ht(h) {}

inline bool QSizeF::isNull() const Q_DECL_NOTHROW
{ return qIsNull(wd) && qIsNull(ht); }

Q_DECL_CONSTEXPR inline bool QSizeF::isEmpty() const Q_DECL_NOTHROW
{ return wd <= 0. || ht <= 0.; }

Q_DECL_CONSTEXPR inline bool QSizeF::isValid() const Q_DECL_NOTHROW
{ return wd >= 0. && ht >= 0.; }

Q_DECL_CONSTEXPR inline qreal QSizeF::width() const Q_DECL_NOTHROW
{ return wd; }

Q_DECL_CONSTEXPR inline qreal QSizeF::height() const Q_DECL_NOTHROW
{ return ht; }

Q_DECL_RELAXED_CONSTEXPR inline void QSizeF::setWidth(qreal w) Q_DECL_NOTHROW
{ wd = w; }

Q_DECL_RELAXED_CONSTEXPR inline void QSizeF::setHeight(qreal h) Q_DECL_NOTHROW
{ ht = h; }

Q_DECL_CONSTEXPR inline QSizeF QSizeF::transposed() const Q_DECL_NOTHROW
{ return QSizeF(ht, wd); }

inline void QSizeF::scale(qreal w, qreal h, Qt::AspectRatioMode mode) Q_DECL_NOTHROW
{ scale(QSizeF(w, h), mode); }

inline void QSizeF::scale(const QSizeF &s, Qt::AspectRatioMode mode) Q_DECL_NOTHROW
{ *this = scaled(s, mode); }

inline QSizeF QSizeF::scaled(qreal w, qreal h, Qt::AspectRatioMode mode) const Q_DECL_NOTHROW
{ return scaled(QSizeF(w, h), mode); }

Q_DECL_RELAXED_CONSTEXPR inline qreal &QSizeF::rwidth() Q_DECL_NOTHROW
{ return wd; }

Q_DECL_RELAXED_CONSTEXPR inline qreal &QSizeF::rheight() Q_DECL_NOTHROW
{ return ht; }

Q_DECL_RELAXED_CONSTEXPR inline QSizeF &QSizeF::operator+=(const QSizeF &s) Q_DECL_NOTHROW
{ wd += s.wd; ht += s.ht; return *this; }

Q_DECL_RELAXED_CONSTEXPR inline QSizeF &QSizeF::operator-=(const QSizeF &s) Q_DECL_NOTHROW
{ wd -= s.wd; ht -= s.ht; return *this; }

Q_DECL_RELAXED_CONSTEXPR inline QSizeF &QSizeF::operator*=(qreal c) Q_DECL_NOTHROW
{ wd *= c; ht *= c; return *this; }

Q_DECL_CONSTEXPR inline bool operator==(const QSizeF &s1, const QSizeF &s2) Q_DECL_NOTHROW
{ return qFuzzyCompare(s1.wd, s2.wd) && qFuzzyCompare(s1.ht, s2.ht); }

Q_DECL_CONSTEXPR inline bool operator!=(const QSizeF &s1, const QSizeF &s2) Q_DECL_NOTHROW
{ return !qFuzzyCompare(s1.wd, s2.wd) || !qFuzzyCompare(s1.ht, s2.ht); }

Q_DECL_CONSTEXPR inline const QSizeF operator+(const QSizeF & s1, const QSizeF & s2) Q_DECL_NOTHROW
{ return QSizeF(s1.wd+s2.wd, s1.ht+s2.ht); }

Q_DECL_CONSTEXPR inline const QSizeF operator-(const QSizeF &s1, const QSizeF &s2) Q_DECL_NOTHROW
{ return QSizeF(s1.wd-s2.wd, s1.ht-s2.ht); }

Q_DECL_CONSTEXPR inline const QSizeF operator*(const QSizeF &s, qreal c) Q_DECL_NOTHROW
{ return QSizeF(s.wd*c, s.ht*c); }

Q_DECL_CONSTEXPR inline const QSizeF operator*(qreal c, const QSizeF &s) Q_DECL_NOTHROW
{ return QSizeF(s.wd*c, s.ht*c); }

inline QSizeF &QSizeF::operator/=(qreal c)
{
    Q_ASSERT(!qFuzzyIsNull(c));
    wd = wd/c; ht = ht/c;
    return *this;
}

inline const QSizeF operator/(const QSizeF &s, qreal c)
{
    Q_ASSERT(!qFuzzyIsNull(c));
    return QSizeF(s.wd/c, s.ht/c);
}

Q_DECL_CONSTEXPR inline QSizeF QSizeF::expandedTo(const QSizeF & otherSize) const Q_DECL_NOTHROW
{
    return QSizeF(qMax(wd,otherSize.wd), qMax(ht,otherSize.ht));
}

Q_DECL_CONSTEXPR inline QSizeF QSizeF::boundedTo(const QSizeF & otherSize) const Q_DECL_NOTHROW
{
    return QSizeF(qMin(wd,otherSize.wd), qMin(ht,otherSize.ht));
}

Q_DECL_CONSTEXPR inline QSize QSizeF::toSize() const Q_DECL_NOTHROW
{
    return QSize(qRound(wd), qRound(ht));
}

#ifndef QT_NO_DEBUG_STREAM
Q_CORE_EXPORT QDebug operator<<(QDebug, const QSizeF &);
#endif

QT_END_NAMESPACE

#endif // QSIZE_H
