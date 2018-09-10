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

#ifndef QRECT_H
#define QRECT_H

#include <QtCore/qmargins.h>
#include <QtCore/qsize.h>
#include <QtCore/qpoint.h>

#ifdef topLeft
#error qrect.h must be included before any header file that defines topLeft
#endif

QT_BEGIN_NAMESPACE

class Q_CORE_EXPORT QRect
{
public:
    Q_DECL_CONSTEXPR QRect() Q_DECL_NOTHROW : x1(0), y1(0), x2(-1), y2(-1) {}
    Q_DECL_CONSTEXPR QRect(const QPoint &topleft, const QPoint &bottomright) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QRect(const QPoint &topleft, const QSize &size) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QRect(int left, int top, int width, int height) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline bool isNull() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isEmpty() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isValid() const Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline int left() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int top() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int right() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int bottom() const Q_DECL_NOTHROW;
    QRect normalized() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline int x() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int y() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setLeft(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setTop(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setRight(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottom(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setX(int x) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setY(int y) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void setTopLeft(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottomRight(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setTopRight(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottomLeft(const QPoint &p) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline QPoint topLeft() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QPoint bottomRight() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QPoint topRight() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QPoint bottomLeft() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QPoint center() const Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void moveLeft(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTop(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveRight(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottom(int pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTopLeft(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottomRight(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTopRight(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottomLeft(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveCenter(const QPoint &p) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void translate(int dx, int dy) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void translate(const QPoint &p) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QRect translated(int dx, int dy) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    Q_DECL_CONSTEXPR inline QRect translated(const QPoint &p) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_RELAXED_CONSTEXPR inline void moveTo(int x, int t) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTo(const QPoint &p) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void setRect(int x, int y, int w, int h) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void getRect(int *x, int *y, int *w, int *h) const;

    Q_DECL_RELAXED_CONSTEXPR inline void setCoords(int x1, int y1, int x2, int y2) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void getCoords(int *x1, int *y1, int *x2, int *y2) const;

    Q_DECL_RELAXED_CONSTEXPR inline void adjust(int x1, int y1, int x2, int y2) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QRect adjusted(int x1, int y1, int x2, int y2) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline QSize size() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int width() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline int height() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setWidth(int w) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setHeight(int h) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setSize(const QSize &s) Q_DECL_NOTHROW;

    QRect operator|(const QRect &r) const Q_DECL_NOTHROW;
    QRect operator&(const QRect &r) const Q_DECL_NOTHROW;
    inline QRect& operator|=(const QRect &r) Q_DECL_NOTHROW;
    inline QRect& operator&=(const QRect &r) Q_DECL_NOTHROW;

    bool contains(const QRect &r, bool proper = false) const Q_DECL_NOTHROW;
    bool contains(const QPoint &p, bool proper=false) const Q_DECL_NOTHROW;
    inline bool contains(int x, int y) const Q_DECL_NOTHROW;
    inline bool contains(int x, int y, bool proper) const Q_DECL_NOTHROW;
    inline QRect united(const QRect &other) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    inline QRect intersected(const QRect &other) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    bool intersects(const QRect &r) const Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline QRect marginsAdded(const QMargins &margins) const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QRect marginsRemoved(const QMargins &margins) const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QRect &operator+=(const QMargins &margins) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QRect &operator-=(const QMargins &margins) Q_DECL_NOTHROW;

#if QT_DEPRECATED_SINCE(5, 0)
    QT_DEPRECATED QRect unite(const QRect &r) const Q_DECL_NOTHROW Q_REQUIRED_RESULT { return united(r); }
    QT_DEPRECATED QRect intersect(const QRect &r) const Q_DECL_NOTHROW Q_REQUIRED_RESULT { return intersected(r); }
#endif

    friend Q_DECL_CONSTEXPR inline bool operator==(const QRect &, const QRect &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline bool operator!=(const QRect &, const QRect &) Q_DECL_NOTHROW;

private:
    int x1;
    int y1;
    int x2;
    int y2;
};
Q_DECLARE_TYPEINFO(QRect, Q_MOVABLE_TYPE);

Q_DECL_CONSTEXPR inline bool operator==(const QRect &, const QRect &) Q_DECL_NOTHROW;
Q_DECL_CONSTEXPR inline bool operator!=(const QRect &, const QRect &) Q_DECL_NOTHROW;


/*****************************************************************************
  QRect stream functions
 *****************************************************************************/
#ifndef QT_NO_DATASTREAM
Q_CORE_EXPORT QDataStream &operator<<(QDataStream &, const QRect &);
Q_CORE_EXPORT QDataStream &operator>>(QDataStream &, QRect &);
#endif

/*****************************************************************************
  QRect inline member functions
 *****************************************************************************/

Q_DECL_CONSTEXPR inline QRect::QRect(int aleft, int atop, int awidth, int aheight) Q_DECL_NOTHROW
    : x1(aleft), y1(atop), x2(aleft + awidth - 1), y2(atop + aheight - 1) {}

Q_DECL_CONSTEXPR inline QRect::QRect(const QPoint &atopLeft, const QPoint &abottomRight) Q_DECL_NOTHROW
    : x1(atopLeft.x()), y1(atopLeft.y()), x2(abottomRight.x()), y2(abottomRight.y()) {}

Q_DECL_CONSTEXPR inline QRect::QRect(const QPoint &atopLeft, const QSize &asize) Q_DECL_NOTHROW
    : x1(atopLeft.x()), y1(atopLeft.y()), x2(atopLeft.x()+asize.width() - 1), y2(atopLeft.y()+asize.height() - 1) {}

Q_DECL_CONSTEXPR inline bool QRect::isNull() const Q_DECL_NOTHROW
{ return x2 == x1 - 1 && y2 == y1 - 1; }

Q_DECL_CONSTEXPR inline bool QRect::isEmpty() const Q_DECL_NOTHROW
{ return x1 > x2 || y1 > y2; }

Q_DECL_CONSTEXPR inline bool QRect::isValid() const Q_DECL_NOTHROW
{ return x1 <= x2 && y1 <= y2; }

Q_DECL_CONSTEXPR inline int QRect::left() const Q_DECL_NOTHROW
{ return x1; }

Q_DECL_CONSTEXPR inline int QRect::top() const Q_DECL_NOTHROW
{ return y1; }

Q_DECL_CONSTEXPR inline int QRect::right() const Q_DECL_NOTHROW
{ return x2; }

Q_DECL_CONSTEXPR inline int QRect::bottom() const Q_DECL_NOTHROW
{ return y2; }

Q_DECL_CONSTEXPR inline int QRect::x() const Q_DECL_NOTHROW
{ return x1; }

Q_DECL_CONSTEXPR inline int QRect::y() const Q_DECL_NOTHROW
{ return y1; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setLeft(int pos) Q_DECL_NOTHROW
{ x1 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setTop(int pos) Q_DECL_NOTHROW
{ y1 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setRight(int pos) Q_DECL_NOTHROW
{ x2 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setBottom(int pos) Q_DECL_NOTHROW
{ y2 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setTopLeft(const QPoint &p) Q_DECL_NOTHROW
{ x1 = p.x(); y1 = p.y(); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setBottomRight(const QPoint &p) Q_DECL_NOTHROW
{ x2 = p.x(); y2 = p.y(); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setTopRight(const QPoint &p) Q_DECL_NOTHROW
{ x2 = p.x(); y1 = p.y(); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setBottomLeft(const QPoint &p) Q_DECL_NOTHROW
{ x1 = p.x(); y2 = p.y(); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setX(int ax) Q_DECL_NOTHROW
{ x1 = ax; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setY(int ay) Q_DECL_NOTHROW
{ y1 = ay; }

Q_DECL_CONSTEXPR inline QPoint QRect::topLeft() const Q_DECL_NOTHROW
{ return QPoint(x1, y1); }

Q_DECL_CONSTEXPR inline QPoint QRect::bottomRight() const Q_DECL_NOTHROW
{ return QPoint(x2, y2); }

Q_DECL_CONSTEXPR inline QPoint QRect::topRight() const Q_DECL_NOTHROW
{ return QPoint(x2, y1); }

Q_DECL_CONSTEXPR inline QPoint QRect::bottomLeft() const Q_DECL_NOTHROW
{ return QPoint(x1, y2); }

Q_DECL_CONSTEXPR inline QPoint QRect::center() const Q_DECL_NOTHROW
{ return QPoint((x1+x2)/2, (y1+y2)/2); }

Q_DECL_CONSTEXPR inline int QRect::width() const Q_DECL_NOTHROW
{ return  x2 - x1 + 1; }

Q_DECL_CONSTEXPR inline int QRect::height() const Q_DECL_NOTHROW
{ return  y2 - y1 + 1; }

Q_DECL_CONSTEXPR inline QSize QRect::size() const Q_DECL_NOTHROW
{ return QSize(width(), height()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::translate(int dx, int dy) Q_DECL_NOTHROW
{
    x1 += dx;
    y1 += dy;
    x2 += dx;
    y2 += dy;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::translate(const QPoint &p) Q_DECL_NOTHROW
{
    x1 += p.x();
    y1 += p.y();
    x2 += p.x();
    y2 += p.y();
}

Q_DECL_CONSTEXPR inline QRect QRect::translated(int dx, int dy) const Q_DECL_NOTHROW
{ return QRect(QPoint(x1 + dx, y1 + dy), QPoint(x2 + dx, y2 + dy)); }

Q_DECL_CONSTEXPR inline QRect QRect::translated(const QPoint &p) const Q_DECL_NOTHROW
{ return QRect(QPoint(x1 + p.x(), y1 + p.y()), QPoint(x2 + p.x(), y2 + p.y())); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveTo(int ax, int ay) Q_DECL_NOTHROW
{
    x2 += ax - x1;
    y2 += ay - y1;
    x1 = ax;
    y1 = ay;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveTo(const QPoint &p) Q_DECL_NOTHROW
{
    x2 += p.x() - x1;
    y2 += p.y() - y1;
    x1 = p.x();
    y1 = p.y();
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveLeft(int pos) Q_DECL_NOTHROW
{ x2 += (pos - x1); x1 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveTop(int pos) Q_DECL_NOTHROW
{ y2 += (pos - y1); y1 = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveRight(int pos) Q_DECL_NOTHROW
{
    x1 += (pos - x2);
    x2 = pos;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveBottom(int pos) Q_DECL_NOTHROW
{
    y1 += (pos - y2);
    y2 = pos;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveTopLeft(const QPoint &p) Q_DECL_NOTHROW
{
    moveLeft(p.x());
    moveTop(p.y());
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveBottomRight(const QPoint &p) Q_DECL_NOTHROW
{
    moveRight(p.x());
    moveBottom(p.y());
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveTopRight(const QPoint &p) Q_DECL_NOTHROW
{
    moveRight(p.x());
    moveTop(p.y());
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveBottomLeft(const QPoint &p) Q_DECL_NOTHROW
{
    moveLeft(p.x());
    moveBottom(p.y());
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::moveCenter(const QPoint &p) Q_DECL_NOTHROW
{
    int w = x2 - x1;
    int h = y2 - y1;
    x1 = p.x() - w/2;
    y1 = p.y() - h/2;
    x2 = x1 + w;
    y2 = y1 + h;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::getRect(int *ax, int *ay, int *aw, int *ah) const
{
    *ax = x1;
    *ay = y1;
    *aw = x2 - x1 + 1;
    *ah = y2 - y1 + 1;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setRect(int ax, int ay, int aw, int ah) Q_DECL_NOTHROW
{
    x1 = ax;
    y1 = ay;
    x2 = (ax + aw - 1);
    y2 = (ay + ah - 1);
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::getCoords(int *xp1, int *yp1, int *xp2, int *yp2) const
{
    *xp1 = x1;
    *yp1 = y1;
    *xp2 = x2;
    *yp2 = y2;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setCoords(int xp1, int yp1, int xp2, int yp2) Q_DECL_NOTHROW
{
    x1 = xp1;
    y1 = yp1;
    x2 = xp2;
    y2 = yp2;
}

Q_DECL_CONSTEXPR inline QRect QRect::adjusted(int xp1, int yp1, int xp2, int yp2) const Q_DECL_NOTHROW
{ return QRect(QPoint(x1 + xp1, y1 + yp1), QPoint(x2 + xp2, y2 + yp2)); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::adjust(int dx1, int dy1, int dx2, int dy2) Q_DECL_NOTHROW
{
    x1 += dx1;
    y1 += dy1;
    x2 += dx2;
    y2 += dy2;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setWidth(int w) Q_DECL_NOTHROW
{ x2 = (x1 + w - 1); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setHeight(int h) Q_DECL_NOTHROW
{ y2 = (y1 + h - 1); }

Q_DECL_RELAXED_CONSTEXPR inline void QRect::setSize(const QSize &s) Q_DECL_NOTHROW
{
    x2 = (s.width()  + x1 - 1);
    y2 = (s.height() + y1 - 1);
}

inline bool QRect::contains(int ax, int ay, bool aproper) const Q_DECL_NOTHROW
{
    return contains(QPoint(ax, ay), aproper);
}

inline bool QRect::contains(int ax, int ay) const Q_DECL_NOTHROW
{
    return contains(QPoint(ax, ay), false);
}

inline QRect& QRect::operator|=(const QRect &r) Q_DECL_NOTHROW
{
    *this = *this | r;
    return *this;
}

inline QRect& QRect::operator&=(const QRect &r) Q_DECL_NOTHROW
{
    *this = *this & r;
    return *this;
}

inline QRect QRect::intersected(const QRect &other) const Q_DECL_NOTHROW
{
    return *this & other;
}

inline QRect QRect::united(const QRect &r) const Q_DECL_NOTHROW
{
    return *this | r;
}

Q_DECL_CONSTEXPR inline bool operator==(const QRect &r1, const QRect &r2) Q_DECL_NOTHROW
{
    return r1.x1==r2.x1 && r1.x2==r2.x2 && r1.y1==r2.y1 && r1.y2==r2.y2;
}

Q_DECL_CONSTEXPR inline bool operator!=(const QRect &r1, const QRect &r2) Q_DECL_NOTHROW
{
    return r1.x1!=r2.x1 || r1.x2!=r2.x2 || r1.y1!=r2.y1 || r1.y2!=r2.y2;
}

Q_DECL_CONSTEXPR inline QRect operator+(const QRect &rectangle, const QMargins &margins) Q_DECL_NOTHROW
{
    return QRect(QPoint(rectangle.left() - margins.left(), rectangle.top() - margins.top()),
                 QPoint(rectangle.right() + margins.right(), rectangle.bottom() + margins.bottom()));
}

Q_DECL_CONSTEXPR inline QRect operator+(const QMargins &margins, const QRect &rectangle) Q_DECL_NOTHROW
{
    return QRect(QPoint(rectangle.left() - margins.left(), rectangle.top() - margins.top()),
                 QPoint(rectangle.right() + margins.right(), rectangle.bottom() + margins.bottom()));
}

Q_DECL_CONSTEXPR inline QRect operator-(const QRect &lhs, const QMargins &rhs) Q_DECL_NOTHROW
{
    return QRect(QPoint(lhs.left() + rhs.left(), lhs.top() + rhs.top()),
                 QPoint(lhs.right() - rhs.right(), lhs.bottom() - rhs.bottom()));
}

Q_DECL_CONSTEXPR inline QRect QRect::marginsAdded(const QMargins &margins) const Q_DECL_NOTHROW
{
    return QRect(QPoint(x1 - margins.left(), y1 - margins.top()),
                 QPoint(x2 + margins.right(), y2 + margins.bottom()));
}

Q_DECL_CONSTEXPR inline QRect QRect::marginsRemoved(const QMargins &margins) const Q_DECL_NOTHROW
{
    return QRect(QPoint(x1 + margins.left(), y1 + margins.top()),
                 QPoint(x2 - margins.right(), y2 - margins.bottom()));
}

Q_DECL_RELAXED_CONSTEXPR inline QRect &QRect::operator+=(const QMargins &margins) Q_DECL_NOTHROW
{
    *this = marginsAdded(margins);
    return *this;
}

Q_DECL_RELAXED_CONSTEXPR inline QRect &QRect::operator-=(const QMargins &margins) Q_DECL_NOTHROW
{
    *this = marginsRemoved(margins);
    return *this;
}

#ifndef QT_NO_DEBUG_STREAM
Q_CORE_EXPORT QDebug operator<<(QDebug, const QRect &);
#endif


class Q_CORE_EXPORT QRectF
{
public:
    Q_DECL_CONSTEXPR QRectF() Q_DECL_NOTHROW : xp(0.), yp(0.), w(0.), h(0.) {}
    Q_DECL_CONSTEXPR QRectF(const QPointF &topleft, const QSizeF &size) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QRectF(const QPointF &topleft, const QPointF &bottomRight) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QRectF(qreal left, qreal top, qreal width, qreal height) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR QRectF(const QRect &rect) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline bool isNull() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isEmpty() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline bool isValid() const Q_DECL_NOTHROW;
    QRectF normalized() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline qreal left() const Q_DECL_NOTHROW { return xp; }
    Q_DECL_CONSTEXPR inline qreal top() const Q_DECL_NOTHROW { return yp; }
    Q_DECL_CONSTEXPR inline qreal right() const Q_DECL_NOTHROW { return xp + w; }
    Q_DECL_CONSTEXPR inline qreal bottom() const Q_DECL_NOTHROW { return yp + h; }

    Q_DECL_CONSTEXPR inline qreal x() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline qreal y() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setLeft(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setTop(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setRight(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottom(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setX(qreal pos) Q_DECL_NOTHROW { setLeft(pos); }
    Q_DECL_RELAXED_CONSTEXPR inline void setY(qreal pos) Q_DECL_NOTHROW { setTop(pos); }

    Q_DECL_CONSTEXPR inline QPointF topLeft() const Q_DECL_NOTHROW { return QPointF(xp, yp); }
    Q_DECL_CONSTEXPR inline QPointF bottomRight() const Q_DECL_NOTHROW { return QPointF(xp+w, yp+h); }
    Q_DECL_CONSTEXPR inline QPointF topRight() const Q_DECL_NOTHROW { return QPointF(xp+w, yp); }
    Q_DECL_CONSTEXPR inline QPointF bottomLeft() const Q_DECL_NOTHROW { return QPointF(xp, yp+h); }
    Q_DECL_CONSTEXPR inline QPointF center() const Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void setTopLeft(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottomRight(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setTopRight(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setBottomLeft(const QPointF &p) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void moveLeft(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTop(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveRight(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottom(qreal pos) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTopLeft(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottomRight(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTopRight(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveBottomLeft(const QPointF &p) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveCenter(const QPointF &p) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void translate(qreal dx, qreal dy) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void translate(const QPointF &p) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline QRectF translated(qreal dx, qreal dy) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    Q_DECL_CONSTEXPR inline QRectF translated(const QPointF &p) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_RELAXED_CONSTEXPR inline void moveTo(qreal x, qreal y) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void moveTo(const QPointF &p) Q_DECL_NOTHROW;

    Q_DECL_RELAXED_CONSTEXPR inline void setRect(qreal x, qreal y, qreal w, qreal h) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void getRect(qreal *x, qreal *y, qreal *w, qreal *h) const;

    Q_DECL_RELAXED_CONSTEXPR inline void setCoords(qreal x1, qreal y1, qreal x2, qreal y2) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void getCoords(qreal *x1, qreal *y1, qreal *x2, qreal *y2) const;

    Q_DECL_RELAXED_CONSTEXPR inline void adjust(qreal x1, qreal y1, qreal x2, qreal y2) Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QRectF adjusted(qreal x1, qreal y1, qreal x2, qreal y2) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

    Q_DECL_CONSTEXPR inline QSizeF size() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline qreal width() const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline qreal height() const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setWidth(qreal w) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setHeight(qreal h) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline void setSize(const QSizeF &s) Q_DECL_NOTHROW;

    QRectF operator|(const QRectF &r) const Q_DECL_NOTHROW;
    QRectF operator&(const QRectF &r) const Q_DECL_NOTHROW;
    inline QRectF& operator|=(const QRectF &r) Q_DECL_NOTHROW;
    inline QRectF& operator&=(const QRectF &r) Q_DECL_NOTHROW;

    bool contains(const QRectF &r) const Q_DECL_NOTHROW;
    bool contains(const QPointF &p) const Q_DECL_NOTHROW;
    inline bool contains(qreal x, qreal y) const Q_DECL_NOTHROW;
    inline QRectF united(const QRectF &other) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    inline QRectF intersected(const QRectF &other) const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    bool intersects(const QRectF &r) const Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline QRectF marginsAdded(const QMarginsF &margins) const Q_DECL_NOTHROW;
    Q_DECL_CONSTEXPR inline QRectF marginsRemoved(const QMarginsF &margins) const Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QRectF &operator+=(const QMarginsF &margins) Q_DECL_NOTHROW;
    Q_DECL_RELAXED_CONSTEXPR inline QRectF &operator-=(const QMarginsF &margins) Q_DECL_NOTHROW;

#if QT_DEPRECATED_SINCE(5, 0)
    QT_DEPRECATED QRectF unite(const QRectF &r) const Q_DECL_NOTHROW Q_REQUIRED_RESULT { return united(r); }
    QT_DEPRECATED QRectF intersect(const QRectF &r) const Q_DECL_NOTHROW Q_REQUIRED_RESULT { return intersected(r); }
#endif

    friend Q_DECL_CONSTEXPR inline bool operator==(const QRectF &, const QRectF &) Q_DECL_NOTHROW;
    friend Q_DECL_CONSTEXPR inline bool operator!=(const QRectF &, const QRectF &) Q_DECL_NOTHROW;

    Q_DECL_CONSTEXPR inline QRect toRect() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;
    QRect toAlignedRect() const Q_DECL_NOTHROW Q_REQUIRED_RESULT;

private:
    qreal xp;
    qreal yp;
    qreal w;
    qreal h;
};
Q_DECLARE_TYPEINFO(QRectF, Q_MOVABLE_TYPE);

Q_DECL_CONSTEXPR inline bool operator==(const QRectF &, const QRectF &) Q_DECL_NOTHROW;
Q_DECL_CONSTEXPR inline bool operator!=(const QRectF &, const QRectF &) Q_DECL_NOTHROW;


/*****************************************************************************
  QRectF stream functions
 *****************************************************************************/
#ifndef QT_NO_DATASTREAM
Q_CORE_EXPORT QDataStream &operator<<(QDataStream &, const QRectF &);
Q_CORE_EXPORT QDataStream &operator>>(QDataStream &, QRectF &);
#endif

/*****************************************************************************
  QRectF inline member functions
 *****************************************************************************/

Q_DECL_CONSTEXPR inline QRectF::QRectF(qreal aleft, qreal atop, qreal awidth, qreal aheight) Q_DECL_NOTHROW
    : xp(aleft), yp(atop), w(awidth), h(aheight)
{
}

Q_DECL_CONSTEXPR inline QRectF::QRectF(const QPointF &atopLeft, const QSizeF &asize) Q_DECL_NOTHROW
    : xp(atopLeft.x()), yp(atopLeft.y()), w(asize.width()), h(asize.height())
{
}


Q_DECL_CONSTEXPR inline QRectF::QRectF(const QPointF &atopLeft, const QPointF &abottomRight) Q_DECL_NOTHROW
    : xp(atopLeft.x()), yp(atopLeft.y()), w(abottomRight.x() - atopLeft.x()), h(abottomRight.y() - atopLeft.y())
{
}

Q_DECL_CONSTEXPR inline QRectF::QRectF(const QRect &r) Q_DECL_NOTHROW
    : xp(r.x()), yp(r.y()), w(r.width()), h(r.height())
{
}

Q_DECL_CONSTEXPR inline bool QRectF::isNull() const Q_DECL_NOTHROW
{ return w == 0. && h == 0.; }

Q_DECL_CONSTEXPR inline bool QRectF::isEmpty() const Q_DECL_NOTHROW
{ return w <= 0. || h <= 0.; }

Q_DECL_CONSTEXPR inline bool QRectF::isValid() const Q_DECL_NOTHROW
{ return w > 0. && h > 0.; }

Q_DECL_CONSTEXPR inline qreal QRectF::x() const Q_DECL_NOTHROW
{ return xp; }

Q_DECL_CONSTEXPR inline qreal QRectF::y() const Q_DECL_NOTHROW
{ return yp; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setLeft(qreal pos) Q_DECL_NOTHROW
{ qreal diff = pos - xp; xp += diff; w -= diff; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setRight(qreal pos) Q_DECL_NOTHROW
{ w = pos - xp; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setTop(qreal pos) Q_DECL_NOTHROW
{ qreal diff = pos - yp; yp += diff; h -= diff; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setBottom(qreal pos) Q_DECL_NOTHROW
{ h = pos - yp; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setTopLeft(const QPointF &p) Q_DECL_NOTHROW
{ setLeft(p.x()); setTop(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setTopRight(const QPointF &p) Q_DECL_NOTHROW
{ setRight(p.x()); setTop(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setBottomLeft(const QPointF &p) Q_DECL_NOTHROW
{ setLeft(p.x()); setBottom(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setBottomRight(const QPointF &p) Q_DECL_NOTHROW
{ setRight(p.x()); setBottom(p.y()); }

Q_DECL_CONSTEXPR inline QPointF QRectF::center() const Q_DECL_NOTHROW
{ return QPointF(xp + w/2, yp + h/2); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveLeft(qreal pos) Q_DECL_NOTHROW
{ xp = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveTop(qreal pos) Q_DECL_NOTHROW
{ yp = pos; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveRight(qreal pos) Q_DECL_NOTHROW
{ xp = pos - w; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveBottom(qreal pos) Q_DECL_NOTHROW
{ yp = pos - h; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveTopLeft(const QPointF &p) Q_DECL_NOTHROW
{ moveLeft(p.x()); moveTop(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveTopRight(const QPointF &p) Q_DECL_NOTHROW
{ moveRight(p.x()); moveTop(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveBottomLeft(const QPointF &p) Q_DECL_NOTHROW
{ moveLeft(p.x()); moveBottom(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveBottomRight(const QPointF &p) Q_DECL_NOTHROW
{ moveRight(p.x()); moveBottom(p.y()); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveCenter(const QPointF &p) Q_DECL_NOTHROW
{ xp = p.x() - w/2; yp = p.y() - h/2; }

Q_DECL_CONSTEXPR inline qreal QRectF::width() const Q_DECL_NOTHROW
{ return w; }

Q_DECL_CONSTEXPR inline qreal QRectF::height() const Q_DECL_NOTHROW
{ return h; }

Q_DECL_CONSTEXPR inline QSizeF QRectF::size() const Q_DECL_NOTHROW
{ return QSizeF(w, h); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::translate(qreal dx, qreal dy) Q_DECL_NOTHROW
{
    xp += dx;
    yp += dy;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::translate(const QPointF &p) Q_DECL_NOTHROW
{
    xp += p.x();
    yp += p.y();
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveTo(qreal ax, qreal ay) Q_DECL_NOTHROW
{
    xp = ax;
    yp = ay;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::moveTo(const QPointF &p) Q_DECL_NOTHROW
{
    xp = p.x();
    yp = p.y();
}

Q_DECL_CONSTEXPR inline QRectF QRectF::translated(qreal dx, qreal dy) const Q_DECL_NOTHROW
{ return QRectF(xp + dx, yp + dy, w, h); }

Q_DECL_CONSTEXPR inline QRectF QRectF::translated(const QPointF &p) const Q_DECL_NOTHROW
{ return QRectF(xp + p.x(), yp + p.y(), w, h); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::getRect(qreal *ax, qreal *ay, qreal *aaw, qreal *aah) const
{
    *ax = this->xp;
    *ay = this->yp;
    *aaw = this->w;
    *aah = this->h;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setRect(qreal ax, qreal ay, qreal aaw, qreal aah) Q_DECL_NOTHROW
{
    this->xp = ax;
    this->yp = ay;
    this->w = aaw;
    this->h = aah;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::getCoords(qreal *xp1, qreal *yp1, qreal *xp2, qreal *yp2) const
{
    *xp1 = xp;
    *yp1 = yp;
    *xp2 = xp + w;
    *yp2 = yp + h;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setCoords(qreal xp1, qreal yp1, qreal xp2, qreal yp2) Q_DECL_NOTHROW
{
    xp = xp1;
    yp = yp1;
    w = xp2 - xp1;
    h = yp2 - yp1;
}

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::adjust(qreal xp1, qreal yp1, qreal xp2, qreal yp2) Q_DECL_NOTHROW
{ xp += xp1; yp += yp1; w += xp2 - xp1; h += yp2 - yp1; }

Q_DECL_CONSTEXPR inline QRectF QRectF::adjusted(qreal xp1, qreal yp1, qreal xp2, qreal yp2) const Q_DECL_NOTHROW
{ return QRectF(xp + xp1, yp + yp1, w + xp2 - xp1, h + yp2 - yp1); }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setWidth(qreal aw) Q_DECL_NOTHROW
{ this->w = aw; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setHeight(qreal ah) Q_DECL_NOTHROW
{ this->h = ah; }

Q_DECL_RELAXED_CONSTEXPR inline void QRectF::setSize(const QSizeF &s) Q_DECL_NOTHROW
{
    w = s.width();
    h = s.height();
}

inline bool QRectF::contains(qreal ax, qreal ay) const Q_DECL_NOTHROW
{
    return contains(QPointF(ax, ay));
}

inline QRectF& QRectF::operator|=(const QRectF &r) Q_DECL_NOTHROW
{
    *this = *this | r;
    return *this;
}

inline QRectF& QRectF::operator&=(const QRectF &r) Q_DECL_NOTHROW
{
    *this = *this & r;
    return *this;
}

inline QRectF QRectF::intersected(const QRectF &r) const Q_DECL_NOTHROW
{
    return *this & r;
}

inline QRectF QRectF::united(const QRectF &r) const Q_DECL_NOTHROW
{
    return *this | r;
}

Q_DECL_CONSTEXPR inline bool operator==(const QRectF &r1, const QRectF &r2) Q_DECL_NOTHROW
{
    return qFuzzyCompare(r1.xp, r2.xp) && qFuzzyCompare(r1.yp, r2.yp)
           && qFuzzyCompare(r1.w, r2.w) && qFuzzyCompare(r1.h, r2.h);
}

Q_DECL_CONSTEXPR inline bool operator!=(const QRectF &r1, const QRectF &r2) Q_DECL_NOTHROW
{
    return !qFuzzyCompare(r1.xp, r2.xp) || !qFuzzyCompare(r1.yp, r2.yp)
           || !qFuzzyCompare(r1.w, r2.w) || !qFuzzyCompare(r1.h, r2.h);
}

Q_DECL_CONSTEXPR inline QRect QRectF::toRect() const Q_DECL_NOTHROW
{
    return QRect(qRound(xp), qRound(yp), qRound(w), qRound(h));
}

Q_DECL_CONSTEXPR inline QRectF operator+(const QRectF &lhs, const QMarginsF &rhs) Q_DECL_NOTHROW
{
    return QRectF(QPointF(lhs.left() - rhs.left(), lhs.top() - rhs.top()),
                  QSizeF(lhs.width() + rhs.left() + rhs.right(), lhs.height() + rhs.top() + rhs.bottom()));
}

Q_DECL_CONSTEXPR inline QRectF operator+(const QMarginsF &lhs, const QRectF &rhs) Q_DECL_NOTHROW
{
    return QRectF(QPointF(rhs.left() - lhs.left(), rhs.top() - lhs.top()),
                  QSizeF(rhs.width() + lhs.left() + lhs.right(), rhs.height() + lhs.top() + lhs.bottom()));
}

Q_DECL_CONSTEXPR inline QRectF operator-(const QRectF &lhs, const QMarginsF &rhs) Q_DECL_NOTHROW
{
    return QRectF(QPointF(lhs.left() + rhs.left(), lhs.top() + rhs.top()),
                  QSizeF(lhs.width() - rhs.left() - rhs.right(), lhs.height() - rhs.top() - rhs.bottom()));
}

Q_DECL_CONSTEXPR inline QRectF QRectF::marginsAdded(const QMarginsF &margins) const Q_DECL_NOTHROW
{
    return QRectF(QPointF(xp - margins.left(), yp - margins.top()),
                  QSizeF(w + margins.left() + margins.right(), h + margins.top() + margins.bottom()));
}

Q_DECL_CONSTEXPR inline QRectF QRectF::marginsRemoved(const QMarginsF &margins) const Q_DECL_NOTHROW
{
    return QRectF(QPointF(xp + margins.left(), yp + margins.top()),
                  QSizeF(w - margins.left() - margins.right(), h - margins.top() - margins.bottom()));
}

Q_DECL_RELAXED_CONSTEXPR inline QRectF &QRectF::operator+=(const QMarginsF &margins) Q_DECL_NOTHROW
{
    *this = marginsAdded(margins);
    return *this;
}

Q_DECL_RELAXED_CONSTEXPR inline QRectF &QRectF::operator-=(const QMarginsF &margins) Q_DECL_NOTHROW
{
    *this = marginsRemoved(margins);
    return *this;
}

#ifndef QT_NO_DEBUG_STREAM
Q_CORE_EXPORT QDebug operator<<(QDebug, const QRectF &);
#endif

QT_END_NAMESPACE

#endif // QRECT_H
