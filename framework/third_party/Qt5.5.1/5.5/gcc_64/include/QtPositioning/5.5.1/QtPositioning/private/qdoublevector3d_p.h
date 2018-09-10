/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtPositioning module of the Qt Toolkit.
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

#ifndef QDOUBLEVECTOR3D_P_H
#define QDOUBLEVECTOR3D_P_H

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

#ifdef QT_BUILD_LOCATION_LIB
#include <QVector3D>
#endif

#include "qpositioningglobal.h"
#include "qdoublevector2d_p.h"
#include <QtCore/qmetatype.h>

QT_BEGIN_NAMESPACE

class Q_POSITIONING_EXPORT QDoubleVector3D
{
public:
    Q_DECL_CONSTEXPR inline QDoubleVector3D();
    Q_DECL_CONSTEXPR inline QDoubleVector3D(double xpos, double ypos, double zpos);
    Q_DECL_CONSTEXPR inline QDoubleVector3D(const QDoubleVector2D &vector);
    Q_DECL_CONSTEXPR inline QDoubleVector3D(const QDoubleVector2D &vector, double zpos);

    inline bool isNull() const;

    Q_DECL_CONSTEXPR inline double x() const;
    Q_DECL_CONSTEXPR inline double y() const;
    Q_DECL_CONSTEXPR inline double z() const;

    inline void setX(double x);
    inline void setY(double y);
    inline void setZ(double z);

    inline double get(int i) const;
    inline void set(int i, double value);

    double length() const;
    Q_DECL_CONSTEXPR inline double lengthSquared() const;

    QDoubleVector3D normalized() const;
    void normalize();

    inline QDoubleVector3D &operator+=(const QDoubleVector3D &vector);
    inline QDoubleVector3D &operator-=(const QDoubleVector3D &vector);
    inline QDoubleVector3D &operator*=(double factor);
    inline QDoubleVector3D &operator*=(const QDoubleVector3D &vector);
    inline QDoubleVector3D &operator/=(double divisor);

    Q_DECL_CONSTEXPR static inline double dotProduct(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
    { return v1.xp * v2.xp + v1.yp * v2.yp + v1.zp * v2.zp; }

    Q_DECL_CONSTEXPR static inline QDoubleVector3D crossProduct(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
    { return QDoubleVector3D(v1.yp * v2.zp - v1.zp * v2.yp,
                    v1.zp * v2.xp - v1.xp * v2.zp,
                    v1.xp * v2.yp - v1.yp * v2.xp); }

    static QDoubleVector3D normal(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    static QDoubleVector3D normal
        (const QDoubleVector3D &v1, const QDoubleVector3D &v2, const QDoubleVector3D &v3);

    double distanceToPlane(const QDoubleVector3D &plane, const QDoubleVector3D &normal) const;
    double distanceToPlane(const QDoubleVector3D &plane1, const QDoubleVector3D &plane2, const QDoubleVector3D &plane3) const;
    double distanceToLine(const QDoubleVector3D &point, const QDoubleVector3D &direction) const;

    friend Q_DECL_CONSTEXPR inline bool operator==(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    friend Q_DECL_CONSTEXPR inline bool operator!=(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator+(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator-(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(double factor, const QDoubleVector3D &vector);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(const QDoubleVector3D &vector, double factor);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(const QDoubleVector3D &v1, const QDoubleVector3D &v2);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator-(const QDoubleVector3D &vector);
    friend Q_DECL_CONSTEXPR inline const QDoubleVector3D operator/(const QDoubleVector3D &vector, double divisor);

    friend Q_DECL_CONSTEXPR inline bool qFuzzyCompare(const QDoubleVector3D &v1, const QDoubleVector3D &v2);

    Q_DECL_CONSTEXPR inline QDoubleVector2D toVector2D() const;

private:
    double xp, yp, zp;

    friend class QDoubleVector2D;
};

Q_DECLARE_TYPEINFO(QDoubleVector3D, Q_MOVABLE_TYPE);

Q_DECL_CONSTEXPR inline QDoubleVector3D::QDoubleVector3D() : xp(0.0), yp(0.0), zp(0.0) {}

Q_DECL_CONSTEXPR inline QDoubleVector3D::QDoubleVector3D(double xpos, double ypos, double zpos) : xp(xpos), yp(ypos), zp(zpos) {}

Q_DECL_CONSTEXPR inline QDoubleVector3D::QDoubleVector3D(const QDoubleVector2D &v)
    : xp(v.xp), yp(v.yp), zp(0.0) {}

Q_DECL_CONSTEXPR inline QDoubleVector3D::QDoubleVector3D(const QDoubleVector2D &v, double zpos)
    : xp(v.xp), yp(v.yp), zp(zpos) {}

inline bool QDoubleVector3D::isNull() const
{
    return qIsNull(xp) && qIsNull(yp) && qIsNull(zp);
}

Q_DECL_CONSTEXPR inline double QDoubleVector3D::x() const { return xp; }
Q_DECL_CONSTEXPR inline double QDoubleVector3D::y() const { return yp; }
Q_DECL_CONSTEXPR inline double QDoubleVector3D::z() const { return zp; }

Q_DECL_CONSTEXPR inline double QDoubleVector3D::lengthSquared() const
{ return xp * xp + yp * yp + zp * zp; }


inline void QDoubleVector3D::setX(double aX) { xp = aX; }
inline void QDoubleVector3D::setY(double aY) { yp = aY; }
inline void QDoubleVector3D::setZ(double aZ) { zp = aZ; }

inline double QDoubleVector3D::get(int i) const
{
    switch (i) {
    case 0:
        return xp;
    case 1:
        return yp;
    case 2:
        return zp;
    default:
        return 0.0;
    }
}

inline void QDoubleVector3D::set(int i, double value)
{
    switch (i) {
    case 0:
        xp = value;
        break;
    case 1:
        yp = value;
        break;
    case 2:
        zp = value;
        break;
    default:
        break;
    }
}

inline QDoubleVector3D &QDoubleVector3D::operator+=(const QDoubleVector3D &vector)
{
    xp += vector.xp;
    yp += vector.yp;
    zp += vector.zp;
    return *this;
}

inline QDoubleVector3D &QDoubleVector3D::operator-=(const QDoubleVector3D &vector)
{
    xp -= vector.xp;
    yp -= vector.yp;
    zp -= vector.zp;
    return *this;
}

inline QDoubleVector3D &QDoubleVector3D::operator*=(double factor)
{
    xp *= factor;
    yp *= factor;
    zp *= factor;
    return *this;
}

inline QDoubleVector3D &QDoubleVector3D::operator*=(const QDoubleVector3D &vector)
{
    xp *= vector.xp;
    yp *= vector.yp;
    zp *= vector.zp;
    return *this;
}

inline QDoubleVector3D &QDoubleVector3D::operator/=(double divisor)
{
    xp /= divisor;
    yp /= divisor;
    zp /= divisor;
    return *this;
}

Q_DECL_CONSTEXPR inline bool operator==(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return v1.xp == v2.xp && v1.yp == v2.yp && v1.zp == v2.zp;
}

Q_DECL_CONSTEXPR inline bool operator!=(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return v1.xp != v2.xp || v1.yp != v2.yp || v1.zp != v2.zp;
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator+(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return QDoubleVector3D(v1.xp + v2.xp, v1.yp + v2.yp, v1.zp + v2.zp);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator-(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return QDoubleVector3D(v1.xp - v2.xp, v1.yp - v2.yp, v1.zp - v2.zp);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(double factor, const QDoubleVector3D &vector)
{
    return QDoubleVector3D(vector.xp * factor, vector.yp * factor, vector.zp * factor);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(const QDoubleVector3D &vector, double factor)
{
    return QDoubleVector3D(vector.xp * factor, vector.yp * factor, vector.zp * factor);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator*(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return QDoubleVector3D(v1.xp * v2.xp, v1.yp * v2.yp, v1.zp * v2.zp);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator-(const QDoubleVector3D &vector)
{
    return QDoubleVector3D(-vector.xp, -vector.yp, -vector.zp);
}

Q_DECL_CONSTEXPR inline const QDoubleVector3D operator/(const QDoubleVector3D &vector, double divisor)
{
    return QDoubleVector3D(vector.xp / divisor, vector.yp / divisor, vector.zp / divisor);
}

Q_DECL_CONSTEXPR inline bool qFuzzyCompare(const QDoubleVector3D &v1, const QDoubleVector3D &v2)
{
    return qFuzzyCompare(v1.xp, v2.xp) &&
           qFuzzyCompare(v1.yp, v2.yp) &&
           qFuzzyCompare(v1.zp, v2.zp);
}

Q_DECL_CONSTEXPR inline QDoubleVector2D QDoubleVector3D::toVector2D() const
{
    return QDoubleVector2D(xp, yp);
}


#ifndef QT_NO_DEBUG_STREAM
Q_POSITIONING_EXPORT QDebug operator<<(QDebug dbg, const QDoubleVector3D &vector);
#endif

#ifndef QT_NO_DATASTREAM
Q_POSITIONING_EXPORT QDataStream &operator<<(QDataStream &, const QDoubleVector3D &);
Q_POSITIONING_EXPORT QDataStream &operator>>(QDataStream &, QDoubleVector3D &);
#endif

QT_END_NAMESPACE

#endif
