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

#ifndef QGEOSHAPE_H
#define QGEOSHAPE_H

#include <QtCore/QSharedDataPointer>
#include <QtPositioning/QGeoCoordinate>

QT_BEGIN_NAMESPACE

class QDebug;
class QGeoShapePrivate;

class Q_POSITIONING_EXPORT QGeoShape
{
    Q_GADGET
    Q_PROPERTY(ShapeType type READ type)
    Q_PROPERTY(bool isValid READ isValid)
    Q_PROPERTY(bool isEmpty READ isEmpty)
    Q_ENUMS(ShapeType)

public:
    QGeoShape();
    QGeoShape(const QGeoShape &other);
    ~QGeoShape();

    enum ShapeType {
        UnknownType,
        RectangleType,
        CircleType
    };

    ShapeType type() const;

    bool isValid() const;
    bool isEmpty() const;
    Q_INVOKABLE bool contains(const QGeoCoordinate &coordinate) const;

    QGeoCoordinate center() const;

    void extendShape(const QGeoCoordinate &coordinate);

    bool operator==(const QGeoShape &other) const;
    bool operator!=(const QGeoShape &other) const;

    QGeoShape &operator=(const QGeoShape &other);

    Q_INVOKABLE QString toString() const;
protected:
    QGeoShape(QGeoShapePrivate *d);

    QSharedDataPointer<QGeoShapePrivate> d_ptr;

private:
    inline QGeoShapePrivate *d_func();
    inline const QGeoShapePrivate *d_func() const;
};

Q_DECLARE_TYPEINFO(QGeoShape, Q_MOVABLE_TYPE);

#ifndef QT_NO_DEBUG_STREAM
Q_POSITIONING_EXPORT QDebug operator<<(QDebug, const QGeoShape &);
#endif

#ifndef QT_NO_DATASTREAM
Q_POSITIONING_EXPORT QDataStream &operator<<(QDataStream &stream, const QGeoShape &shape);
Q_POSITIONING_EXPORT QDataStream &operator>>(QDataStream &stream, QGeoShape &shape);
#endif

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QGeoShape)

#endif

