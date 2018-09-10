/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKPATH_H
#define QQUICKPATH_H

#include <qqml.h>

#include <private/qqmlnullablevalue_p.h>
#include <private/qbezier_p.h>

#include <QtCore/QObject>
#include <QtGui/QPainterPath>

QT_BEGIN_NAMESPACE

class QQuickCurve;
struct QQuickPathData
{
    int index;
    QPointF endPoint;
    QList<QQuickCurve*> curves;
};

class Q_AUTOTEST_EXPORT QQuickPathElement : public QObject
{
    Q_OBJECT
public:
    QQuickPathElement(QObject *parent=0) : QObject(parent) {}
Q_SIGNALS:
    void changed();
};

class Q_AUTOTEST_EXPORT QQuickPathAttribute : public QQuickPathElement
{
    Q_OBJECT

    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(qreal value READ value WRITE setValue NOTIFY valueChanged)
public:
    QQuickPathAttribute(QObject *parent=0) : QQuickPathElement(parent), _value(0) {}


    QString name() const;
    void setName(const QString &name);

    qreal value() const;
    void setValue(qreal value);

Q_SIGNALS:
    void nameChanged();
    void valueChanged();

private:
    QString _name;
    qreal _value;
};

class Q_AUTOTEST_EXPORT QQuickCurve : public QQuickPathElement
{
    Q_OBJECT

    Q_PROPERTY(qreal x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y WRITE setY NOTIFY yChanged)
    Q_PROPERTY(qreal relativeX READ relativeX WRITE setRelativeX NOTIFY relativeXChanged)
    Q_PROPERTY(qreal relativeY READ relativeY WRITE setRelativeY NOTIFY relativeYChanged)
public:
    QQuickCurve(QObject *parent=0) : QQuickPathElement(parent) {}

    qreal x() const;
    void setX(qreal x);
    bool hasX();

    qreal y() const;
    void setY(qreal y);
    bool hasY();

    qreal relativeX() const;
    void setRelativeX(qreal x);
    bool hasRelativeX();

    qreal relativeY() const;
    void setRelativeY(qreal y);
    bool hasRelativeY();

    virtual void addToPath(QPainterPath &, const QQuickPathData &) {}

Q_SIGNALS:
    void xChanged();
    void yChanged();
    void relativeXChanged();
    void relativeYChanged();

private:
    QQmlNullableValue<qreal> _x;
    QQmlNullableValue<qreal> _y;
    QQmlNullableValue<qreal> _relativeX;
    QQmlNullableValue<qreal> _relativeY;
};

class Q_AUTOTEST_EXPORT QQuickPathLine : public QQuickCurve
{
    Q_OBJECT
public:
    QQuickPathLine(QObject *parent=0) : QQuickCurve(parent) {}

    void addToPath(QPainterPath &path, const QQuickPathData &);
};

class Q_AUTOTEST_EXPORT QQuickPathQuad : public QQuickCurve
{
    Q_OBJECT

    Q_PROPERTY(qreal controlX READ controlX WRITE setControlX NOTIFY controlXChanged)
    Q_PROPERTY(qreal controlY READ controlY WRITE setControlY NOTIFY controlYChanged)
    Q_PROPERTY(qreal relativeControlX READ relativeControlX WRITE setRelativeControlX NOTIFY relativeControlXChanged)
    Q_PROPERTY(qreal relativeControlY READ relativeControlY WRITE setRelativeControlY NOTIFY relativeControlYChanged)
public:
    QQuickPathQuad(QObject *parent=0) : QQuickCurve(parent), _controlX(0), _controlY(0) {}

    qreal controlX() const;
    void setControlX(qreal x);

    qreal controlY() const;
    void setControlY(qreal y);

    qreal relativeControlX() const;
    void setRelativeControlX(qreal x);
    bool hasRelativeControlX();

    qreal relativeControlY() const;
    void setRelativeControlY(qreal y);
    bool hasRelativeControlY();

    void addToPath(QPainterPath &path, const QQuickPathData &);

Q_SIGNALS:
    void controlXChanged();
    void controlYChanged();
    void relativeControlXChanged();
    void relativeControlYChanged();

private:
    qreal _controlX;
    qreal _controlY;
    QQmlNullableValue<qreal> _relativeControlX;
    QQmlNullableValue<qreal> _relativeControlY;
};

class Q_AUTOTEST_EXPORT QQuickPathCubic : public QQuickCurve
{
    Q_OBJECT

    Q_PROPERTY(qreal control1X READ control1X WRITE setControl1X NOTIFY control1XChanged)
    Q_PROPERTY(qreal control1Y READ control1Y WRITE setControl1Y NOTIFY control1YChanged)
    Q_PROPERTY(qreal control2X READ control2X WRITE setControl2X NOTIFY control2XChanged)
    Q_PROPERTY(qreal control2Y READ control2Y WRITE setControl2Y NOTIFY control2YChanged)
    Q_PROPERTY(qreal relativeControl1X READ relativeControl1X WRITE setRelativeControl1X NOTIFY relativeControl1XChanged)
    Q_PROPERTY(qreal relativeControl1Y READ relativeControl1Y WRITE setRelativeControl1Y NOTIFY relativeControl1YChanged)
    Q_PROPERTY(qreal relativeControl2X READ relativeControl2X WRITE setRelativeControl2X NOTIFY relativeControl2XChanged)
    Q_PROPERTY(qreal relativeControl2Y READ relativeControl2Y WRITE setRelativeControl2Y NOTIFY relativeControl2YChanged)
public:
    QQuickPathCubic(QObject *parent=0) : QQuickCurve(parent), _control1X(0), _control1Y(0), _control2X(0), _control2Y(0) {}

    qreal control1X() const;
    void setControl1X(qreal x);

    qreal control1Y() const;
    void setControl1Y(qreal y);

    qreal control2X() const;
    void setControl2X(qreal x);

    qreal control2Y() const;
    void setControl2Y(qreal y);

    qreal relativeControl1X() const;
    void setRelativeControl1X(qreal x);
    bool hasRelativeControl1X();

    qreal relativeControl1Y() const;
    void setRelativeControl1Y(qreal y);
    bool hasRelativeControl1Y();

    qreal relativeControl2X() const;
    void setRelativeControl2X(qreal x);
    bool hasRelativeControl2X();

    qreal relativeControl2Y() const;
    void setRelativeControl2Y(qreal y);
    bool hasRelativeControl2Y();

    void addToPath(QPainterPath &path, const QQuickPathData &);

Q_SIGNALS:
    void control1XChanged();
    void control1YChanged();
    void control2XChanged();
    void control2YChanged();
    void relativeControl1XChanged();
    void relativeControl1YChanged();
    void relativeControl2XChanged();
    void relativeControl2YChanged();

private:
    qreal _control1X;
    qreal _control1Y;
    qreal _control2X;
    qreal _control2Y;
    QQmlNullableValue<qreal> _relativeControl1X;
    QQmlNullableValue<qreal> _relativeControl1Y;
    QQmlNullableValue<qreal> _relativeControl2X;
    QQmlNullableValue<qreal> _relativeControl2Y;
};

class Q_AUTOTEST_EXPORT QQuickPathCatmullRomCurve : public QQuickCurve
{
    Q_OBJECT
public:
    QQuickPathCatmullRomCurve(QObject *parent=0) : QQuickCurve(parent) {}

    void addToPath(QPainterPath &path, const QQuickPathData &);
};

class Q_AUTOTEST_EXPORT QQuickPathArc : public QQuickCurve
{
    Q_OBJECT
    Q_PROPERTY(qreal radiusX READ radiusX WRITE setRadiusX NOTIFY radiusXChanged)
    Q_PROPERTY(qreal radiusY READ radiusY WRITE setRadiusY NOTIFY radiusYChanged)
    Q_PROPERTY(bool useLargeArc READ useLargeArc WRITE setUseLargeArc NOTIFY useLargeArcChanged)
    Q_PROPERTY(ArcDirection direction READ direction WRITE setDirection NOTIFY directionChanged)

public:
    QQuickPathArc(QObject *parent=0)
        : QQuickCurve(parent), _radiusX(0), _radiusY(0), _useLargeArc(false), _direction(Clockwise) {}

    enum ArcDirection { Clockwise, Counterclockwise };
    Q_ENUMS(ArcDirection)

    qreal radiusX() const;
    void setRadiusX(qreal);

    qreal radiusY() const;
    void setRadiusY(qreal);

    bool useLargeArc() const;
    void setUseLargeArc(bool);

    ArcDirection direction() const;
    void setDirection(ArcDirection direction);

    void addToPath(QPainterPath &path, const QQuickPathData &);

Q_SIGNALS:
    void radiusXChanged();
    void radiusYChanged();
    void useLargeArcChanged();
    void directionChanged();

private:
    qreal _radiusX;
    qreal _radiusY;
    bool _useLargeArc;
    ArcDirection _direction;
};

class Q_AUTOTEST_EXPORT QQuickPathSvg : public QQuickCurve
{
    Q_OBJECT
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged)
public:
    QQuickPathSvg(QObject *parent=0) : QQuickCurve(parent) {}

    QString path() const;
    void setPath(const QString &path);

    void addToPath(QPainterPath &path, const QQuickPathData &);

Q_SIGNALS:
    void pathChanged();

private:
    QString _path;
};

class Q_AUTOTEST_EXPORT QQuickPathPercent : public QQuickPathElement
{
    Q_OBJECT
    Q_PROPERTY(qreal value READ value WRITE setValue NOTIFY valueChanged)
public:
    QQuickPathPercent(QObject *parent=0) : QQuickPathElement(parent) {}

    qreal value() const;
    void setValue(qreal value);

Q_SIGNALS:
    void valueChanged();

private:
    qreal _value;
};

struct QQuickCachedBezier
{
    QQuickCachedBezier() : isValid(false) {}
    QBezier bezier;
    int element;
    qreal bezLength;
    qreal currLength;
    qreal p;
    bool isValid;
};

class QQuickPathPrivate;
class Q_AUTOTEST_EXPORT QQuickPath : public QObject, public QQmlParserStatus
{
    Q_OBJECT

    Q_INTERFACES(QQmlParserStatus)
    Q_PROPERTY(QQmlListProperty<QQuickPathElement> pathElements READ pathElements)
    Q_PROPERTY(qreal startX READ startX WRITE setStartX NOTIFY startXChanged)
    Q_PROPERTY(qreal startY READ startY WRITE setStartY NOTIFY startYChanged)
    Q_PROPERTY(bool closed READ isClosed NOTIFY changed)
    Q_CLASSINFO("DefaultProperty", "pathElements")
    Q_INTERFACES(QQmlParserStatus)
public:
    QQuickPath(QObject *parent=0);
    ~QQuickPath();

    QQmlListProperty<QQuickPathElement> pathElements();

    qreal startX() const;
    void setStartX(qreal x);
    bool hasStartX() const;

    qreal startY() const;
    void setStartY(qreal y);
    bool hasStartY() const;

    bool isClosed() const;
    bool hasEnd() const;

    QPainterPath path() const;
    QStringList attributes() const;
    qreal attributeAt(const QString &, qreal) const;
    QPointF pointAt(qreal) const;
    QPointF sequentialPointAt(qreal p, qreal *angle = 0) const;
    void invalidateSequentialHistory() const;

Q_SIGNALS:
    void changed();
    void startXChanged();
    void startYChanged();

protected:
    virtual void componentComplete();
    virtual void classBegin();
    void disconnectPathElements();
    void connectPathElements();
    void gatherAttributes();

    // pathElements property
    static QQuickPathElement *pathElements_at(QQmlListProperty<QQuickPathElement> *, int);
    static void pathElements_append(QQmlListProperty<QQuickPathElement> *, QQuickPathElement *);
    static int pathElements_count(QQmlListProperty<QQuickPathElement> *);
    static void pathElements_clear(QQmlListProperty<QQuickPathElement> *);

private Q_SLOTS:
    void processPath();

private:
    struct AttributePoint {
        AttributePoint() : percent(0), scale(1), origpercent(0) {}
        AttributePoint(const AttributePoint &other)
            : percent(other.percent), scale(other.scale), origpercent(other.origpercent), values(other.values) {}
        AttributePoint &operator=(const AttributePoint &other) {
            percent = other.percent; scale = other.scale; origpercent = other.origpercent; values = other.values; return *this;
        }
        qreal percent;      //massaged percent along the painter path
        qreal scale;
        qreal origpercent;  //'real' percent along the painter path
        QHash<QString, qreal> values;
    };

    void interpolate(int idx, const QString &name, qreal value);
    void endpoint(const QString &name);
    void createPointCache() const;

    static void interpolate(QList<AttributePoint> &points, int idx, const QString &name, qreal value);
    static void endpoint(QList<AttributePoint> &attributePoints, const QString &name);
    static QPointF forwardsPointAt(const QPainterPath &path, const qreal &pathLength, const QList<AttributePoint> &attributePoints, QQuickCachedBezier &prevBez, qreal p, qreal *angle = 0);
    static QPointF backwardsPointAt(const QPainterPath &path, const qreal &pathLength, const QList<AttributePoint> &attributePoints, QQuickCachedBezier &prevBez, qreal p, qreal *angle = 0);

private:
    Q_DISABLE_COPY(QQuickPath)
    Q_DECLARE_PRIVATE(QQuickPath)
    friend class QQuickPathAnimationUpdater;

public:
    QPainterPath createPath(const QPointF &startPoint, const QPointF &endPoint, const QStringList &attributes, qreal &pathLength, QList<AttributePoint> &attributePoints, bool *closed = 0);
    static QPointF sequentialPointAt(const QPainterPath &path, const qreal &pathLength, const QList<AttributePoint> &attributePoints, QQuickCachedBezier &prevBez, qreal p, qreal *angle = 0);
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickPathElement)
QML_DECLARE_TYPE(QQuickPathAttribute)
QML_DECLARE_TYPE(QQuickCurve)
QML_DECLARE_TYPE(QQuickPathLine)
QML_DECLARE_TYPE(QQuickPathQuad)
QML_DECLARE_TYPE(QQuickPathCubic)
QML_DECLARE_TYPE(QQuickPathCatmullRomCurve)
QML_DECLARE_TYPE(QQuickPathArc)
QML_DECLARE_TYPE(QQuickPathSvg)
QML_DECLARE_TYPE(QQuickPathPercent)
QML_DECLARE_TYPE(QQuickPath)

#endif // QQUICKPATH_H
