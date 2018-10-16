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

#ifndef QQMLVALUETYPE_P_H
#define QQMLVALUETYPE_P_H

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

#include "qqml.h"
#include "qqmlproperty.h"
#include "qqmlproperty_p.h"
#include <private/qqmlnullablevalue_p.h>

#include <QtCore/qobject.h>
#include <QtCore/qrect.h>
#include <QtCore/qeasingcurve.h>
#include <QtCore/qvariant.h>

QT_BEGIN_NAMESPACE

class Q_QML_PRIVATE_EXPORT QQmlValueType : public QObject, public QAbstractDynamicMetaObject
{
public:
    QQmlValueType(int userType, const QMetaObject *metaObject);
    ~QQmlValueType();
    void read(QObject *, int);
    void write(QObject *, int, QQmlPropertyPrivate::WriteFlags flags);
    QVariant value();
    void setValue(const QVariant &);

    // ---- dynamic meta object data interface
    virtual QAbstractDynamicMetaObject *toDynamicMetaObject(QObject *);
    virtual void objectDestroyed(QObject *);
    virtual int metaCall(QObject *obj, QMetaObject::Call type, int _id, void **argv);
    // ----

private:
    const QMetaObject *_metaObject;
    void *gadgetPtr;

public:
    int typeId;
    QMetaType metaType;
};

class Q_QML_PRIVATE_EXPORT QQmlValueTypeFactory
{
public:
    static bool isValueType(int);
    static QQmlValueType *valueType(int idx);
    static const QMetaObject *metaObjectForMetaType(int type);

    static void registerValueTypes(const char *uri, int versionMajor, int versionMinor);
};

struct QQmlPointFValueType
{
    QPointF v;
    Q_PROPERTY(qreal x READ x WRITE setX FINAL)
    Q_PROPERTY(qreal y READ y WRITE setY FINAL)
    Q_GADGET
public:
    Q_INVOKABLE QString toString() const;
    qreal x() const;
    qreal y() const;
    void setX(qreal);
    void setY(qreal);
};

struct QQmlPointValueType
{
    QPoint v;
    Q_PROPERTY(int x READ x WRITE setX FINAL)
    Q_PROPERTY(int y READ y WRITE setY FINAL)
    Q_GADGET
public:
    int x() const;
    int y() const;
    void setX(int);
    void setY(int);
};

struct QQmlSizeFValueType
{
    QSizeF v;
    Q_PROPERTY(qreal width READ width WRITE setWidth FINAL)
    Q_PROPERTY(qreal height READ height WRITE setHeight FINAL)
    Q_GADGET
public:
    Q_INVOKABLE QString toString() const;
    qreal width() const;
    qreal height() const;
    void setWidth(qreal);
    void setHeight(qreal);
};

struct QQmlSizeValueType
{
    QSize v;
    Q_PROPERTY(int width READ width WRITE setWidth FINAL)
    Q_PROPERTY(int height READ height WRITE setHeight FINAL)
    Q_GADGET
public:
    int width() const;
    int height() const;
    void setWidth(int);
    void setHeight(int);
};

struct QQmlRectFValueType
{
    QRectF v;
    Q_PROPERTY(qreal x READ x WRITE setX FINAL)
    Q_PROPERTY(qreal y READ y WRITE setY FINAL)
    Q_PROPERTY(qreal width READ width WRITE setWidth FINAL)
    Q_PROPERTY(qreal height READ height WRITE setHeight FINAL)
    Q_PROPERTY(qreal left READ left DESIGNABLE false FINAL)
    Q_PROPERTY(qreal right READ right DESIGNABLE false FINAL)
    Q_PROPERTY(qreal top READ top DESIGNABLE false FINAL)
    Q_PROPERTY(qreal bottom READ bottom DESIGNABLE false FINAL)
    Q_GADGET
public:
    Q_INVOKABLE QString toString() const;
    qreal x() const;
    qreal y() const;
    void setX(qreal);
    void setY(qreal);

    qreal width() const;
    qreal height() const;
    void setWidth(qreal);
    void setHeight(qreal);

    qreal left() const;
    qreal right() const;
    qreal top() const;
    qreal bottom() const;
};

struct QQmlRectValueType
{
    QRect v;
    Q_PROPERTY(int x READ x WRITE setX FINAL)
    Q_PROPERTY(int y READ y WRITE setY FINAL)
    Q_PROPERTY(int width READ width WRITE setWidth FINAL)
    Q_PROPERTY(int height READ height WRITE setHeight FINAL)
    Q_PROPERTY(int left READ left DESIGNABLE false FINAL)
    Q_PROPERTY(int right READ right DESIGNABLE false FINAL)
    Q_PROPERTY(int top READ top DESIGNABLE false FINAL)
    Q_PROPERTY(int bottom READ bottom DESIGNABLE false FINAL)
    Q_GADGET
public:
    int x() const;
    int y() const;
    void setX(int);
    void setY(int);

    int width() const;
    int height() const;
    void setWidth(int);
    void setHeight(int);

    int left() const;
    int right() const;
    int top() const;
    int bottom() const;
};

struct QQmlEasingValueType
{
    QEasingCurve v;
    Q_GADGET
    Q_ENUMS(Type)

    Q_PROPERTY(QQmlEasingValueType::Type type READ type WRITE setType FINAL)
    Q_PROPERTY(qreal amplitude READ amplitude WRITE setAmplitude FINAL)
    Q_PROPERTY(qreal overshoot READ overshoot WRITE setOvershoot FINAL)
    Q_PROPERTY(qreal period READ period WRITE setPeriod FINAL)
    Q_PROPERTY(QVariantList bezierCurve READ bezierCurve WRITE setBezierCurve FINAL)
public:
    enum Type {
        Linear = QEasingCurve::Linear,
        InQuad = QEasingCurve::InQuad, OutQuad = QEasingCurve::OutQuad,
        InOutQuad = QEasingCurve::InOutQuad, OutInQuad = QEasingCurve::OutInQuad,
        InCubic = QEasingCurve::InCubic, OutCubic = QEasingCurve::OutCubic,
        InOutCubic = QEasingCurve::InOutCubic, OutInCubic = QEasingCurve::OutInCubic,
        InQuart = QEasingCurve::InQuart, OutQuart = QEasingCurve::OutQuart,
        InOutQuart = QEasingCurve::InOutQuart, OutInQuart = QEasingCurve::OutInQuart,
        InQuint = QEasingCurve::InQuint, OutQuint = QEasingCurve::OutQuint,
        InOutQuint = QEasingCurve::InOutQuint, OutInQuint = QEasingCurve::OutInQuint,
        InSine = QEasingCurve::InSine, OutSine = QEasingCurve::OutSine,
        InOutSine = QEasingCurve::InOutSine, OutInSine = QEasingCurve::OutInSine,
        InExpo = QEasingCurve::InExpo, OutExpo = QEasingCurve::OutExpo,
        InOutExpo = QEasingCurve::InOutExpo, OutInExpo = QEasingCurve::OutInExpo,
        InCirc = QEasingCurve::InCirc, OutCirc = QEasingCurve::OutCirc,
        InOutCirc = QEasingCurve::InOutCirc, OutInCirc = QEasingCurve::OutInCirc,
        InElastic = QEasingCurve::InElastic, OutElastic = QEasingCurve::OutElastic,
        InOutElastic = QEasingCurve::InOutElastic, OutInElastic = QEasingCurve::OutInElastic,
        InBack = QEasingCurve::InBack, OutBack = QEasingCurve::OutBack,
        InOutBack = QEasingCurve::InOutBack, OutInBack = QEasingCurve::OutInBack,
        InBounce = QEasingCurve::InBounce, OutBounce = QEasingCurve::OutBounce,
        InOutBounce = QEasingCurve::InOutBounce, OutInBounce = QEasingCurve::OutInBounce,
        InCurve = QEasingCurve::InCurve, OutCurve = QEasingCurve::OutCurve,
        SineCurve = QEasingCurve::SineCurve, CosineCurve = QEasingCurve::CosineCurve,
        Bezier = QEasingCurve::BezierSpline
    };

    Type type() const;
    qreal amplitude() const;
    qreal overshoot() const;
    qreal period() const;
    void setType(Type);
    void setAmplitude(qreal);
    void setOvershoot(qreal);
    void setPeriod(qreal);
    void setBezierCurve(const QVariantList &);
    QVariantList bezierCurve() const;
};

template<typename T>
int qmlRegisterValueTypeEnums(const char *uri, int versionMajor, int versionMinor, const char *qmlName)
{
    QByteArray name(T::staticMetaObject.className());

    QByteArray pointerName(name + '*');

    QQmlPrivate::RegisterType type = {
        0,

        qRegisterNormalizedMetaType<T *>(pointerName.constData()), 0, 0, 0,

        QString(),

        uri, versionMajor, versionMinor, qmlName, &T::staticMetaObject,

        0, 0,

        0, 0, 0,

        0, 0,

        0,
        0
    };

    return QQmlPrivate::qmlregister(QQmlPrivate::TypeRegistration, &type);
}

QT_END_NAMESPACE

#endif  // QQMLVALUETYPE_P_H
