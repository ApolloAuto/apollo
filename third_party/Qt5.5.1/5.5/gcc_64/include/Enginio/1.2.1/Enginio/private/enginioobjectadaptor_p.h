/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtEnginio module of the Qt Toolkit.
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

#ifndef ENGINIOOBJECTADAPTOR_P_H
#define ENGINIOOBJECTADAPTOR_P_H

#include <QtCore/qjsonarray.h>
#include <QtCore/qjsondocument.h>
#include <QtCore/qjsonobject.h>
#include <QtCore/qjsonvalue.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qstring.h>

QT_BEGIN_NAMESPACE

// generic versions of these clasess are undefined
template <class T> struct ValueAdaptor;
template <class T> struct ObjectAdaptor;
template <class T> struct ArrayAdaptor;

template <> struct ValueAdaptor<QJsonObject>;
template <> struct ObjectAdaptor<QJsonObject>;
template <> struct ArrayAdaptor<QJsonObject>;

template <> struct ValueAdaptor<QJsonObject>
{
    QJsonValue _value;
    ValueAdaptor(const QJsonValue &value)
        : _value(value)
    {}
    ValueAdaptor(const QJsonValueRef &value)
        : _value(value)
    {}

    bool isComposedType() const { return _value.isObject() || _value.isArray(); }

    int toInt() const { return _value.toDouble(); }
    QString toString() const { return _value.toString(); }
    QByteArray toJson() const
    {
        if (_value.isObject())
            return QJsonDocument(_value.toObject()).toJson(QJsonDocument::Compact);
        if (_value.isArray())
            return QJsonDocument(_value.toArray()).toJson(QJsonDocument::Compact);
        Q_UNIMPLEMENTED();
        return QByteArray();
    }
    inline ObjectAdaptor<QJsonObject> toObject() const;
    inline ArrayAdaptor<QJsonObject> toArray() const;
};

template <> struct ObjectAdaptor<QJsonObject>
{
    QJsonObject _object;
    ObjectAdaptor(const QJsonObject &object, EnginioClientConnectionPrivate* = 0)
        : _object(object)
    {}

    ValueAdaptor<QJsonObject> operator[](const QString &index) const { return _object[index]; }
    bool contains(const QString &key) const { return _object.contains(key); }
    QByteArray toJson() const { return QJsonDocument(_object).toJson(QJsonDocument::Compact); }
    void remove(const QString &index) { _object.remove(index); }
    bool isEmpty() const { return _object.isEmpty(); }
};

template <> struct ArrayAdaptor<QJsonObject>
{
    const QJsonArray _array;
    ArrayAdaptor(const QJsonArray &object)
        : _array(object)
    {}

    bool isEmpty() const { return _array.isEmpty(); }
    QByteArray toJson() const { return QJsonDocument(_array).toJson(QJsonDocument::Compact); }

    struct const_iterator: public QJsonArray::const_iterator {
        ValueAdaptor<QJsonObject> operator *() const
        {
            return QJsonArray::const_iterator::operator *();
        }
        const_iterator(const QJsonArray::const_iterator &other)
            : QJsonArray::const_iterator(other)
        {}
    };

    const_iterator constBegin() const { return _array.constBegin(); }
    const_iterator constEnd() const { return _array.constEnd(); }
};


ObjectAdaptor<QJsonObject> ValueAdaptor<QJsonObject>::toObject() const { return _value.toObject(); }
ArrayAdaptor<QJsonObject> ValueAdaptor<QJsonObject>::toArray() const { return _value.toArray(); }

QT_END_NAMESPACE

#endif // ENGINIOOBJECTADAPTOR_P_H
