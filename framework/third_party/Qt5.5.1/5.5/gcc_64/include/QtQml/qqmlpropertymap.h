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

#ifndef QQMLPROPERTYMAP_H
#define QQMLPROPERTYMAP_H

#include <QtQml/qtqmlglobal.h>

#include <QtCore/QObject>
#include <QtCore/QHash>
#include <QtCore/QStringList>
#include <QtCore/QVariant>

QT_BEGIN_NAMESPACE


class QQmlPropertyMapPrivate;
class Q_QML_EXPORT QQmlPropertyMap : public QObject
{
    Q_OBJECT
public:
    explicit QQmlPropertyMap(QObject *parent = 0);
    virtual ~QQmlPropertyMap();

    QVariant value(const QString &key) const;
    void insert(const QString &key, const QVariant &value);
    void clear(const QString &key);

    Q_INVOKABLE QStringList keys() const;

    int count() const;
    int size() const;
    bool isEmpty() const;
    bool contains(const QString &key) const;

    QVariant &operator[](const QString &key);
    QVariant operator[](const QString &key) const;

Q_SIGNALS:
    void valueChanged(const QString &key, const QVariant &value);

protected:
    virtual QVariant updateValue(const QString &key, const QVariant &input);

    template<class DerivedType>
    QQmlPropertyMap(DerivedType *derived, QObject *parentObj)
        : QObject(*allocatePrivate(), parentObj)
    {
        Q_UNUSED(derived)
        init(&DerivedType::staticMetaObject);
    }

private:
    void init(const QMetaObject *staticMetaObject);
    static QObjectPrivate *allocatePrivate();

    Q_DECLARE_PRIVATE(QQmlPropertyMap)
    Q_DISABLE_COPY(QQmlPropertyMap)
};

QT_END_NAMESPACE

#endif
