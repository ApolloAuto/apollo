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

#ifndef QQMLPROPERTY_H
#define QQMLPROPERTY_H

#include <QtQml/qtqmlglobal.h>
#include <QtCore/qmetaobject.h>

QT_BEGIN_NAMESPACE


class QObject;
class QVariant;
class QQmlContext;
class QQmlEngine;

class QQmlPropertyPrivate;
class Q_QML_EXPORT QQmlProperty
{
public:
    enum PropertyTypeCategory {
        InvalidCategory,
        List,
        Object,
        Normal
    };

    enum Type {
        Invalid,
        Property,
        SignalProperty
    };

    QQmlProperty();
    ~QQmlProperty();

    QQmlProperty(QObject *);
    QQmlProperty(QObject *, QQmlContext *);
    QQmlProperty(QObject *, QQmlEngine *);

    QQmlProperty(QObject *, const QString &);
    QQmlProperty(QObject *, const QString &, QQmlContext *);
    QQmlProperty(QObject *, const QString &, QQmlEngine *);

    QQmlProperty(const QQmlProperty &);
    QQmlProperty &operator=(const QQmlProperty &);

    bool operator==(const QQmlProperty &) const;

    Type type() const;
    bool isValid() const;
    bool isProperty() const;
    bool isSignalProperty() const;

    int propertyType() const;
    PropertyTypeCategory propertyTypeCategory() const;
    const char *propertyTypeName() const;

    QString name() const;

    QVariant read() const;
    static QVariant read(const QObject *, const QString &);
    static QVariant read(const QObject *, const QString &, QQmlContext *);
    static QVariant read(const QObject *, const QString &, QQmlEngine *);

    bool write(const QVariant &) const;
    static bool write(QObject *, const QString &, const QVariant &);
    static bool write(QObject *, const QString &, const QVariant &, QQmlContext *);
    static bool write(QObject *, const QString &, const QVariant &, QQmlEngine *);

    bool reset() const;

    bool hasNotifySignal() const;
    bool needsNotifySignal() const;
    bool connectNotifySignal(QObject *dest, const char *slot) const;
    bool connectNotifySignal(QObject *dest, int method) const;

    bool isWritable() const;
    bool isDesignable() const;
    bool isResettable() const;
    QObject *object() const;

    int index() const;
    QMetaProperty property() const;
    QMetaMethod method() const;

private:
    friend class QQmlPropertyPrivate;
    QQmlPropertyPrivate *d;
};
typedef QList<QQmlProperty> QQmlProperties;

inline uint qHash (const QQmlProperty &key)
{
    return qHash(key.object()) + qHash(key.name());
}

Q_DECLARE_TYPEINFO(QQmlProperty, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif // QQMLPROPERTY_H
