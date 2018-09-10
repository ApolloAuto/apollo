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

#ifndef QQMLACCESSORS_P_H
#define QQMLACCESSORS_P_H

#include <private/qtqmlglobal_p.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qvector.h>
#include <QtCore/qhash.h>
#include <QtCore/QReadWriteLock>

#if defined(Q_OS_QNX) || defined(Q_OS_LINUX)
#include <stdint.h>
#endif

QT_BEGIN_NAMESPACE

class QObject;
struct QMetaObject;
class QQmlNotifier;

// QML "accessor properties" allow V4 and V8 to bypass Qt's meta system to read and, more
// importantly, subscribe to properties directly.  Any property that is primarily read
// from bindings is a candidate for inclusion as an accessor property.
//
// To define accessor properties, use the QML_DECLARE_PROPERTIES() and QML_DEFINE_PROPERTIES()
// macros.  The QML_DECLARE_PROPERTIES() macro is used to specify the properties, and the
// QML_DEFINE_PROPERTIES() macro to register the properties with the
// QQmlAccessorProperties singleton.
//
// A class with accessor properties must also add the Q_CLASSINFO("qt_HasQmlAccessors", "true")
// tag to its declaration.  This is essential for QML to maintain internal consistency,
// and forgetting to do so will probably cause your application to qFatal() with a
// helpful reminder of this requirement.
//
// It is important that QML_DEFINE_PROPERTIES() has been called before QML ever sees
// the type with the accessor properties.  As QML_DEFINE_PROPERTIES() is idempotent, it is
// recommended to call it in the type's constructor as well as when the type is registered
// as a QML element (if it ever is).  QML_DEFINE_PROPERTIES() is a very cheap operation
// if registration has already occurred.

#define QML_DECLARE_PROPERTIES(type) \
    static volatile bool qqml_accessor_properties_isregistered_ ## type = false; \
    static QQmlAccessorProperties::Property qqml_accessor_properties_ ## type[] =

#define QML_DEFINE_PROPERTIES(type) \
    do { \
        if (!qqml_accessor_properties_isregistered_ ## type) { \
            int count = sizeof(qqml_accessor_properties_ ## type) / \
                        sizeof(QQmlAccessorProperties::Property); \
            QQmlAccessorProperties::registerProperties(&type::staticMetaObject, count, \
                    qqml_accessor_properties_ ## type);\
            qqml_accessor_properties_isregistered_ ## type = true; \
        } \
    } while (false);

#define QML_PRIVATE_ACCESSOR(clazz, cpptype, name, variable) \
    static void clazz ## _ ## name ## Read(QObject *o, qintptr, void *rv) \
    { \
        clazz ## Private *d = clazz ## Private::get(static_cast<clazz *>(o)); \
        *static_cast<cpptype *>(rv) = d->variable; \
    }

#define QML_PROPERTY_NAME(name) #name, sizeof #name - 1

class QQmlAccessors
{
public:
    void (*read)(QObject *object, qintptr property, void *output);
    void (*notifier)(QObject *object, qintptr property, QQmlNotifier **notifier);
};

namespace QQmlAccessorProperties {
    struct Property {
        const char *name;
        unsigned int nameLength;
        qintptr data;
        QQmlAccessors *accessors;
    };

    struct Properties {
        inline Properties();
        Properties(Property *, int);

        bool operator==(const Properties &o) const {
            return count == o.count && properties == o.properties;
        }

        inline Property *property(const char *name);

        int count;
        Property *properties;
        quint32 nameMask;
    };

    Properties properties(const QMetaObject *);
    void Q_QML_PRIVATE_EXPORT registerProperties(const QMetaObject *, int, Property *);
};

QQmlAccessorProperties::Property *
QQmlAccessorProperties::Properties::property(const char *name)
{
    if (count == 0)
        return 0;

    const unsigned int length = (unsigned int)strlen(name);

    Q_ASSERT(length);

    if (nameMask & (1 << qMin(31U, length - 1))) {

        for (int ii = 0; ii < count; ++ii) {
            if (properties[ii].nameLength == length && 0 == qstrcmp(name, properties[ii].name))
                return &properties[ii];
        }

    }

    return 0;
}

QQmlAccessorProperties::Properties::Properties()
: count(0), properties(0), nameMask(0)
{
}

QT_END_NAMESPACE

#endif // QQMLACCESSORS_P_H
