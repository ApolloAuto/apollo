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

#ifndef QQMLPRIVATE_H
#define QQMLPRIVATE_H

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

#include <QtQml/qtqmlglobal.h>

#include <QtCore/qglobal.h>
#include <QtCore/qvariant.h>
#include <QtCore/qurl.h>

QT_BEGIN_NAMESPACE

namespace QQmlPrivate {
struct CachedQmlUnit;
}

namespace QV4 {
struct ExecutionEngine;
namespace CompiledData {
struct Unit;
struct CompilationUnit;
}
typedef CompiledData::CompilationUnit *(*CompilationUnitFactoryFunction)();
}
namespace QmlIR {
struct Document;
typedef void (*IRLoaderFunction)(Document *, const QQmlPrivate::CachedQmlUnit *);
}

typedef QObject *(*QQmlAttachedPropertiesFunc)(QObject *);

template <typename TYPE>
class QQmlTypeInfo
{
public:
    enum {
        hasAttachedProperties = 0
    };
};


class QJSValue;
class QJSEngine;
class QQmlEngine;
class QQmlCustomParser;
namespace QQmlPrivate
{
    void Q_QML_EXPORT qdeclarativeelement_destructor(QObject *);
    template<typename T>
    class QQmlElement : public T
    {
    public:
        virtual ~QQmlElement() {
            QQmlPrivate::qdeclarativeelement_destructor(this);
        }
    };

    template<typename T>
    void createInto(void *memory) { new (memory) QQmlElement<T>; }

    template<typename T>
    QObject *createParent(QObject *p) { return new T(p); }

    template<class From, class To, int N>
    struct StaticCastSelectorClass
    {
        static inline int cast() { return -1; }
    };

    template<class From, class To>
    struct StaticCastSelectorClass<From, To, sizeof(int)>
    {
        static inline int cast() { return int(reinterpret_cast<quintptr>(static_cast<To *>(reinterpret_cast<From *>(0x10000000)))) - 0x10000000; }
    };

    template<class From, class To>
    struct StaticCastSelector
    {
        typedef int yes_type;
        typedef char no_type;

        static yes_type checkType(To *);
        static no_type checkType(...);

        static inline int cast()
        {
            return StaticCastSelectorClass<From, To, sizeof(checkType(reinterpret_cast<From *>(0)))>::cast();
        }
    };

    template <typename T>
    struct has_attachedPropertiesMember
    {
        static bool const value = QQmlTypeInfo<T>::hasAttachedProperties;
    };

    template <typename T, bool hasMember>
    class has_attachedPropertiesMethod
    {
    public:
        typedef int yes_type;
        typedef char no_type;

        template<typename ReturnType>
        static yes_type checkType(ReturnType *(*)(QObject *));
        static no_type checkType(...);

        static bool const value = sizeof(checkType(&T::qmlAttachedProperties)) == sizeof(yes_type);
    };

    template <typename T>
    class has_attachedPropertiesMethod<T, false>
    {
    public:
        static bool const value = false;
    };

    template<typename T, int N>
    class AttachedPropertySelector
    {
    public:
        static inline QQmlAttachedPropertiesFunc func() { return 0; }
        static inline const QMetaObject *metaObject() { return 0; }
    };
    template<typename T>
    class AttachedPropertySelector<T, 1>
    {
        static inline QObject *attachedProperties(QObject *obj) {
            return T::qmlAttachedProperties(obj);
        }
        template<typename ReturnType>
        static inline const QMetaObject *attachedPropertiesMetaObject(ReturnType *(*)(QObject *)) {
            return &ReturnType::staticMetaObject;
        }
    public:
        static inline QQmlAttachedPropertiesFunc func() {
            return &attachedProperties;
        }
        static inline const QMetaObject *metaObject() {
            return attachedPropertiesMetaObject(&T::qmlAttachedProperties);
        }
    };

    template<typename T>
    inline QQmlAttachedPropertiesFunc attachedPropertiesFunc()
    {
        return AttachedPropertySelector<T, has_attachedPropertiesMethod<T, has_attachedPropertiesMember<T>::value>::value>::func();
    }

    template<typename T>
    inline const QMetaObject *attachedPropertiesMetaObject()
    {
        return AttachedPropertySelector<T, has_attachedPropertiesMethod<T, has_attachedPropertiesMember<T>::value>::value>::metaObject();
    }

    enum AutoParentResult { Parented, IncompatibleObject, IncompatibleParent };
    typedef AutoParentResult (*AutoParentFunction)(QObject *object, QObject *parent);

    struct RegisterType {
        int version;

        int typeId;
        int listId;
        int objectSize;
        void (*create)(void *);
        QString noCreationReason;

        const char *uri;
        int versionMajor;
        int versionMinor;
        const char *elementName;
        const QMetaObject *metaObject;

        QQmlAttachedPropertiesFunc attachedPropertiesFunction;
        const QMetaObject *attachedPropertiesMetaObject;

        int parserStatusCast;
        int valueSourceCast;
        int valueInterceptorCast;

        QObject *(*extensionObjectCreate)(QObject *);
        const QMetaObject *extensionMetaObject;

        QQmlCustomParser *customParser;
        int revision;
        // If this is extended ensure "version" is bumped!!!
    };

    struct RegisterInterface {
        int version;

        int typeId;
        int listId;

        const char *iid;
    };

    struct RegisterAutoParent {
        int version;

        AutoParentFunction function;
    };

    struct RegisterSingletonType {
        int version;

        const char *uri;
        int versionMajor;
        int versionMinor;
        const char *typeName;

        QJSValue (*scriptApi)(QQmlEngine *, QJSEngine *);
        QObject *(*qobjectApi)(QQmlEngine *, QJSEngine *);
        const QMetaObject *instanceMetaObject; // new in version 1
        int typeId; // new in version 2
        int revision; // new in version 2
        // If this is extended ensure "version" is bumped!!!
    };

    struct RegisterCompositeType {
        QUrl url;
        const char *uri;
        int versionMajor;
        int versionMinor;
        const char *typeName;
    };

    struct RegisterCompositeSingletonType {
        QUrl url;
        const char *uri;
        int versionMajor;
        int versionMinor;
        const char *typeName;
    };

    struct CachedQmlUnit {
        const QV4::CompiledData::Unit *qmlData;
        QV4::CompilationUnitFactoryFunction createCompilationUnit;
        QmlIR::IRLoaderFunction loadIR;
    };

    typedef const CachedQmlUnit *(*QmlUnitCacheLookupFunction)(const QUrl &url);
    struct RegisterQmlUnitCacheHook {
        int version;
        QmlUnitCacheLookupFunction lookupCachedQmlUnit;
    };

    enum RegistrationType {
        TypeRegistration       = 0,
        InterfaceRegistration  = 1,
        AutoParentRegistration = 2,
        SingletonRegistration  = 3,
        CompositeRegistration  = 4,
        CompositeSingletonRegistration = 5,
        QmlUnitCacheHookRegistration = 6
    };

    int Q_QML_EXPORT qmlregister(RegistrationType, void *);
}

QT_END_NAMESPACE

#endif // QQMLPRIVATE_H
