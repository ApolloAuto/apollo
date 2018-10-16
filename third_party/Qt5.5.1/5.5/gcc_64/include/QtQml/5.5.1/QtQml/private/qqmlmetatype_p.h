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

#ifndef QQMLMETATYPE_P_H
#define QQMLMETATYPE_P_H

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
#include <private/qtqmlglobal_p.h>

#include <QtCore/qglobal.h>
#include <QtCore/qvariant.h>
#include <QtCore/qbitarray.h>
#include <QtQml/qjsvalue.h>

QT_BEGIN_NAMESPACE

class QQmlType;
class QQmlEngine;
class QQmlCustomParser;
class QQmlTypePrivate;
class QQmlTypeModule;
class QHashedString;
class QHashedStringRef;
class QMutex;

namespace QV4 { struct String; }

class Q_QML_PRIVATE_EXPORT QQmlMetaType
{
public:
    static QList<QString> qmlTypeNames();
    static QList<QQmlType*> qmlTypes();
    static QList<QQmlType*> qmlSingletonTypes();
    static QList<QQmlType*> qmlAllTypes();

    static QQmlType *qmlType(const QString &qualifiedName, int, int);
    static QQmlType *qmlType(const QHashedStringRef &name, const QHashedStringRef &module, int, int);
    static QQmlType *qmlType(const QMetaObject *);
    static QQmlType *qmlType(const QMetaObject *metaObject, const QHashedStringRef &module, int version_major, int version_minor);
    static QQmlType *qmlType(int);
    static QQmlType *qmlType(const QUrl &url, bool includeNonFileImports = false);
    static QQmlType *qmlTypeFromIndex(int);

    static QMetaProperty defaultProperty(const QMetaObject *);
    static QMetaProperty defaultProperty(QObject *);
    static QMetaMethod defaultMethod(const QMetaObject *);
    static QMetaMethod defaultMethod(QObject *);

    static bool isQObject(int);
    static QObject *toQObject(const QVariant &, bool *ok = 0);

    static int listType(int);
    static int attachedPropertiesFuncId(const QMetaObject *);
    static QQmlAttachedPropertiesFunc attachedPropertiesFuncById(int);

    enum TypeCategory { Unknown, Object, List };
    static TypeCategory typeCategory(int);

    static bool isInterface(int);
    static const char *interfaceIId(int);
    static bool isList(int);

    typedef QVariant (*StringConverter)(const QString &);
    static void registerCustomStringConverter(int, StringConverter);
    static StringConverter customStringConverter(int);

    static bool isAnyModule(const QString &uri);
    static bool isLockedModule(const QString &uri, int majorVersion);
    static bool isModule(const QString &module, int versionMajor, int versionMinor);
    static QQmlTypeModule *typeModule(const QString &uri, int majorVersion);

    static QList<QQmlPrivate::AutoParentFunction> parentFunctions();

    static const QQmlPrivate::CachedQmlUnit *findCachedCompilationUnit(const QUrl &uri);

    static bool namespaceContainsRegistrations(const QString &, int majorVersion);

    static void protectNamespace(const QString &);

    static void setTypeRegistrationNamespace(const QString &);
    static QStringList typeRegistrationFailures();

    static QMutex *typeRegistrationLock();
};

struct QQmlMetaTypeData;
class QHashedCStringRef;
class Q_QML_PRIVATE_EXPORT QQmlType
{
public:
    QByteArray typeName() const;
    const QString &qmlTypeName() const;
    const QString &elementName() const;

    const QHashedString &module() const;
    int majorVersion() const;
    int minorVersion() const;

    bool availableInVersion(int vmajor, int vminor) const;
    bool availableInVersion(const QHashedStringRef &module, int vmajor, int vminor) const;

    QObject *create() const;
    void create(QObject **, void **, size_t) const;

    typedef void (*CreateFunc)(void *);
    CreateFunc createFunction() const;
    int createSize() const;

    QQmlCustomParser *customParser() const;

    bool isCreatable() const;
    bool isExtendedType() const;
    QString noCreationReason() const;

    bool isSingleton() const;
    bool isInterface() const;
    bool isComposite() const;
    bool isCompositeSingleton() const;

    int typeId() const;
    int qListTypeId() const;

    const QMetaObject *metaObject() const;
    const QMetaObject *baseMetaObject() const;
    int metaObjectRevision() const;
    bool containsRevisionedAttributes() const;

    QQmlAttachedPropertiesFunc attachedPropertiesFunction() const;
    const QMetaObject *attachedPropertiesType() const;
    int attachedPropertiesId() const;

    int parserStatusCast() const;
    const char *interfaceIId() const;
    int propertyValueSourceCast() const;
    int propertyValueInterceptorCast() const;

    int index() const;

    class Q_QML_PRIVATE_EXPORT SingletonInstanceInfo
    {
    public:
        SingletonInstanceInfo()
            : scriptCallback(0), qobjectCallback(0), instanceMetaObject(0) {}

        QJSValue (*scriptCallback)(QQmlEngine *, QJSEngine *);
        QObject *(*qobjectCallback)(QQmlEngine *, QJSEngine *);
        const QMetaObject *instanceMetaObject;
        QString typeName;
        QUrl url; // used by composite singletons

        void setQObjectApi(QQmlEngine *, QObject *);
        QObject *qobjectApi(QQmlEngine *) const;
        void setScriptApi(QQmlEngine *, QJSValue);
        QJSValue scriptApi(QQmlEngine *) const;

        void init(QQmlEngine *);
        void destroy(QQmlEngine *);

        QHash<QQmlEngine *, QJSValue> scriptApis;
        QHash<QQmlEngine *, QObject *> qobjectApis;
    };
    SingletonInstanceInfo *singletonInstanceInfo() const;

    QUrl sourceUrl() const;

    int enumValue(const QHashedStringRef &, bool *ok) const;
    int enumValue(const QHashedCStringRef &, bool *ok) const;
    int enumValue(const QV4::String *, bool *ok) const;
private:
    QQmlType *superType() const;
    friend class QQmlTypePrivate;
    friend struct QQmlMetaTypeData;

    enum RegistrationType {
        CppType = 0,
        SingletonType = 1,
        InterfaceType = 2,
        CompositeType = 3,
        CompositeSingletonType = 4
    };
    friend QString registrationTypeString(RegistrationType);
    friend bool checkRegistration(RegistrationType, QQmlMetaTypeData *, const char *, const QString &, int);
    friend int registerType(const QQmlPrivate::RegisterType &);
    friend int registerSingletonType(const QQmlPrivate::RegisterSingletonType &);
    friend int registerInterface(const QQmlPrivate::RegisterInterface &);
    friend int registerCompositeType(const QQmlPrivate::RegisterCompositeType &);
    friend int registerCompositeSingletonType(const QQmlPrivate::RegisterCompositeSingletonType &);
    friend int registerQmlUnitCacheHook(const QQmlPrivate::RegisterQmlUnitCacheHook &);
    friend Q_QML_EXPORT void qmlClearTypeRegistrations();
    QQmlType(int, const QQmlPrivate::RegisterInterface &);
    QQmlType(int, const QString &, const QQmlPrivate::RegisterSingletonType &);
    QQmlType(int, const QString &, const QQmlPrivate::RegisterType &);
    QQmlType(int, const QString &, const QQmlPrivate::RegisterCompositeType &);
    QQmlType(int, const QString &, const QQmlPrivate::RegisterCompositeSingletonType &);
    ~QQmlType();

    QQmlTypePrivate *d;
};

class QQmlTypeModulePrivate;
class QQmlTypeModule
{
public:
    QString module() const;
    int majorVersion() const;

    int minimumMinorVersion() const;
    int maximumMinorVersion() const;

    QQmlType *type(const QHashedStringRef &, int);
    QQmlType *type(const QV4::String *, int);

    QList<QQmlType*> singletonTypes(int) const;

private:
    //Used by register functions and creates the QQmlTypeModule for them
    friend void addTypeToData(QQmlType* type, QQmlMetaTypeData *data);
    friend struct QQmlMetaTypeData;
    friend Q_QML_EXPORT void qmlClearTypeRegistrations();
    friend class QQmlTypeModulePrivate;

    QQmlTypeModule();
    ~QQmlTypeModule();
    QQmlTypeModulePrivate *d;
};

class QQmlTypeModuleVersion
{
public:
    QQmlTypeModuleVersion();
    QQmlTypeModuleVersion(QQmlTypeModule *, int);
    QQmlTypeModuleVersion(const QQmlTypeModuleVersion &);
    QQmlTypeModuleVersion &operator=(const QQmlTypeModuleVersion &);

    QQmlTypeModule *module() const;
    int minorVersion() const;

    QQmlType *type(const QHashedStringRef &) const;
    QQmlType *type(const QV4::String *) const;

private:
    QQmlTypeModule *m_module;
    int m_minor;
};

QT_END_NAMESPACE

#endif // QQMLMETATYPE_P_H

