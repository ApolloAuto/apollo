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

#ifndef QQMLPROPERTYCACHE_P_H
#define QQMLPROPERTYCACHE_P_H

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

#include <private/qqmlrefcount_p.h>
#include <private/qflagpointer_p.h>
#include "qqmlcleanup_p.h"
#include "qqmlnotifier_p.h"

#include <private/qhashedstring_p.h>
#include <QtCore/qvarlengtharray.h>
#include <QtCore/qvector.h>

#include <private/qv4value_inl_p.h>

QT_BEGIN_NAMESPACE

class QV8Engine;
class QMetaProperty;
class QQmlEngine;
class QJSEngine;
class QQmlPropertyData;
class QQmlAccessors;
class QMetaObjectBuilder;
class QQmlPropertyCacheMethodArguments;
class QQmlVMEMetaObject;
class QQmlPropertyCacheCreator;

// We have this somewhat awful split between RawData and Data so that RawData can be
// used in unions.  In normal code, you should always use Data which initializes RawData
// to an invalid state on construction.
class QQmlPropertyRawData
{
public:
    enum Flag {
        NoFlags           = 0x00000000,
        ValueTypeFlagMask = 0x0000FFFF, // Flags in valueTypeFlags must fit in this mask

        // Can apply to all properties, except IsFunction
        IsConstant         = 0x00000001, // Has CONST flag
        IsWritable         = 0x00000002, // Has WRITE function
        IsResettable       = 0x00000004, // Has RESET function
        IsAlias            = 0x00000008, // Is a QML alias to another property
        IsFinal            = 0x00000010, // Has FINAL flag
        IsOverridden       = 0x00000020, // Is overridden by a extension property
        IsDirect           = 0x00000040, // Exists on a C++ QMetaObject
        HasAccessors       = 0x00000080, // Has property accessors

        // These are mutualy exclusive
        IsFunction         = 0x00000100, // Is an invokable
        IsQObjectDerived   = 0x00000200, // Property type is a QObject* derived type
        IsEnumType         = 0x00000400, // Property type is an enum
        IsQList            = 0x00000800, // Property type is a QML list
        IsQmlBinding       = 0x00001000, // Property type is a QQmlBinding*
        IsQJSValue         = 0x00002000, // Property type is a QScriptValue
        IsV4Handle         = 0x00004000, // Property type is a QQmlV4Handle
        IsVarProperty      = 0x00008000, // Property type is a "var" property of VMEMO
        IsValueTypeVirtual = 0x00010000, // Property is a value type "virtual" property
        IsQVariant         = 0x00020000, // Property is a QVariant

        // Apply only to IsFunctions
        IsVMEFunction      = 0x00040000, // Function was added by QML
        HasArguments       = 0x00080000, // Function takes arguments
        IsSignal           = 0x00100000, // Function is a signal
        IsVMESignal        = 0x00200000, // Signal was added by QML
        IsV4Function       = 0x00400000, // Function takes QQmlV4Function* args
        IsSignalHandler    = 0x00800000, // Function is a signal handler
        IsOverload         = 0x01000000, // Function is an overload of another function
        IsCloned           = 0x02000000, // The function was marked as cloned

        // Internal QQmlPropertyCache flags
        NotFullyResolved   = 0x04000000, // True if the type data is to be lazily resolved

        // Flags that are set based on the propType field
        PropTypeFlagMask = IsQObjectDerived | IsEnumType | IsQList | IsQmlBinding | IsQJSValue |
                           IsV4Handle | IsQVariant,
    };
    Q_DECLARE_FLAGS(Flags, Flag)

    Flags getFlags() const { return Flag(flags); }
    void setFlags(Flags f) { flags = f; }

    bool isValid() const { return coreIndex != -1; }

    bool isConstant() const { return flags & IsConstant; }
    bool isWritable() const { return flags & IsWritable; }
    bool isResettable() const { return flags & IsResettable; }
    bool isAlias() const { return flags & IsAlias; }
    bool isFinal() const { return flags & IsFinal; }
    bool isOverridden() const { return flags & IsOverridden; }
    bool isDirect() const { return flags & IsDirect; }
    bool hasAccessors() const { return flags & HasAccessors; }
    bool isFunction() const { return flags & IsFunction; }
    bool isQObject() const { return flags & IsQObjectDerived; }
    bool isEnum() const { return flags & IsEnumType; }
    bool isQList() const { return flags & IsQList; }
    bool isQmlBinding() const { return flags & IsQmlBinding; }
    bool isQJSValue() const { return flags & IsQJSValue; }
    bool isV4Handle() const { return flags & IsV4Handle; }
    bool isVarProperty() const { return flags & IsVarProperty; }
    bool isValueTypeVirtual() const { return flags & IsValueTypeVirtual; }
    bool isQVariant() const { return flags & IsQVariant; }
    bool isVMEFunction() const { return flags & IsVMEFunction; }
    bool hasArguments() const { return flags & HasArguments; }
    bool isSignal() const { return flags & IsSignal; }
    bool isVMESignal() const { return flags & IsVMESignal; }
    bool isV4Function() const { return flags & IsV4Function; }
    bool isSignalHandler() const { return flags & IsSignalHandler; }
    bool isOverload() const { return flags & IsOverload; }
    bool isCloned() const { return flags & IsCloned; }

    bool hasOverride() const { return !(flags & IsValueTypeVirtual) &&
                                      !(flags & HasAccessors) &&
                                      overrideIndex >= 0; }
    bool hasRevision() const { return !(flags & HasAccessors) && revision != 0; }

    // Returns -1 if not a value type virtual property
    inline int getValueTypeCoreIndex() const;

    // Returns the "encoded" index for use with bindings.  Encoding is:
    //     coreIndex | ((valueTypeCoreIndex + 1) << 16)
    inline int encodedIndex() const;
    static int encodeValueTypePropertyIndex(int coreIndex, int valueTypeCoreIndex)
    { return coreIndex | ((valueTypeCoreIndex + 1) << 16); }
    static int decodeValueTypePropertyIndex(int index, int *coreIndex = 0) {
        if (coreIndex) *coreIndex = index & 0xffff;
        return (index >> 16) - 1;
    }

    union {
        int propType;             // When !NotFullyResolved
        const char *propTypeName; // When NotFullyResolved
    };
    int coreIndex;
    union {
        // The notify index is in the range returned by QObjectPrivate::signalIndex().
        // This is different from QMetaMethod::methodIndex().
        int notifyIndex;  // When !IsFunction
        void *arguments;  // When IsFunction && HasArguments
    };

    union {
        struct { // When !HasAccessors
            qint16 revision;
            qint16 metaObjectOffset;

            union {
                struct { // When IsValueTypeVirtual
                    quint16 valueTypeFlags; // flags of the access property on the value type proxy
                                            // object
                    quint16 valueTypePropType; // The QVariant::Type of access property on the value
                                               // type proxy object
                    quint16 valueTypeCoreIndex; // The prop index of the access property on the value
                                                // type proxy object
                };

                struct { // When !IsValueTypeVirtual
                    uint overrideIndexIsProperty : 1;
                    signed int overrideIndex : 31;
                };
            };
        };
        struct { // When HasAccessors
            QQmlAccessors *accessors;
            qintptr accessorData;
        };
    };

private:
    friend class QQmlPropertyData;
    friend class QQmlPropertyCache;
    quint32 flags;
};
Q_DECLARE_OPERATORS_FOR_FLAGS(QQmlPropertyRawData::Flags)

class QQmlPropertyData : public QQmlPropertyRawData
{
public:
    inline QQmlPropertyData();
    inline QQmlPropertyData(const QQmlPropertyRawData &);

    inline bool operator==(const QQmlPropertyRawData &);

    static Flags flagsForProperty(const QMetaProperty &, QQmlEngine *engine = 0);
    void load(const QMetaProperty &, QQmlEngine *engine = 0);
    void load(const QMetaMethod &);
    QString name(QObject *) const;
    QString name(const QMetaObject *) const;

    void markAsOverrideOf(QQmlPropertyData *predecessor);

private:
    friend class QQmlPropertyCache;
    void lazyLoad(const QMetaProperty &);
    void lazyLoad(const QMetaMethod &);
    bool notFullyResolved() const { return flags & NotFullyResolved; }
};

class QQmlPropertyCacheMethodArguments;
class Q_QML_PRIVATE_EXPORT QQmlPropertyCache : public QQmlRefCount, public QQmlCleanup
{
public:
    QQmlPropertyCache(QJSEngine *);
    QQmlPropertyCache(QJSEngine *, const QMetaObject *);
    virtual ~QQmlPropertyCache();

    void update(const QMetaObject *);
    void invalidate(const QMetaObject *);
    // Used by qmlpuppet. Remove as soon Creator requires Qt 5.5.
    void invalidate(void *, const QMetaObject *mo) { invalidate(mo); }

    QQmlPropertyCache *copy();

    QQmlPropertyCache *copyAndAppend(const QMetaObject *,
                QQmlPropertyData::Flag propertyFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag methodFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag signalFlags = QQmlPropertyData::NoFlags);
    QQmlPropertyCache *copyAndAppend(const QMetaObject *, int revision,
                QQmlPropertyData::Flag propertyFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag methodFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag signalFlags = QQmlPropertyData::NoFlags);

    QQmlPropertyCache *copyAndReserve(int propertyCount,
                                      int methodCount, int signalCount);
    void appendProperty(const QString &,
                        quint32 flags, int coreIndex, int propType, int notifyIndex);
    void appendProperty(const QHashedCStringRef &,
                        quint32 flags, int coreIndex, int propType, int notifyIndex);
    void appendSignal(const QString &, quint32, int coreIndex, const int *types = 0,
                      const QList<QByteArray> &names = QList<QByteArray>());
    void appendSignal(const QHashedCStringRef &, quint32, int coreIndex, const int *types = 0,
                      const QList<QByteArray> &names = QList<QByteArray>());
    void appendMethod(const QString &, quint32 flags, int coreIndex,
                      const QList<QByteArray> &names = QList<QByteArray>());
    void appendMethod(const QHashedCStringRef &, quint32 flags, int coreIndex,
                      const QList<QByteArray> &names = QList<QByteArray>());

    const QMetaObject *metaObject() const;
    const QMetaObject *createMetaObject();
    const QMetaObject *firstCppMetaObject() const;

    template<typename K>
    QQmlPropertyData *property(const K &key, QObject *object, QQmlContextData *context) const
    {
        return findProperty(stringCache.find(key), object, context);
    }

    QQmlPropertyData *property(int) const;
    QQmlPropertyData *method(int) const;
    QQmlPropertyData *signal(int index) const { return signal(index, 0); }
    int methodIndexToSignalIndex(int) const;
    QStringList propertyNames() const;

    QString defaultPropertyName() const;
    QQmlPropertyData *defaultProperty() const;
    QQmlPropertyCache *parent() const;
    // is used by the Qml Designer
    void setParent(QQmlPropertyCache *newParent);

    inline QQmlPropertyData *overrideData(QQmlPropertyData *) const;
    inline bool isAllowedInRevision(QQmlPropertyData *) const;

    static QQmlPropertyData *property(QJSEngine *, QObject *, const QString &,
                                              QQmlContextData *, QQmlPropertyData &);
    static QQmlPropertyData *property(QJSEngine *, QObject *, const QV4::String *,
                                              QQmlContextData *, QQmlPropertyData &);

    //see QMetaObjectPrivate::originalClone
    int originalClone(int index);
    static int originalClone(QObject *, int index);

    QList<QByteArray> signalParameterNames(int index) const;
    QString signalParameterStringForJS(int index, QString *errorString = 0);
    static QString signalParameterStringForJS(QV4::ExecutionEngine *engine, const QList<QByteArray> &parameterNameList, QString *errorString = 0);

    const char *className() const;

    inline int propertyCount() const;
    inline int propertyOffset() const;
    inline int methodCount() const;
    inline int methodOffset() const;
    inline int signalCount() const;
    inline int signalOffset() const;

    static bool isDynamicMetaObject(const QMetaObject *);

    void toMetaObjectBuilder(QMetaObjectBuilder &);

protected:
    virtual void destroy();
    virtual void clear();

private:
    friend class QQmlEnginePrivate;
    friend class QQmlCompiler;
    friend class QQmlPropertyCacheCreator;
    friend class QQmlComponentAndAliasResolver;
    friend class QQmlMetaObject;

    inline QQmlPropertyCache *copy(int reserve);

    void append(const QMetaObject *, int revision,
                QQmlPropertyData::Flag propertyFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag methodFlags = QQmlPropertyData::NoFlags,
                QQmlPropertyData::Flag signalFlags = QQmlPropertyData::NoFlags);

    QQmlPropertyCacheMethodArguments *createArgumentsObject(int count,
                                                            const QList<QByteArray> &names);
    QQmlPropertyData *signal(int, QQmlPropertyCache **) const;

    typedef QVector<QQmlPropertyData> IndexCache;
    typedef QStringMultiHash<QPair<int, QQmlPropertyData *> > StringCache;
    typedef QVector<int> AllowedRevisionCache;

    QQmlPropertyData *findProperty(StringCache::ConstIterator it, QObject *, QQmlContextData *) const;
    QQmlPropertyData *findProperty(StringCache::ConstIterator it, const QQmlVMEMetaObject *, QQmlContextData *) const;

    QQmlPropertyData *ensureResolved(QQmlPropertyData*) const;

    void resolve(QQmlPropertyData *) const;
    void updateRecur(const QMetaObject *);

    template<typename K>
    QQmlPropertyData *findNamedProperty(const K &key)
    {
        StringCache::mapped_type *it = stringCache.value(key);
        return it ? it->second : 0;
    }

    template<typename K>
    void setNamedProperty(const K &key, int index, QQmlPropertyData *data, bool isOverride)
    {
        stringCache.insert(key, qMakePair(index, data));
        _hasPropertyOverrides |= isOverride;
    }

    QJSEngine *engine;

    QQmlPropertyCache *_parent;
    int propertyIndexCacheStart;
    int methodIndexCacheStart;
    int signalHandlerIndexCacheStart;

    IndexCache propertyIndexCache;
    IndexCache methodIndexCache;
    IndexCache signalHandlerIndexCache;
    StringCache stringCache;
    AllowedRevisionCache allowedRevisionCache;

    bool _hasPropertyOverrides : 1;
    bool _ownMetaObject : 1;
    const QMetaObject *_metaObject;
    QByteArray _dynamicClassName;
    QByteArray _dynamicStringData;
    QString _defaultPropertyName;
    QQmlPropertyCacheMethodArguments *argumentsCache;
};

// QQmlMetaObject serves as a wrapper around either QMetaObject or QQmlPropertyCache.
// This is necessary as we delay creation of QMetaObject for synthesized QObjects, but
// we don't want to needlessly generate QQmlPropertyCaches every time we encounter a
// QObject type used in assignment or when we don't have a QQmlEngine etc.
//
// This class does NOT reference the propertycache.
class QQmlEnginePrivate;
class Q_QML_EXPORT QQmlMetaObject
{
public:
    inline QQmlMetaObject();
    inline QQmlMetaObject(QObject *);
    inline QQmlMetaObject(const QMetaObject *);
    inline QQmlMetaObject(QQmlPropertyCache *);
    inline QQmlMetaObject(const QQmlMetaObject &);

    inline QQmlMetaObject &operator=(const QQmlMetaObject &);

    inline bool isNull() const;

    inline const char *className() const;
    inline int propertyCount() const;

    inline bool hasMetaObject() const;
    inline const QMetaObject *metaObject() const;

    QQmlPropertyCache *propertyCache(QQmlEnginePrivate *) const;

    int methodReturnType(const QQmlPropertyData &data, QByteArray *unknownTypeError) const;
    int *methodParameterTypes(int index, QVarLengthArray<int, 9> &dummy, QByteArray *unknownTypeError) const;

    static bool canConvert(const QQmlMetaObject &from, const QQmlMetaObject &to);

    // static_metacall (on Gadgets) doesn't call the base implementation and therefore
    // we need a helper to find the correct meta object and property/method index.
    static void resolveGadgetMethodOrPropertyIndex(QMetaObject::Call type, const QMetaObject **metaObject, int *index);

protected:
    QBiPointer<QQmlPropertyCache, const QMetaObject> _m;
};

class QQmlObjectOrGadget: public QQmlMetaObject
{
public:
    QQmlObjectOrGadget(QObject *obj)
        : QQmlMetaObject(obj),
          ptr(obj)
    {}
    QQmlObjectOrGadget(QQmlPropertyCache *propertyCache, void *gadget)
        : QQmlMetaObject(propertyCache)
        , ptr(gadget)
    {}

    void metacall(QMetaObject::Call type, int index, void **argv) const;

private:
    QBiPointer<QObject, void> ptr;
};

QQmlPropertyData::QQmlPropertyData()
{
    propType = 0;
    coreIndex = -1;
    notifyIndex = -1;
    overrideIndexIsProperty = false;
    overrideIndex = -1;
    revision = 0;
    metaObjectOffset = -1;
    flags = 0;
}

QQmlPropertyData::QQmlPropertyData(const QQmlPropertyRawData &d)
{
    *(static_cast<QQmlPropertyRawData *>(this)) = d;
}

bool QQmlPropertyData::operator==(const QQmlPropertyRawData &other)
{
    return flags == other.flags &&
           propType == other.propType &&
           coreIndex == other.coreIndex &&
           notifyIndex == other.notifyIndex &&
           revision == other.revision &&
           (!isValueTypeVirtual() ||
            (valueTypeCoreIndex == other.valueTypeCoreIndex &&
             valueTypePropType == other.valueTypePropType));
}

int QQmlPropertyRawData::getValueTypeCoreIndex() const
{
    return isValueTypeVirtual()?valueTypeCoreIndex:-1;
}

int QQmlPropertyRawData::encodedIndex() const
{
    return isValueTypeVirtual()?QQmlPropertyData::encodeValueTypePropertyIndex(coreIndex, valueTypeCoreIndex):coreIndex;
}

QQmlPropertyData *
QQmlPropertyCache::overrideData(QQmlPropertyData *data) const
{
    if (!data->hasOverride())
        return 0;

    if (data->overrideIndexIsProperty)
        return property(data->overrideIndex);
    else
        return method(data->overrideIndex);
}

bool QQmlPropertyCache::isAllowedInRevision(QQmlPropertyData *data) const
{
    return (data->hasAccessors() || (data->metaObjectOffset == -1 && data->revision == 0)) ||
           (allowedRevisionCache[data->metaObjectOffset] >= data->revision);
}

int QQmlPropertyCache::propertyCount() const
{
    return propertyIndexCacheStart + propertyIndexCache.count();
}

int QQmlPropertyCache::propertyOffset() const
{
    return propertyIndexCacheStart;
}

int QQmlPropertyCache::methodCount() const
{
    return methodIndexCacheStart + methodIndexCache.count();
}

int QQmlPropertyCache::methodOffset() const
{
    return methodIndexCacheStart;
}

int QQmlPropertyCache::signalCount() const
{
    return signalHandlerIndexCacheStart + signalHandlerIndexCache.count();
}

int QQmlPropertyCache::signalOffset() const
{
    return signalHandlerIndexCacheStart;
}

QQmlMetaObject::QQmlMetaObject()
{
}

QQmlMetaObject::QQmlMetaObject(QObject *o)
{
    if (o) {
        QQmlData *ddata = QQmlData::get(o, false);
        if (ddata && ddata->propertyCache) _m = ddata->propertyCache;
        else _m = o->metaObject();
    }
}

QQmlMetaObject::QQmlMetaObject(const QMetaObject *m)
: _m(m)
{
}

QQmlMetaObject::QQmlMetaObject(QQmlPropertyCache *m)
: _m(m)
{
}

QQmlMetaObject::QQmlMetaObject(const QQmlMetaObject &o)
: _m(o._m)
{
}

QQmlMetaObject &QQmlMetaObject::operator=(const QQmlMetaObject &o)
{
    _m = o._m;
    return *this;
}

bool QQmlMetaObject::isNull() const
{
    return _m.isNull();
}

const char *QQmlMetaObject::className() const
{
    if (_m.isNull()) {
        return 0;
    } else if (_m.isT1()) {
        return _m.asT1()->className();
    } else {
        return _m.asT2()->className();
    }
}

int QQmlMetaObject::propertyCount() const
{
    if (_m.isNull()) {
        return 0;
    } else if (_m.isT1()) {
        return _m.asT1()->propertyCount();
    } else {
        return _m.asT2()->propertyCount();
    }
}

bool QQmlMetaObject::hasMetaObject() const
{
    return _m.isT2() || (!_m.isNull() && _m.asT1()->metaObject());
}

const QMetaObject *QQmlMetaObject::metaObject() const
{
    if (_m.isNull()) return 0;
    if (_m.isT1()) return _m.asT1()->createMetaObject();
    else return _m.asT2();
}

QT_END_NAMESPACE

#endif // QQMLPROPERTYCACHE_P_H
