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
#ifndef QV4COMPILEDDATA_P_H
#define QV4COMPILEDDATA_P_H

#include <QtCore/qstring.h>
#include <QVector>
#include <QStringList>
#include <QHash>
#include <QUrl>

#include <private/qv4value_p.h>
#include <private/qv4executableallocator_p.h>
#include <private/qqmlrefcount_p.h>
#include <private/qqmlnullablevalue_p.h>

QT_BEGIN_NAMESPACE

class QQmlPropertyCache;
class QQmlPropertyData;

namespace QmlIR {
struct Document;
}

namespace QV4 {
namespace IR {
struct Function;
}

struct Function;

namespace CompiledData {

struct String;
struct Function;
struct Lookup;
struct RegExp;
struct Unit;

#if defined(Q_CC_MSVC) || defined(Q_CC_GNU)
#pragma pack(push, 1)
#endif

struct Location
{
    qint32 line;
    qint32 column;

    Location(): line(-1), column(-1) {}

    inline bool operator<(const Location &other) const {
        return line < other.line ||
               (line == other.line && column < other.column);
    }
};

struct RegExp
{
    enum Flags {
        RegExp_Global     = 0x01,
        RegExp_IgnoreCase = 0x02,
        RegExp_Multiline  = 0x04
    };
    quint32 flags;
    quint32 stringIndex;

    static int calculateSize() { return sizeof(RegExp); }
};

struct Lookup
{
    enum Type {
        Type_Getter = 0x0,
        Type_Setter = 0x1,
        Type_GlobalGetter = 2,
        Type_IndexedGetter = 3,
        Type_IndexedSetter = 4
    };

    quint32 type_and_flags;
    quint32 nameIndex;

    static int calculateSize() { return sizeof(Lookup); }
};

struct JSClassMember
{
    uint nameOffset : 31;
    uint isAccessor : 1;
};

struct JSClass
{
    uint nMembers;
    // JSClassMember[nMembers]

    static int calculateSize(int nMembers) { return (sizeof(JSClass) + nMembers * sizeof(JSClassMember) + 7) & ~7; }
};

struct String
{
    quint32 flags; // isArrayIndex
    qint32 size;
    // uint16 strdata[]

    static int calculateSize(const QString &str) {
        return (sizeof(String) + str.length() * sizeof(quint16) + 7) & ~0x7;
    }
};

struct Function
{
    enum Flags {
        HasDirectEval       = 0x1,
        UsesArgumentsObject = 0x2,
        IsStrict            = 0x4,
        IsNamedExpression   = 0x8,
        HasCatchOrWith      = 0x10
    };

    quint32 index; // in CompilationUnit's function table
    quint32 nameIndex;
    qint64 flags;
    quint32 nFormals;
    quint32 formalsOffset;
    quint32 nLocals;
    quint32 localsOffset;
    quint32 nInnerFunctions;
    quint32 innerFunctionsOffset;
    Location location;

    // Qml Extensions Begin
    quint32 nDependingIdObjects;
    quint32 dependingIdObjectsOffset; // Array of resolved ID objects
    quint32 nDependingContextProperties;
    quint32 dependingContextPropertiesOffset; // Array of int pairs (property index and notify index)
    quint32 nDependingScopeProperties;
    quint32 dependingScopePropertiesOffset; // Array of int pairs (property index and notify index)
    // Qml Extensions End

//    quint32 formalsIndex[nFormals]
//    quint32 localsIndex[nLocals]
//    quint32 offsetForInnerFunctions[nInnerFunctions]
//    Function[nInnerFunctions]

    const quint32 *formalsTable() const { return reinterpret_cast<const quint32 *>(reinterpret_cast<const char *>(this) + formalsOffset); }
    const quint32 *localsTable() const { return reinterpret_cast<const quint32 *>(reinterpret_cast<const char *>(this) + localsOffset); }
    const quint32 *qmlIdObjectDependencyTable() const { return reinterpret_cast<const quint32 *>(reinterpret_cast<const char *>(this) + dependingIdObjectsOffset); }
    const quint32 *qmlContextPropertiesDependencyTable() const { return reinterpret_cast<const quint32 *>(reinterpret_cast<const char *>(this) + dependingContextPropertiesOffset); }
    const quint32 *qmlScopePropertiesDependencyTable() const { return reinterpret_cast<const quint32 *>(reinterpret_cast<const char *>(this) + dependingScopePropertiesOffset); }

    inline bool hasQmlDependencies() const { return nDependingIdObjects > 0 || nDependingContextProperties > 0 || nDependingScopeProperties > 0; }

    static int calculateSize(int nFormals, int nLocals, int nInnerfunctions, int nIdObjectDependencies, int nPropertyDependencies) {
        return (sizeof(Function) + (nFormals + nLocals + nInnerfunctions + nIdObjectDependencies + 2 * nPropertyDependencies) * sizeof(quint32) + 7) & ~0x7;
    }
};

// Qml data structures

struct Q_QML_EXPORT TranslationData {
    quint32 commentIndex;
    int number;
};

struct Q_QML_PRIVATE_EXPORT Binding
{
    quint32 propertyNameIndex;

    enum ValueType {
        Type_Invalid,
        Type_Boolean,
        Type_Number,
        Type_String,
        Type_Translation,
        Type_TranslationById,
        Type_Script,
        Type_Object,
        Type_AttachedProperty,
        Type_GroupProperty
    };

    enum Flags {
        IsSignalHandlerExpression = 0x1,
        IsSignalHandlerObject = 0x2,
        IsOnAssignment = 0x4,
        InitializerForReadOnlyDeclaration = 0x8,
        IsResolvedEnum = 0x10,
        IsListItem = 0x20,
        IsBindingToAlias = 0x40
    };

    quint32 flags : 16;
    quint32 type : 16;
    union {
        bool b;
        double d;
        quint32 compiledScriptIndex; // used when Type_Script
        quint32 objectIndex;
        TranslationData translationData; // used when Type_Translation
    } value;
    quint32 stringIndex; // Set for Type_String, Type_Translation and Type_Script (the latter because of script strings)

    Location location;
    Location valueLocation;

    bool isValueBinding() const
    {
        if (type == Type_AttachedProperty
            || type == Type_GroupProperty)
            return false;
        if (flags & IsSignalHandlerExpression
            || flags & IsSignalHandlerObject)
            return false;
        return true;
    }

    bool isValueBindingNoAlias() const { return isValueBinding() && !(flags & IsBindingToAlias); }
    bool isValueBindingToAlias() const { return isValueBinding() && (flags & IsBindingToAlias); }

    bool isSignalHandler() const
    {
        if (flags & IsSignalHandlerExpression || flags & IsSignalHandlerObject) {
            Q_ASSERT(!isValueBinding());
            Q_ASSERT(!isAttachedProperty());
            Q_ASSERT(!isGroupProperty());
            return true;
        }
        return false;
    }

    bool isAttachedProperty() const
    {
        if (type == Type_AttachedProperty) {
            Q_ASSERT(!isValueBinding());
            Q_ASSERT(!isSignalHandler());
            Q_ASSERT(!isGroupProperty());
            return true;
        }
        return false;
    }

    bool isGroupProperty() const
    {
        if (type == Type_GroupProperty) {
            Q_ASSERT(!isValueBinding());
            Q_ASSERT(!isSignalHandler());
            Q_ASSERT(!isAttachedProperty());
            return true;
        }
        return false;
    }

    static QString escapedString(const QString &string);

    bool evaluatesToString() const { return type == Type_String || type == Type_Translation || type == Type_TranslationById; }

    QString valueAsString(const Unit *unit) const;
    QString valueAsScriptString(const Unit *unit) const;
    double valueAsNumber() const
    {
        if (type == Type_Number)
            return value.d;
        return 0.0;

    }
    bool valueAsBoolean() const
    {
        if (type == Type_Boolean)
            return value.b;
        return false;
    }

};

struct Parameter
{
    quint32 nameIndex;
    quint32 type;
    quint32 customTypeNameIndex;
    quint32 reserved;
    Location location;
};

struct Signal
{
    quint32 nameIndex;
    quint32 nParameters;
    Location location;
    // Parameter parameters[1];

    const Parameter *parameterAt(int idx) const {
        return reinterpret_cast<const Parameter*>(this + 1) + idx;
    }

    static int calculateSize(int nParameters) {
        return (sizeof(Signal)
                + nParameters * sizeof(Parameter)
                + 7) & ~0x7;
    }
};

struct Property
{
    enum Type { Var = 0, Variant, Int, Bool, Real, String, Url, Color,
                Font, Time, Date, DateTime, Rect, Point, Size,
                Vector2D, Vector3D, Vector4D, Matrix4x4, Quaternion,
                Alias, Custom, CustomList };

    enum Flags {
        IsReadOnly = 0x1
    };

    quint32 nameIndex;
    quint32 type;
    union {
        quint32 customTypeNameIndex; // If type >= Custom
        quint32 aliasIdValueIndex; // If type == Alias
    };
    quint32 aliasPropertyValueIndex;
    quint32 flags; // readonly
    Location location;
    Location aliasLocation; // If type == Alias
};

struct Object
{
    // Depending on the use, this may be the type name to instantiate before instantiating this
    // object. For grouped properties the type name will be empty and for attached properties
    // it will be the name of the attached type.
    quint32 inheritedTypeNameIndex;
    quint32 idIndex;
    qint32 indexOfDefaultProperty; // -1 means no default property declared in this object
    quint32 nFunctions;
    quint32 offsetToFunctions;
    quint32 nProperties;
    quint32 offsetToProperties;
    quint32 nSignals;
    quint32 offsetToSignals; // which in turn will be a table with offsets to variable-sized Signal objects
    quint32 nBindings;
    quint32 offsetToBindings;
    Location location;
    Location locationOfIdProperty;
//    Function[]
//    Property[]
//    Signal[]
//    Binding[]

    static int calculateSizeExcludingSignals(int nFunctions, int nProperties, int nSignals, int nBindings)
    {
        return ( sizeof(Object)
                 + nFunctions * sizeof(quint32)
                 + nProperties * sizeof(Property)
                 + nSignals * sizeof(quint32)
                 + nBindings * sizeof(Binding)
                 + 0x7
               ) & ~0x7;
    }

    const quint32 *functionOffsetTable() const
    {
        return reinterpret_cast<const quint32*>(reinterpret_cast<const char *>(this) + offsetToFunctions);
    }

    const Property *propertyTable() const
    {
        return reinterpret_cast<const Property*>(reinterpret_cast<const char *>(this) + offsetToProperties);
    }

    const Binding *bindingTable() const
    {
        return reinterpret_cast<const Binding*>(reinterpret_cast<const char *>(this) + offsetToBindings);
    }

    const Signal *signalAt(int idx) const
    {
        const uint *offsetTable = reinterpret_cast<const uint*>((reinterpret_cast<const char *>(this)) + offsetToSignals);
        const uint offset = offsetTable[idx];
        return reinterpret_cast<const Signal*>(reinterpret_cast<const char*>(this) + offset);
    }
};

struct Import
{
    enum ImportType {
        ImportLibrary = 0x1,
        ImportFile = 0x2,
        ImportScript = 0x3
    };
    quint32 type;

    quint32 uriIndex;
    quint32 qualifierIndex;

    qint32 majorVersion;
    qint32 minorVersion;

    Location location;

    Import(): type(0), uriIndex(0), qualifierIndex(0), majorVersion(0), minorVersion(0) {}
};

static const char magic_str[] = "qv4cdata";

struct Unit
{
    char magic[8];
    qint16 architecture;
    qint16 version;
    quint32 unitSize; // Size of the Unit and any depending data.

    enum {
        IsJavascript = 0x1,
        IsQml = 0x2,
        StaticData = 0x4, // Unit data persistent in memory?
        IsSingleton = 0x8,
        IsSharedLibrary = 0x10 // .pragma shared?
    };
    quint32 flags;
    uint stringTableSize;
    uint offsetToStringTable;
    uint functionTableSize;
    uint offsetToFunctionTable;
    uint lookupTableSize;
    uint offsetToLookupTable;
    uint regexpTableSize;
    uint offsetToRegexpTable;
    uint constantTableSize;
    uint offsetToConstantTable;
    uint jsClassTableSize;
    uint offsetToJSClassTable;
    qint32 indexOfRootFunction;
    quint32 sourceFileIndex;

    /* QML specific fields */
    quint32 nImports;
    quint32 offsetToImports;
    quint32 nObjects;
    quint32 offsetToObjects;
    quint32 indexOfRootObject;

    const Import *importAt(int idx) const {
        return reinterpret_cast<const Import*>((reinterpret_cast<const char *>(this)) + offsetToImports + idx * sizeof(Import));
    }

    const Object *objectAt(int idx) const {
        const uint *offsetTable = reinterpret_cast<const uint*>((reinterpret_cast<const char *>(this)) + offsetToObjects);
        const uint offset = offsetTable[idx];
        return reinterpret_cast<const Object*>(reinterpret_cast<const char*>(this) + offset);
    }

    bool isSingleton() const {
        return flags & Unit::IsSingleton;
    }
    /* end QML specific fields*/

    QString stringAt(int idx) const {
        const uint *offsetTable = reinterpret_cast<const uint*>((reinterpret_cast<const char *>(this)) + offsetToStringTable);
        const uint offset = offsetTable[idx];
        const String *str = reinterpret_cast<const String*>(reinterpret_cast<const char *>(this) + offset);
        if (str->size == 0)
            return QString();
        const QChar *characters = reinterpret_cast<const QChar *>(str + 1);
        if (flags & StaticData)
            return QString::fromRawData(characters, str->size);
        return QString(characters, str->size);
    }

    const uint *functionOffsetTable() const { return reinterpret_cast<const uint*>((reinterpret_cast<const char *>(this)) + offsetToFunctionTable); }

    const Function *functionAt(int idx) const {
        const uint *offsetTable = functionOffsetTable();
        const uint offset = offsetTable[idx];
        return reinterpret_cast<const Function*>(reinterpret_cast<const char *>(this) + offset);
    }

    const Lookup *lookupTable() const { return reinterpret_cast<const Lookup*>(reinterpret_cast<const char *>(this) + offsetToLookupTable); }
    const RegExp *regexpAt(int index) const {
        return reinterpret_cast<const RegExp*>(reinterpret_cast<const char *>(this) + offsetToRegexpTable + index * sizeof(RegExp));
    }
    const QV4::Value *constants() const {
        return reinterpret_cast<const QV4::Value*>(reinterpret_cast<const char *>(this) + offsetToConstantTable);
    }

    const JSClassMember *jsClassAt(int idx, int *nMembers) const {
        const uint *offsetTable = reinterpret_cast<const uint *>(reinterpret_cast<const char *>(this) + offsetToJSClassTable);
        const uint offset = offsetTable[idx];
        const char *ptr = reinterpret_cast<const char *>(this) + offset;
        const JSClass *klass = reinterpret_cast<const JSClass *>(ptr);
        *nMembers = klass->nMembers;
        return reinterpret_cast<const JSClassMember*>(ptr + sizeof(JSClass));
    }

    static int calculateSize(uint nFunctions, uint nRegExps, uint nConstants,
                             uint nLookups, uint nClasses) {
        return (sizeof(Unit)
                + (nFunctions + nClasses) * sizeof(uint)
                + nRegExps * RegExp::calculateSize()
                + nConstants * sizeof(QV4::ReturnedValue)
                + nLookups * Lookup::calculateSize()
                + 7) & ~7; }
};

#if defined(Q_CC_MSVC) || defined(Q_CC_GNU)
#pragma pack(pop)
#endif

struct TypeReference
{
    TypeReference(const Location &loc)
        : location(loc)
        , needsCreation(false)
        , errorWhenNotFound(false)
    {}
    Location location; // first use
    bool needsCreation : 1; // whether the type needs to be creatable or not
    bool errorWhenNotFound: 1;
};

// map from name index to location of first use
struct TypeReferenceMap : QHash<int, TypeReference>
{
    TypeReference &add(int nameIndex, const Location &loc) {
        Iterator it = find(nameIndex);
        if (it != end())
            return *it;
        return *insert(nameIndex, loc);
    }
};

// index is per-object binding index
typedef QVector<QQmlPropertyData*> BindingPropertyData;

// This is how this hooks into the existing structures:

//VM::Function
//    CompilationUnit * (for functions that need to clean up)
//    CompiledData::Function *compiledFunction

struct Q_QML_PRIVATE_EXPORT CompilationUnit : public QQmlRefCount
{
#ifdef V4_BOOTSTRAP
    CompilationUnit()
        : data(0)
    {}
    virtual ~CompilationUnit() {}
#else
    CompilationUnit();
    virtual ~CompilationUnit();
#endif

    Unit *data;

    // Called only when building QML, when we build the header for JS first and append QML data
    virtual QV4::CompiledData::Unit *createUnitData(QmlIR::Document *irDocument);

#ifndef V4_BOOTSTRAP
    ExecutionEngine *engine;
    QString fileName() const { return data->stringAt(data->sourceFileIndex); }
    QUrl url() const { if (m_url.isNull) m_url = QUrl(fileName()); return m_url; }

    QV4::Heap::String **runtimeStrings; // Array
    QV4::Lookup *runtimeLookups;
    QV4::Value *runtimeRegularExpressions;
    QV4::InternalClass **runtimeClasses;
    QVector<QV4::Function *> runtimeFunctions;
    mutable QQmlNullableValue<QUrl> m_url;

    // index is object index. This allows fast access to the
    // property data when initializing bindings, avoiding expensive
    // lookups by string (property name).
    QVector<BindingPropertyData> bindingPropertyDataPerObject;

    QV4::Function *linkToEngine(QV4::ExecutionEngine *engine);
    void unlink();

    virtual QV4::ExecutableAllocator::ChunkOfPages *chunkForFunction(int /*functionIndex*/) { return 0; }

    void markObjects(QV4::ExecutionEngine *e);

protected:
    virtual void linkBackendToEngine(QV4::ExecutionEngine *engine) = 0;
#endif // V4_BOOTSTRAP
};

}

}

Q_DECLARE_TYPEINFO(QV4::CompiledData::JSClassMember, Q_PRIMITIVE_TYPE);

QT_END_NAMESPACE

#endif
