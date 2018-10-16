/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the tools applications of the Qt Toolkit.
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
#ifndef QQMLTYPECOMPILER_P_H
#define QQMLTYPECOMPILER_P_H

#include <qglobal.h>
#include <qqmlerror.h>
#include <qhash.h>
#include <private/qqmlcompiler_p.h>
#include <private/qqmlirbuilder_p.h>

QT_BEGIN_NAMESPACE

class QQmlEnginePrivate;
class QQmlCompiledData;
class QQmlError;
class QQmlTypeData;
class QQmlImports;

namespace QmlIR {
struct Document;
}

namespace QV4 {
namespace CompiledData {
struct QmlUnit;
struct Location;
}
}

struct QQmlTypeCompiler
{
    Q_DECLARE_TR_FUNCTIONS(QQmlTypeCompiler)
public:
    QQmlTypeCompiler(QQmlEnginePrivate *engine, QQmlCompiledData *compiledData, QQmlTypeData *typeData, QmlIR::Document *document);

    bool compile();

    QList<QQmlError> compilationErrors() const { return errors; }
    void recordError(const QQmlError &error);

    QString stringAt(int idx) const;
    int registerString(const QString &str);

    QV4::IR::Module *jsIRModule() const;

    const QV4::CompiledData::Unit *qmlUnit() const;

    QUrl url() const { return typeData->finalUrl(); }
    QQmlEnginePrivate *enginePrivate() const { return engine; }
    const QQmlImports *imports() const;
    QHash<int, QQmlCompiledData::TypeReference *> *resolvedTypes();
    QList<QmlIR::Object*> *qmlObjects();
    int rootObjectIndex() const;
    void setPropertyCaches(const QVector<QQmlPropertyCache *> &caches);
    const QVector<QQmlPropertyCache *> &propertyCaches() const;
    void setVMEMetaObjects(const QVector<QByteArray> &metaObjects);
    QVector<QByteArray> *vmeMetaObjects() const;
    QHash<int, int> *objectIndexToIdForRoot();
    QHash<int, QHash<int, int> > *objectIndexToIdPerComponent();
    QHash<int, QBitArray> *customParserBindings();
    QQmlJS::MemoryPool *memoryPool();
    QStringRef newStringRef(const QString &string);
    const QV4::Compiler::StringTableGenerator *stringPool() const;
    void setDeferredBindingsPerObject(const QHash<int, QBitArray> &deferredBindingsPerObject);
    void setBindingPropertyDataPerObject(const QVector<QV4::CompiledData::BindingPropertyData> &propertyData);

    const QHash<int, QQmlCustomParser*> &customParserCache() const { return customParsers; }

    QString bindingAsString(const QmlIR::Object *object, int scriptIndex) const;

private:
    QList<QQmlError> errors;
    QQmlEnginePrivate *engine;
    QQmlCompiledData *compiledData;
    QQmlTypeData *typeData;
    QmlIR::Document *document;
    // index is string index of type name (use obj->inheritedTypeNameIndex)
    QHash<int, QQmlCustomParser*> customParsers;
};

struct QQmlCompilePass
{
    virtual ~QQmlCompilePass() {}

    QQmlCompilePass(QQmlTypeCompiler *typeCompiler);

    QString stringAt(int idx) const { return compiler->stringAt(idx); }
protected:
    void recordError(const QV4::CompiledData::Location &location, const QString &description) const;

    QQmlTypeCompiler *compiler;
};

class QQmlPropertyCacheCreator : public QQmlCompilePass
{
    Q_DECLARE_TR_FUNCTIONS(QQmlPropertyCacheCreator)
public:
    QQmlPropertyCacheCreator(QQmlTypeCompiler *typeCompiler);
    ~QQmlPropertyCacheCreator();

    bool buildMetaObjects();
protected:
    bool buildMetaObjectRecursively(int objectIndex, int referencingObjectIndex, const QV4::CompiledData::Binding *instantiatingBinding);
    bool ensureMetaObject(int objectIndex);
    bool createMetaObject(int objectIndex, const QmlIR::Object *obj, QQmlPropertyCache *baseTypeCache);

    QQmlEnginePrivate *enginePrivate;
    const QList<QmlIR::Object*> &qmlObjects;
    const QQmlImports *imports;
    QHash<int, QQmlCompiledData::TypeReference*> *resolvedTypes;
    QVector<QByteArray> vmeMetaObjects;
    QVector<QQmlPropertyCache*> propertyCaches;
};

// "Converts" signal expressions to full-fleged function declarations with
// parameters taken from the signal declarations
// It also updates the QV4::CompiledData::Binding objects to set the property name
// to the final signal name (onTextChanged -> textChanged) and sets the IsSignalExpression flag.
struct SignalHandlerConverter : public QQmlCompilePass
{
    Q_DECLARE_TR_FUNCTIONS(SignalHandlerConverter)
public:
    SignalHandlerConverter(QQmlTypeCompiler *typeCompiler);

    bool convertSignalHandlerExpressionsToFunctionDeclarations();

private:
    bool convertSignalHandlerExpressionsToFunctionDeclarations(const QmlIR::Object *obj, const QString &typeName, QQmlPropertyCache *propertyCache);

    const QList<QmlIR::Object*> &qmlObjects;
    const QHash<int, QQmlCustomParser*> &customParsers;
    const QHash<int, QQmlCompiledData::TypeReference*> &resolvedTypes;
    const QSet<QString> &illegalNames;
    const QVector<QQmlPropertyCache*> &propertyCaches;
};

// ### This will go away when the codegen resolves all enums to constant expressions
// and we replace the constant expression with a literal binding instead of using
// a script.
class QQmlEnumTypeResolver : public QQmlCompilePass
{
    Q_DECLARE_TR_FUNCTIONS(QQmlEnumTypeResolver)
public:
    QQmlEnumTypeResolver(QQmlTypeCompiler *typeCompiler);

    bool resolveEnumBindings();

private:
    bool tryQualifiedEnumAssignment(const QmlIR::Object *obj, const QQmlPropertyCache *propertyCache,
                                    const QQmlPropertyData *prop,
                                    QmlIR::Binding *binding);
    int evaluateEnum(const QString &scope, const QByteArray &enumValue, bool *ok) const;


    const QList<QmlIR::Object*> &qmlObjects;
    const QVector<QQmlPropertyCache *> propertyCaches;
    const QQmlImports *imports;
    QHash<int, QQmlCompiledData::TypeReference *> *resolvedTypes;
};

class QQmlCustomParserScriptIndexer: public QQmlCompilePass
{
public:
    QQmlCustomParserScriptIndexer(QQmlTypeCompiler *typeCompiler);

    void annotateBindingsWithScriptStrings();

private:
    void scanObjectRecursively(int objectIndex, bool annotateScriptBindings = false);

    const QList<QmlIR::Object*> &qmlObjects;
    const QHash<int, QQmlCustomParser*> &customParsers;
};

// Annotate properties bound to aliases with a flag
class QQmlAliasAnnotator : public QQmlCompilePass
{
public:
    QQmlAliasAnnotator(QQmlTypeCompiler *typeCompiler);

    void annotateBindingsToAliases();
private:
    const QList<QmlIR::Object*> &qmlObjects;
    const QVector<QQmlPropertyCache *> propertyCaches;
};

class QQmlScriptStringScanner : public QQmlCompilePass
{
public:
    QQmlScriptStringScanner(QQmlTypeCompiler *typeCompiler);

    void scan();

private:
    const QList<QmlIR::Object*> &qmlObjects;
    const QVector<QQmlPropertyCache *> propertyCaches;
};

class QQmlComponentAndAliasResolver : public QQmlCompilePass
{
    Q_DECLARE_TR_FUNCTIONS(QQmlAnonymousComponentResolver)
public:
    QQmlComponentAndAliasResolver(QQmlTypeCompiler *typeCompiler);

    bool resolve();

protected:
    void findAndRegisterImplicitComponents(const QmlIR::Object *obj, QQmlPropertyCache *propertyCache);
    bool collectIdsAndAliases(int objectIndex);
    bool resolveAliases();

    QQmlEnginePrivate *enginePrivate;
    QQmlJS::MemoryPool *pool;

    QList<QmlIR::Object*> *qmlObjects;
    const int indexOfRootObject;

    // indices of the objects that are actually Component {}
    QVector<int> componentRoots;
    // indices of objects that are the beginning of a new component
    // scope. This is sorted and used for binary search.
    QVector<quint32> componentBoundaries;

    int _componentIndex;
    QHash<int, int> _idToObjectIndex;
    QHash<int, int> *_objectIndexToIdInScope;
    QList<int> _objectsWithAliases;

    QHash<int, QQmlCompiledData::TypeReference*> *resolvedTypes;
    QVector<QQmlPropertyCache *> propertyCaches;
    QVector<QByteArray> *vmeMetaObjectData;
    QHash<int, int> *objectIndexToIdForRoot;
    QHash<int, QHash<int, int> > *objectIndexToIdPerComponent;
};

class QQmlPropertyValidator : public QQmlCompilePass, public QQmlCustomParserCompilerBackend
{
    Q_DECLARE_TR_FUNCTIONS(QQmlPropertyValidator)
public:
    QQmlPropertyValidator(QQmlTypeCompiler *typeCompiler);

    bool validate();

    // Re-implemented for QQmlCustomParser
    virtual const QQmlImports &imports() const;
    virtual QString bindingAsString(int objectIndex, const QV4::CompiledData::Binding *binding) const;

private:
    bool validateObject(int objectIndex, const QV4::CompiledData::Binding *instantiatingBinding, bool populatingValueTypeGroupProperty = false) const;
    bool validateLiteralBinding(QQmlPropertyCache *propertyCache, QQmlPropertyData *property, const QV4::CompiledData::Binding *binding) const;
    bool validateObjectBinding(QQmlPropertyData *property, const QString &propertyName, const QV4::CompiledData::Binding *binding) const;

    bool isComponent(int objectIndex) const { return objectIndexToIdPerComponent.contains(objectIndex); }

    bool canCoerce(int to, QQmlPropertyCache *fromMo) const;

    QQmlEnginePrivate *enginePrivate;
    const QV4::CompiledData::Unit *qmlUnit;
    const QHash<int, QQmlCompiledData::TypeReference*> &resolvedTypes;
    const QHash<int, QQmlCustomParser*> &customParsers;
    const QVector<QQmlPropertyCache *> &propertyCaches;
    const QHash<int, QHash<int, int> > objectIndexToIdPerComponent;
    QHash<int, QBitArray> *customParserBindingsPerObject;

    // collected state variables, essentially write-only
    mutable QHash<int, QBitArray> _deferredBindingsPerObject;
    mutable bool _seenObjectWithId;
    mutable QVector<QV4::CompiledData::BindingPropertyData> _bindingPropertyDataPerObject;
};

// ### merge with QtQml::JSCodeGen and operate directly on object->functionsAndExpressions once old compiler is gone.
class QQmlJSCodeGenerator : public QQmlCompilePass
{
public:
    QQmlJSCodeGenerator(QQmlTypeCompiler *typeCompiler, QmlIR::JSCodeGen *v4CodeGen);

    bool generateCodeForComponents();

private:
    bool compileComponent(int componentRoot, const QHash<int, int> &objectIndexToId);
    bool compileJavaScriptCodeInObjectsRecursively(int objectIndex, int scopeObjectIndex);

    bool isComponent(int objectIndex) const { return objectIndexToIdPerComponent.contains(objectIndex); }

    const QHash<int, QHash<int, int> > &objectIndexToIdPerComponent;
    const QHash<int, QQmlCompiledData::TypeReference*> &resolvedTypes;
    const QHash<int, QQmlCustomParser*> &customParsers;
    const QList<QmlIR::Object*> &qmlObjects;
    const QVector<QQmlPropertyCache *> &propertyCaches;
    QmlIR::JSCodeGen * const v4CodeGen;
};

class QQmlDefaultPropertyMerger : public QQmlCompilePass
{
public:
    QQmlDefaultPropertyMerger(QQmlTypeCompiler *typeCompiler);

    void mergeDefaultProperties();

private:
    void mergeDefaultProperties(int objectIndex);

    const QList<QmlIR::Object*> &qmlObjects;
    const QVector<QQmlPropertyCache*> &propertyCaches;
};

class QQmlJavaScriptBindingExpressionSimplificationPass : public QQmlCompilePass, public QV4::IR::StmtVisitor
{
public:
    QQmlJavaScriptBindingExpressionSimplificationPass(QQmlTypeCompiler *typeCompiler);

    void reduceTranslationBindings();

private:
    void reduceTranslationBindings(int objectIndex);

    virtual void visitMove(QV4::IR::Move *move);
    virtual void visitJump(QV4::IR::Jump *) {}
    virtual void visitCJump(QV4::IR::CJump *) { discard(); }
    virtual void visitExp(QV4::IR::Exp *) { discard(); }
    virtual void visitPhi(QV4::IR::Phi *) {}
    virtual void visitRet(QV4::IR::Ret *ret);

    void visitFunctionCall(const QString *name, QV4::IR::ExprList *args, QV4::IR::Temp *target);

    void discard() { _canSimplify = false; }

    bool simplifyBinding(QV4::IR::Function *function, QmlIR::Binding *binding);
    bool detectTranslationCallAndConvertBinding(QmlIR::Binding *binding);

    const QList<QmlIR::Object*> &qmlObjects;
    QV4::IR::Module *jsModule;

    bool _canSimplify;
    const QString *_nameOfFunctionCalled;
    QVector<int> _functionParameters;
    int _functionCallReturnValue;

    QHash<int, QV4::IR::Expr*> _temps;
    int _returnValueOfBindingExpression;
    int _synthesizedConsts;

    QVector<int> irFunctionsToRemove;
};

class QQmlIRFunctionCleanser : public QQmlCompilePass, public QV4::IR::StmtVisitor,
                               public QV4::IR::ExprVisitor
{
public:
    QQmlIRFunctionCleanser(QQmlTypeCompiler *typeCompiler, const QVector<int> &functionsToRemove);

    void clean();

private:
    virtual void visitClosure(QV4::IR::Closure *closure);

    virtual void visitTemp(QV4::IR::Temp *) {}
    virtual void visitArgLocal(QV4::IR::ArgLocal *) {}

    virtual void visitMove(QV4::IR::Move *s) {
        s->source->accept(this);
        s->target->accept(this);
    }

    virtual void visitConvert(QV4::IR::Convert *e) { e->expr->accept(this); }
    virtual void visitPhi(QV4::IR::Phi *) { }

    virtual void visitExp(QV4::IR::Exp *s) { s->expr->accept(this); }

    virtual void visitJump(QV4::IR::Jump *) {}
    virtual void visitCJump(QV4::IR::CJump *s) { s->cond->accept(this); }
    virtual void visitRet(QV4::IR::Ret *s) { s->expr->accept(this); }

    virtual void visitConst(QV4::IR::Const *) {}
    virtual void visitString(QV4::IR::String *) {}
    virtual void visitRegExp(QV4::IR::RegExp *) {}
    virtual void visitName(QV4::IR::Name *) {}
    virtual void visitUnop(QV4::IR::Unop *e) { e->expr->accept(this); }
    virtual void visitBinop(QV4::IR::Binop *e) { e->left->accept(this); e->right->accept(this); }
    virtual void visitCall(QV4::IR::Call *e) {
        e->base->accept(this);
        for (QV4::IR::ExprList *it = e->args; it; it = it->next)
            it->expr->accept(this);
    }

    virtual void visitNew(QV4::IR::New *e) {
        e->base->accept(this);
        for (QV4::IR::ExprList *it = e->args; it; it = it->next)
            it->expr->accept(this);
    }

    virtual void visitSubscript(QV4::IR::Subscript *e) {
        e->base->accept(this);
        e->index->accept(this);
    }

    virtual void visitMember(QV4::IR::Member *e) {
        e->base->accept(this);
    }

private:
    QV4::IR::Module *module;
    const QVector<int> &functionsToRemove;

    QVector<int> newFunctionIndices;
};

QT_END_NAMESPACE

#endif // QQMLTYPECOMPILER_P_H
