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
#ifndef QQMLOBJECTCREATOR_P_H
#define QQMLOBJECTCREATOR_P_H

#include <private/qqmlimport_p.h>
#include <private/qqmltypenamecache_p.h>
#include <private/qv4compileddata_p.h>
#include <private/qqmlcompiler_p.h>
#include <private/qqmltypecompiler_p.h>
#include <private/qfinitestack_p.h>
#include <private/qrecursionwatcher_p.h>
#include <private/qqmlprofiler_p.h>

#include <qpointer.h>

QT_BEGIN_NAMESPACE

class QQmlAbstractBinding;
struct QQmlTypeCompiler;
class QQmlInstantiationInterrupt;
struct QQmlVmeProfiler;

struct QQmlObjectCreatorSharedState : public QSharedData
{
    QQmlContextData *rootContext;
    QQmlContextData *creationContext;
    QFiniteStack<QQmlAbstractBinding*> allCreatedBindings;
    QFiniteStack<QQmlParserStatus*> allParserStatusCallbacks;
    QFiniteStack<QPointer<QObject> > allCreatedObjects;
    QV4::Value *allJavaScriptObjects; // pointer to vector on JS stack to reference JS wrappers during creation phase.
    QQmlComponentAttached *componentAttached;
    QList<QQmlEnginePrivate::FinalizeCallback> finalizeCallbacks;
    QQmlVmeProfiler profiler;
    QRecursionNode recursionNode;
};

class QQmlObjectCreator
{
    Q_DECLARE_TR_FUNCTIONS(QQmlObjectCreator)
public:
    QQmlObjectCreator(QQmlContextData *parentContext, QQmlCompiledData *compiledData, QQmlContextData *creationContext, void *activeVMEDataForRootContext = 0);
    ~QQmlObjectCreator();

    QObject *create(int subComponentIndex = -1, QObject *parent = 0, QQmlInstantiationInterrupt *interrupt = 0);
    bool populateDeferredProperties(QObject *instance);
    QQmlContextData *finalize(QQmlInstantiationInterrupt &interrupt);
    void clear();

    QQmlComponentAttached **componentAttachment() { return &sharedState->componentAttached; }

    QList<QQmlEnginePrivate::FinalizeCallback> *finalizeCallbacks() { return &sharedState->finalizeCallbacks; }

    QList<QQmlError> errors;

    QQmlContextData *parentContextData() { return parentContext.contextData(); }
    QFiniteStack<QPointer<QObject> > &allCreatedObjects() const { return sharedState->allCreatedObjects; }

private:
    QQmlObjectCreator(QQmlContextData *contextData, QQmlCompiledData *compiledData, QQmlObjectCreatorSharedState *inheritedSharedState);

    void init(QQmlContextData *parentContext);

    QObject *createInstance(int index, QObject *parent = 0, bool isContextObject = false);

    bool populateInstance(int index, QObject *instance,
                          QObject *bindingTarget, const QQmlPropertyData *valueTypeProperty,
                          const QBitArray &bindingsToSkip = QBitArray());

    void setupBindings(const QBitArray &bindingsToSkip);
    bool setPropertyBinding(const QQmlPropertyData *property, const QV4::CompiledData::Binding *binding);
    void setPropertyValue(const QQmlPropertyData *property, const QV4::CompiledData::Binding *binding);
    void setupFunctions();

    QString stringAt(int idx) const { return qmlUnit->stringAt(idx); }
    void recordError(const QV4::CompiledData::Location &location, const QString &description);

    void registerObjectWithContextById(int objectIndex, QObject *instance) const;

    QV4::Heap::ExecutionContext *currentQmlContext();

    enum Phase {
        Startup,
        CreatingObjects,
        CreatingObjectsPhase2,
        ObjectsCreated,
        Finalizing,
        Done
    } phase;

    QQmlEngine *engine;
    QV4::ExecutionEngine *v4;
    QQmlCompiledData *compiledData;
    const QV4::CompiledData::Unit *qmlUnit;
    QQmlGuardedContextData parentContext;
    QQmlContextData *context;
    const QHash<int, QQmlCompiledData::TypeReference*> &resolvedTypes;
    const QVector<QQmlPropertyCache *> &propertyCaches;
    const QVector<QByteArray> &vmeMetaObjectData;
    QHash<int, int> objectIndexToId;
    QExplicitlySharedDataPointer<QQmlObjectCreatorSharedState> sharedState;
    bool topLevelCreator;
    void *activeVMEDataForRootContext;

    QObject *_qobject;
    QObject *_scopeObject;
    QObject *_bindingTarget;

    const QQmlPropertyData *_valueTypeProperty; // belongs to _qobjectForBindings's property cache
    int _compiledObjectIndex;
    const QV4::CompiledData::Object *_compiledObject;
    QQmlData *_ddata;
    QQmlRefPointer<QQmlPropertyCache> _propertyCache;
    QQmlVMEMetaObject *_vmeMetaObject;
    QQmlListProperty<void> _currentList;
    QV4::Value *_qmlBindingWrapper;

    friend struct QQmlObjectCreatorRecursionWatcher;
};

struct QQmlObjectCreatorRecursionWatcher
{
    QQmlObjectCreatorRecursionWatcher(QQmlObjectCreator *creator);

    bool hasRecursed() const { return watcher.hasRecursed(); }

private:
    QExplicitlySharedDataPointer<QQmlObjectCreatorSharedState> sharedState;
    QRecursionWatcher<QQmlObjectCreatorSharedState, &QQmlObjectCreatorSharedState::recursionNode> watcher;
};

QT_END_NAMESPACE

#endif // QQMLOBJECTCREATOR_P_H
