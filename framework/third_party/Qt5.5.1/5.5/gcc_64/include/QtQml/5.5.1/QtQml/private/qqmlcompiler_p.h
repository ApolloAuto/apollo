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

#ifndef QQMLCOMPILER_P_H
#define QQMLCOMPILER_P_H

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
#include "qqmlerror.h"
#include "qqmlengine_p.h"
#include <private/qbitfield_p.h>
#include "qqmlpropertycache_p.h"
#include "qqmltypenamecache_p.h"
#include "qqmltypeloader_p.h"
#include "private/qv4identifier_p.h"
#include <private/qqmljsastfwd_p.h>
#include "qqmlcustomparser_p.h"

#include <QtCore/qbytearray.h>
#include <QtCore/qset.h>
#include <QtCore/QCoreApplication>

QT_BEGIN_NAMESPACE

namespace QV4 {
namespace CompiledData {
struct CompilationUnit;
struct Unit;
}
}

class QQmlEngine;
class QQmlComponent;
class QQmlContext;
class QQmlContextData;

// ### Merge with QV4::CompiledData::CompilationUnit
class Q_AUTOTEST_EXPORT QQmlCompiledData : public QQmlRefCount, public QQmlCleanup
{
public:
    QQmlCompiledData(QQmlEngine *engine);
    virtual ~QQmlCompiledData();

    QQmlEngine *engine;

    QString fileName() const { return compilationUnit->fileName(); }
    QUrl url() const { return compilationUnit->url(); }
    QQmlTypeNameCache *importCache;

    int metaTypeId;
    int listMetaTypeId;
    bool isRegisteredWithEngine;

    struct TypeReference
    {
        TypeReference()
            : type(0), typePropertyCache(0), component(0)
            , majorVersion(0)
            , minorVersion(0)
            , isFullyDynamicType(false)
        {}

        QQmlType *type;
        QQmlPropertyCache *typePropertyCache;
        QQmlCompiledData *component;

        int majorVersion;
        int minorVersion;
        // Types such as QQmlPropertyMap can add properties dynamically at run-time and
        // therefore cannot have a property cache installed when instantiated.
        bool isFullyDynamicType;

        QQmlPropertyCache *propertyCache() const;
        QQmlPropertyCache *createPropertyCache(QQmlEngine *);

        void doDynamicTypeCheck();
    };
    // map from name index
    QHash<int, TypeReference*> resolvedTypes;

    QQmlPropertyCache *rootPropertyCache;
    QVector<QByteArray> metaObjects;
    QVector<QQmlPropertyCache *> propertyCaches;
    QList<QQmlScriptData *> scripts;

    QQmlRefPointer<QV4::CompiledData::CompilationUnit> compilationUnit;
    // index in first hash is component index, hash inside maps from object index in that scope to integer id
    QHash<int, QHash<int, int> > objectIndexToIdPerComponent;
    QHash<int, int> objectIndexToIdForRoot;
    // hash key is object index, value is indicies of bindings covered by custom parser
    QHash<int, QBitArray> customParserBindings;
    QHash<int, QBitArray> deferredBindingsPerObject; // index is object index
    int totalBindingsCount; // Number of bindings used in this type
    int totalParserStatusCount; // Number of instantiated types that are QQmlParserStatus subclasses
    int totalObjectCount; // Number of objects explicitly instantiated

    bool isComponent(int objectIndex) const { return objectIndexToIdPerComponent.contains(objectIndex); }
    bool isCompositeType() const { return !metaObjects.at(compilationUnit->data->indexOfRootObject).isEmpty(); }

    bool isInitialized() const { return hasEngine(); }
    void initialize(QQmlEngine *);

protected:
    virtual void destroy(); // From QQmlRefCount
    virtual void clear(); // From QQmlCleanup

private:
    QQmlCompiledData(const QQmlCompiledData &other);
    QQmlCompiledData &operator=(const QQmlCompiledData &other);
};

QT_END_NAMESPACE

#endif // QQMLCOMPILER_P_H
