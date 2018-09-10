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
#ifndef QV4COMPILER_P_H
#define QV4COMPILER_P_H

#include <QtCore/qstring.h>
#include "qv4jsir_p.h"

QT_BEGIN_NAMESPACE

class QQmlPropertyData;

namespace QV4 {

namespace CompiledData {
struct Unit;
struct Lookup;
struct RegExp;
struct JSClassMember;
}

namespace Compiler {

struct Q_QML_PRIVATE_EXPORT StringTableGenerator {
    StringTableGenerator();

    int registerString(const QString &str);
    int getStringId(const QString &string) const;
    QString stringForIndex(int index) const { return strings.at(index); }
    uint stringCount() const { return strings.size(); }

    uint sizeOfTableAndData() const { return stringDataSize + strings.count() * sizeof(uint); }

    void clear();

    void serialize(CompiledData::Unit *unit);

private:
    QHash<QString, int> stringToId;
    QStringList strings;
    uint stringDataSize;
};

struct Q_QML_PRIVATE_EXPORT JSUnitGenerator {
    JSUnitGenerator(IR::Module *module);

    int registerString(const QString &str) { return stringTable.registerString(str); }
    int getStringId(const QString &string) const { return stringTable.getStringId(string); }
    QString stringForIndex(int index) const { return stringTable.stringForIndex(index); }

    uint registerGetterLookup(const QString &name);
    uint registerSetterLookup(const QString &name);
    uint registerGlobalGetterLookup(const QString &name);
    uint registerIndexedGetterLookup();
    uint registerIndexedSetterLookup();

    int registerRegExp(IR::RegExp *regexp);

    int registerConstant(ReturnedValue v);

    int registerJSClass(int count, IR::ExprList *args);

    enum GeneratorOption {
        GenerateWithStringTable,
        GenerateWithoutStringTable
    };

    QV4::CompiledData::Unit *generateUnit(GeneratorOption option = GenerateWithStringTable);
    // Returns bytes written
    int writeFunction(char *f, int index, IR::Function *irFunction);

    StringTableGenerator stringTable;
private:
    IR::Module *irModule;

    QHash<IR::Function *, uint> functionOffsets;
    QList<CompiledData::Lookup> lookups;
    QVector<CompiledData::RegExp> regexps;
    QVector<ReturnedValue> constants;
    QList<QList<CompiledData::JSClassMember> > jsClasses;
    uint jsClassDataSize;
};

}

}

QT_END_NAMESPACE

#endif
