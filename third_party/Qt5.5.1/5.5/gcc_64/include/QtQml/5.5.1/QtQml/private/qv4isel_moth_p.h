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

#ifndef QV4ISEL_MOTH_P_H
#define QV4ISEL_MOTH_P_H

#include <private/qv4global_p.h>
#include <private/qv4isel_p.h>
#include <private/qv4isel_util_p.h>
#include <private/qv4jsir_p.h>
#include <private/qv4value_p.h>
#include "qv4instr_moth_p.h"

QT_BEGIN_NAMESPACE

namespace QV4 {
namespace Moth {

struct CompilationUnit : public QV4::CompiledData::CompilationUnit
{
    virtual ~CompilationUnit();
    virtual void linkBackendToEngine(QV4::ExecutionEngine *engine);

    QVector<QByteArray> codeRefs;

};

class Q_QML_EXPORT InstructionSelection:
        public IR::IRDecoder,
        public EvalInstructionSelection
{
public:
    InstructionSelection(QQmlEnginePrivate *qmlEngine, QV4::ExecutableAllocator *execAllocator, IR::Module *module, QV4::Compiler::JSUnitGenerator *jsGenerator);
    ~InstructionSelection();

    virtual void run(int functionIndex);

protected:
    virtual QQmlRefPointer<CompiledData::CompilationUnit> backendCompileStep();

    virtual void visitJump(IR::Jump *);
    virtual void visitCJump(IR::CJump *);
    virtual void visitRet(IR::Ret *);

    virtual void callBuiltinInvalid(IR::Name *func, IR::ExprList *args, IR::Expr *result);
    virtual void callBuiltinTypeofMember(IR::Expr *base, const QString &name, IR::Expr *result);
    virtual void callBuiltinTypeofSubscript(IR::Expr *base, IR::Expr *index, IR::Expr *result);
    virtual void callBuiltinTypeofName(const QString &name, IR::Expr *result);
    virtual void callBuiltinTypeofValue(IR::Expr *value, IR::Expr *result);
    virtual void callBuiltinDeleteMember(IR::Expr *base, const QString &name, IR::Expr *result);
    virtual void callBuiltinDeleteSubscript(IR::Expr *base, IR::Expr *index, IR::Expr *result);
    virtual void callBuiltinDeleteName(const QString &name, IR::Expr *result);
    virtual void callBuiltinDeleteValue(IR::Expr *result);
    virtual void callBuiltinThrow(IR::Expr *arg);
    virtual void callBuiltinReThrow();
    virtual void callBuiltinUnwindException(IR::Expr *);
    virtual void callBuiltinPushCatchScope(const QString &exceptionName);
    virtual void callBuiltinForeachIteratorObject(IR::Expr *arg, IR::Expr *result);
    virtual void callBuiltinForeachNextPropertyname(IR::Expr *arg, IR::Expr *result);
    virtual void callBuiltinPushWithScope(IR::Expr *arg);
    virtual void callBuiltinPopScope();
    virtual void callBuiltinDeclareVar(bool deletable, const QString &name);
    virtual void callBuiltinDefineArray(IR::Expr *result, IR::ExprList *args);
    virtual void callBuiltinDefineObjectLiteral(IR::Expr *result, int keyValuePairCount, IR::ExprList *keyValuePairs, IR::ExprList *arrayEntries, bool needSparseArray);
    virtual void callBuiltinSetupArgumentObject(IR::Expr *result);
    virtual void callBuiltinConvertThisToObject();
    virtual void callValue(IR::Expr *value, IR::ExprList *args, IR::Expr *result);
    virtual void callProperty(IR::Expr *base, const QString &name, IR::ExprList *args, IR::Expr *result);
    virtual void callSubscript(IR::Expr *base, IR::Expr *index, IR::ExprList *args, IR::Expr *result);
    virtual void convertType(IR::Expr *source, IR::Expr *target);
    virtual void constructActivationProperty(IR::Name *func, IR::ExprList *args, IR::Expr *result);
    virtual void constructProperty(IR::Expr *base, const QString &name, IR::ExprList *args, IR::Expr *result);
    virtual void constructValue(IR::Expr *value, IR::ExprList *args, IR::Expr *result);
    virtual void loadThisObject(IR::Expr *e);
    virtual void loadQmlIdArray(IR::Expr *e);
    virtual void loadQmlImportedScripts(IR::Expr *e);
    virtual void loadQmlContextObject(IR::Expr *e);
    virtual void loadQmlScopeObject(IR::Expr *e);
    virtual void loadQmlSingleton(const QString &name, IR::Expr *e);
    virtual void loadConst(IR::Const *sourceConst, IR::Expr *e);
    virtual void loadString(const QString &str, IR::Expr *target);
    virtual void loadRegexp(IR::RegExp *sourceRegexp, IR::Expr *target);
    virtual void getActivationProperty(const IR::Name *name, IR::Expr *target);
    virtual void setActivationProperty(IR::Expr *source, const QString &targetName);
    virtual void initClosure(IR::Closure *closure, IR::Expr *target);
    virtual void getProperty(IR::Expr *base, const QString &name, IR::Expr *target);
    virtual void setProperty(IR::Expr *source, IR::Expr *targetBase, const QString &targetName);
    virtual void setQObjectProperty(IR::Expr *source, IR::Expr *targetBase, int propertyIndex);
    virtual void getQObjectProperty(IR::Expr *base, int propertyIndex, bool captureRequired, bool isSingleton, int attachedPropertiesId, IR::Expr *target);
    virtual void getElement(IR::Expr *base, IR::Expr *index, IR::Expr *target);
    virtual void setElement(IR::Expr *source, IR::Expr *targetBase, IR::Expr *targetIndex);
    virtual void copyValue(IR::Expr *source, IR::Expr *target);
    virtual void swapValues(IR::Expr *source, IR::Expr *target);
    virtual void unop(IR::AluOp oper, IR::Expr *source, IR::Expr *target);
    virtual void binop(IR::AluOp oper, IR::Expr *leftSource, IR::Expr *rightSource, IR::Expr *target);

private:
    Param binopHelper(IR::AluOp oper, IR::Expr *leftSource, IR::Expr *rightSource, IR::Expr *target);

    struct Instruction {
#define MOTH_INSTR_DATA_TYPEDEF(I, FMT) typedef InstrData<Instr::I> I;
    FOR_EACH_MOTH_INSTR(MOTH_INSTR_DATA_TYPEDEF)
#undef MOTH_INSTR_DATA_TYPEDEF
    private:
        Instruction();
    };

    Param getParam(IR::Expr *e);

    Param getResultParam(IR::Expr *result)
    {
        if (result)
            return getParam(result);
        else
            return Param::createTemp(scratchTempIndex());
    }

    void simpleMove(IR::Move *);
    void prepareCallArgs(IR::ExprList *, quint32 &, quint32 * = 0);

    int scratchTempIndex() const { return _function->tempCount; }
    int callDataStart() const { return scratchTempIndex() + 1; }
    int outgoingArgumentTempStart() const { return callDataStart() + qOffsetOf(QV4::CallData, args)/sizeof(QV4::Value); }
    int frameSize() const { return outgoingArgumentTempStart() + _function->maxNumberOfArguments; }

    template <int Instr>
    inline ptrdiff_t addInstruction(const InstrData<Instr> &data);
    ptrdiff_t addInstructionHelper(Instr::Type type, Instr &instr);
    void patchJumpAddresses();
    QByteArray squeezeCode() const;

    QQmlEnginePrivate *qmlEngine;

    bool blockNeedsDebugInstruction;
    uint currentLine;
    IR::BasicBlock *_block;
    IR::BasicBlock *_nextBlock;

    QHash<IR::BasicBlock *, QVector<ptrdiff_t> > _patches;
    QHash<IR::BasicBlock *, ptrdiff_t> _addrs;

    uchar *_codeStart;
    uchar *_codeNext;
    uchar *_codeEnd;

    QSet<IR::Jump *> _removableJumps;
    IR::Stmt *_currentStatement;

    QScopedPointer<CompilationUnit> compilationUnit;
    QHash<IR::Function *, QByteArray> codeRefs;
};

class Q_QML_EXPORT ISelFactory: public EvalISelFactory
{
public:
    virtual ~ISelFactory() {}
    virtual EvalInstructionSelection *create(QQmlEnginePrivate *qmlEngine, QV4::ExecutableAllocator *execAllocator, IR::Module *module, QV4::Compiler::JSUnitGenerator *jsGenerator)
    { return new InstructionSelection(qmlEngine, execAllocator, module, jsGenerator); }
    virtual bool jitCompileRegexps() const
    { return false; }
};

template<int InstrT>
ptrdiff_t InstructionSelection::addInstruction(const InstrData<InstrT> &data)
{
    Instr genericInstr;
    InstrMeta<InstrT>::setDataNoCommon(genericInstr, data);
    return addInstructionHelper(static_cast<Instr::Type>(InstrT), genericInstr);
}

} // namespace Moth
} // namespace QV4

QT_END_NAMESPACE

#endif // QV4ISEL_MOTH_P_H
