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
#ifndef QV4BINOP_P_H
#define QV4BINOP_P_H

#include <qv4jsir_p.h>
#include <qv4isel_masm_p.h>
#include <qv4assembler_p.h>

QT_BEGIN_NAMESPACE

#if ENABLE(ASSEMBLER)

namespace QV4 {
namespace JIT {

struct Binop {
    Binop(Assembler *assembler, IR::AluOp operation)
        : as(assembler)
        , op(operation)
    {}

    void generate(IR::Expr *lhs, IR::Expr *rhs, IR::Expr *target);
    void doubleBinop(IR::Expr *lhs, IR::Expr *rhs, IR::Expr *target);
    bool int32Binop(IR::Expr *leftSource, IR::Expr *rightSource, IR::Expr *target);
    Assembler::Jump genInlineBinop(IR::Expr *leftSource, IR::Expr *rightSource, IR::Expr *target);

    typedef Assembler::Jump (Binop::*MemRegOp)(Assembler::Address, Assembler::RegisterID);
    typedef Assembler::Jump (Binop::*ImmRegOp)(Assembler::TrustedImm32, Assembler::RegisterID);

    struct OpInfo {
        const char *name;
        QV4::Runtime::BinaryOperation fallbackImplementation;
        QV4::Runtime::BinaryOperationContext contextImplementation;
        MemRegOp inlineMemRegOp;
        ImmRegOp inlineImmRegOp;
    };

    static const OpInfo operations[IR::LastAluOp + 1];
    static const OpInfo &operation(IR::AluOp operation)
    { return operations[operation]; }

    Assembler::Jump inline_add32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        return as->branchAdd32(Assembler::Overflow, addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        return as->branchAdd32(Assembler::Overflow, Assembler::ScratchRegister, reg);
#endif
    }

    Assembler::Jump inline_add32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        return as->branchAdd32(Assembler::Overflow, imm, reg);
    }

    Assembler::Jump inline_sub32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        return as->branchSub32(Assembler::Overflow, addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        return as->branchSub32(Assembler::Overflow, Assembler::ScratchRegister, reg);
#endif
    }

    Assembler::Jump inline_sub32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        return as->branchSub32(Assembler::Overflow, imm, reg);
    }

    Assembler::Jump inline_mul32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        return as->branchMul32(Assembler::Overflow, addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        return as->branchMul32(Assembler::Overflow, Assembler::ScratchRegister, reg);
#endif
    }

    Assembler::Jump inline_mul32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        return as->branchMul32(Assembler::Overflow, imm, reg, reg);
    }

    Assembler::Jump inline_shl32(Assembler::Address addr, Assembler::RegisterID reg)
    {
        as->load32(addr, Assembler::ScratchRegister);
        as->and32(Assembler::TrustedImm32(0x1f), Assembler::ScratchRegister);
        as->lshift32(Assembler::ScratchRegister, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_shl32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        imm.m_value &= 0x1f;
        as->lshift32(imm, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_shr32(Assembler::Address addr, Assembler::RegisterID reg)
    {
        as->load32(addr, Assembler::ScratchRegister);
        as->and32(Assembler::TrustedImm32(0x1f), Assembler::ScratchRegister);
        as->rshift32(Assembler::ScratchRegister, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_shr32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        imm.m_value &= 0x1f;
        as->rshift32(imm, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_ushr32(Assembler::Address addr, Assembler::RegisterID reg)
    {
        as->load32(addr, Assembler::ScratchRegister);
        as->and32(Assembler::TrustedImm32(0x1f), Assembler::ScratchRegister);
        as->urshift32(Assembler::ScratchRegister, reg);
        return as->branchTest32(Assembler::Signed, reg, reg);
    }

    Assembler::Jump inline_ushr32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        imm.m_value &= 0x1f;
        as->urshift32(imm, reg);
        return as->branchTest32(Assembler::Signed, reg, reg);
    }

    Assembler::Jump inline_and32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        as->and32(addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        as->and32(Assembler::ScratchRegister, reg);
#endif
        return Assembler::Jump();
    }

    Assembler::Jump inline_and32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        as->and32(imm, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_or32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        as->or32(addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        as->or32(Assembler::ScratchRegister, reg);
#endif
        return Assembler::Jump();
    }

    Assembler::Jump inline_or32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        as->or32(imm, reg);
        return Assembler::Jump();
    }

    Assembler::Jump inline_xor32(Assembler::Address addr, Assembler::RegisterID reg)
    {
#if HAVE(ALU_OPS_WITH_MEM_OPERAND)
        as->xor32(addr, reg);
#else
        as->load32(addr, Assembler::ScratchRegister);
        as->xor32(Assembler::ScratchRegister, reg);
#endif
        return Assembler::Jump();
    }

    Assembler::Jump inline_xor32(Assembler::TrustedImm32 imm, Assembler::RegisterID reg)
    {
        as->xor32(imm, reg);
        return Assembler::Jump();
    }



    Assembler *as;
    IR::AluOp op;
};

}
}

#endif

QT_END_NAMESPACE

#endif
