/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the V4VM module of the Qt Toolkit.
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
#ifndef QV4REGALLOC_P_H
#define QV4REGALLOC_P_H

#include "qv4global_p.h"
#include "qv4isel_p.h"
#include "qv4ssa_p.h"
#include "qv4registerinfo_p.h"

#include <config.h>

QT_BEGIN_NAMESPACE

namespace QV4 {
namespace JIT {

class RegAllocInfo;

// This class implements a linear-scan register allocator, with a couple of tweaks:
//  - Second-chance allocation is used when an interval becomes active after a spill, which results
//    in fewer differences between edges, and hence fewer moves before jumps.
//  - Use positions are flagged with either "must have" register or "could have" register. This is
//    used to decide whether a register is really needed when a temporary is used after a spill
//    occurred.
//  - Fixed intervals are used to denotate IR positions where certain registers are needed in order
//    to implement the operation, and cannot be used by a temporary on that position. An example is
//    caller saved registers, where the call will use/clobber those registers.
//  - Hints are used to indicate which registers could be used to generate more compact code. An
//    example is an addition, where one (or both) operands' life-time ends at that instruction. In
//    this case, re-using an operand register for the result will result in an in-place add.
//  - SSA form properties are used:
//      - to simplify life-times (two temporaries will never interfere as long as their intervals
//        are not split), resulting in a slightly faster algorithm;
//      - when a temporary needs to be spilled, it is done directly after calculating it, so that
//        1 store is generated even if multiple spills/splits happen.
//      - phi-node elimination (SSA form deconstruction) is done when resolving differences between
//        CFG edges
class RegisterAllocator
{
    typedef IR::LifeTimeInterval LifeTimeInterval;

    const RegisterInformation &_registerInformation;
    QVector<const RegisterInfo *> _normalRegisters;
    QVector<const RegisterInfo *> _fpRegisters;
    QScopedPointer<RegAllocInfo> _info;

    QVector<LifeTimeInterval *> _fixedRegisterRanges, _fixedFPRegisterRanges;

    IR::LifeTimeIntervals::Ptr _lifeTimeIntervals;
    typedef QVector<LifeTimeInterval *> Intervals;
    Intervals _unhandled, _active, _inactive, _handled;

    std::vector<int> _lastAssignedRegister;
    std::vector<int> _assignedSpillSlots;
    QVector<int> _activeSpillSlots;

    QBitArray _regularRegsInUse, _fpRegsInUse;

    Q_DISABLE_COPY(RegisterAllocator)

public:
    enum { InvalidSpillSlot = -1 };

    RegisterAllocator(const RegisterInformation &registerInformation);
    ~RegisterAllocator();

    void run(IR::Function *function, const IR::Optimizer &opt);
    RegisterInformation usedRegisters() const;

private:
    void markInUse(int reg, bool isFPReg);
    LifeTimeInterval *cloneFixedInterval(int reg, bool isFP, const LifeTimeInterval &original);
    void prepareRanges();
    void linearScan();
    void tryAllocateFreeReg(LifeTimeInterval &current);
    void allocateBlockedReg(LifeTimeInterval &current);
    int nextIntersection(const LifeTimeInterval &current, const LifeTimeInterval &another) const;
    int nextUse(const IR::Temp &t, int startPosition) const;
    void split(LifeTimeInterval &current, int beforePosition, bool skipOptionalRegisterUses =false);
    void splitInactiveAtEndOfLifetimeHole(int reg, bool isFPReg, int position);
    void assignSpillSlot(const IR::Temp &t, int startPos, int endPos);
    void resolve(IR::Function *function, const IR::Optimizer &opt);

    void dump(IR::Function *function) const;
};

} // end of namespace JIT
} // end of namespace QV4

QT_END_NAMESPACE

#endif // QV4REGALLOC_P_H
