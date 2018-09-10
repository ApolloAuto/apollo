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

#ifndef QV4SSA_P_H
#define QV4SSA_P_H

#include "qv4jsir_p.h"
#include <QtCore/QSharedPointer>

QT_BEGIN_NAMESPACE
class QTextStream;
class QQmlEnginePrivate;

namespace QV4 {
namespace IR {

class Q_AUTOTEST_EXPORT LifeTimeInterval {
public:
    struct Range {
        int start;
        int end;

        Range(int start = InvalidPosition, int end = InvalidPosition)
            : start(start)
            , end(end)
        {}

        bool covers(int position) const { return start <= position && position <= end; }
    };
    typedef QVector<Range> Ranges;

private:
    Temp _temp;
    Ranges _ranges;
    int _end;
    int _reg;
    unsigned _isFixedInterval : 1;
    unsigned _isSplitFromInterval : 1;

public:
    enum { InvalidPosition = -1 };
    enum { InvalidRegister = -1 };

    explicit LifeTimeInterval(int rangeCapacity = 2)
        : _end(InvalidPosition)
        , _reg(InvalidRegister)
        , _isFixedInterval(0)
        , _isSplitFromInterval(0)
    { _ranges.reserve(rangeCapacity); }

    bool isValid() const { return _end != InvalidRegister; }

    void setTemp(const Temp &temp) { this->_temp = temp; }
    Temp temp() const { return _temp; }
    bool isFP() const { return _temp.type == IR::DoubleType; }

    void setFrom(int from);
    void addRange(int from, int to);
    const Ranges &ranges() const { return _ranges; }

    int start() const { return _ranges.first().start; }
    int end() const { return _end; }
    bool covers(int position) const
    {
        for (int i = 0, ei = _ranges.size(); i != ei; ++i) {
            if (_ranges.at(i).covers(position))
                return true;
        }
        return false;
    }

    int reg() const { return _reg; }
    void setReg(int reg) { Q_ASSERT(!_isFixedInterval); _reg = reg; }

    bool isFixedInterval() const { return _isFixedInterval; }
    void setFixedInterval(bool isFixedInterval) { _isFixedInterval = isFixedInterval; }

    LifeTimeInterval split(int atPosition, int newStart);
    bool isSplitFromInterval() const { return _isSplitFromInterval; }
    void setSplitFromInterval(bool isSplitFromInterval) { _isSplitFromInterval = isSplitFromInterval; }

    void dump(QTextStream &out) const;
    static bool lessThan(const LifeTimeInterval *r1, const LifeTimeInterval *r2);
    static bool lessThanForTemp(const LifeTimeInterval *r1, const LifeTimeInterval *r2);

    void validate() const {
#if !defined(QT_NO_DEBUG)
        // Validate the new range
        if (_end != InvalidPosition) {
            Q_ASSERT(!_ranges.isEmpty());
            foreach (const Range &range, _ranges) {
                Q_ASSERT(range.start >= 0);
                Q_ASSERT(range.end >= 0);
                Q_ASSERT(range.start <= range.end);
            }
        }
#endif
    }
};

class LifeTimeIntervals
{
    Q_DISABLE_COPY(LifeTimeIntervals)

    LifeTimeIntervals(IR::Function *function);
    void renumber(IR::Function *function);

public:
    typedef QSharedPointer<LifeTimeIntervals> Ptr;
    static Ptr create(IR::Function *function)
    { return Ptr(new LifeTimeIntervals(function)); }

    ~LifeTimeIntervals();

    // takes ownership of the pointer
    void add(LifeTimeInterval *interval)
    { _intervals.append(interval); }

    // After calling Optimizer::lifeTimeIntervals() the result will have all intervals in descending order of start position.
    QVector<LifeTimeInterval *> intervals() const
    { return _intervals; }

    int size() const
    { return _intervals.size(); }

    int positionForStatement(Stmt *stmt) const
    {
        Q_ASSERT(stmt->id() >= 0);
        if (static_cast<unsigned>(stmt->id()) < _positionForStatement.size())
            return _positionForStatement[stmt->id()];

        return Stmt::InvalidId;
    }

    int startPosition(BasicBlock *bb) const
    {
        Q_ASSERT(bb->index() >= 0);
        Q_ASSERT(static_cast<unsigned>(bb->index()) < _basicBlockPosition.size());

        return _basicBlockPosition.at(bb->index()).start;
    }

    int endPosition(BasicBlock *bb) const
    {
        Q_ASSERT(bb->index() >= 0);
        Q_ASSERT(static_cast<unsigned>(bb->index()) < _basicBlockPosition.size());

        return _basicBlockPosition.at(bb->index()).end;
    }

    int lastPosition() const
    {
        return _lastPosition;
    }

private:
    struct BasicBlockPositions {
        int start;
        int end;

        BasicBlockPositions()
            : start(IR::Stmt::InvalidId)
            , end(IR::Stmt::InvalidId)
        {}
    };

    std::vector<BasicBlockPositions> _basicBlockPosition;
    std::vector<int> _positionForStatement;
    QVector<LifeTimeInterval *> _intervals;
    int _lastPosition;
};

class Q_QML_PRIVATE_EXPORT Optimizer
{
    Q_DISABLE_COPY(Optimizer)

public:
    Optimizer(Function *function);

    void run(QQmlEnginePrivate *qmlEngine, bool doTypeInference = true, bool peelLoops = true);
    void convertOutOfSSA();

    bool isInSSA() const
    { return inSSA; }

    QHash<BasicBlock *, BasicBlock *> loopStartEndBlocks() const { return startEndLoops; }

    LifeTimeIntervals::Ptr lifeTimeIntervals() const;

    QSet<IR::Jump *> calculateOptionalJumps();

    static void showMeTheCode(Function *function, const char *marker);

private:
    Function *function;
    bool inSSA;
    QHash<BasicBlock *, BasicBlock *> startEndLoops;
};

class MoveMapping
{
    struct Move {
        Expr *from;
        Temp *to;
        bool needsSwap;

        Move(Expr *from, Temp *to)
            : from(from), to(to), needsSwap(false)
        {}

        bool operator==(const Move &other) const
        { return from == other.from && to == other.to; }
    };
    typedef QList<Move> Moves;

    Moves _moves;

    static Moves sourceUsages(Expr *e, const Moves &moves);

public:
    void add(Expr *from, Temp *to);
    void order();
    QList<IR::Move *> insertMoves(BasicBlock *bb, Function *function, bool atEnd) const;

    void dump() const;

private:
    enum Action { NormalMove, NeedsSwap };
    Action schedule(const Move &m, QList<Move> &todo, QList<Move> &delayed, QList<Move> &output,
                    QList<Move> &swaps) const;
};

} // IR namespace
} // QV4 namespace


Q_DECLARE_TYPEINFO(QV4::IR::LifeTimeInterval, Q_MOVABLE_TYPE);
Q_DECLARE_TYPEINFO(QV4::IR::LifeTimeInterval::Range, Q_PRIMITIVE_TYPE);

QT_END_NAMESPACE

#endif // QV4SSA_P_H
