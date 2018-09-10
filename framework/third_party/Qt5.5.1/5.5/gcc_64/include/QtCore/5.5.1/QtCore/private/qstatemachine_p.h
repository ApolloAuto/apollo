/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#ifndef QSTATEMACHINE_P_H
#define QSTATEMACHINE_P_H

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

#include "private/qstate_p.h"

#include <QtCore/qcoreevent.h>
#include <QtCore/qhash.h>
#include <QtCore/qlist.h>
#include <QtCore/qmutex.h>
#include <QtCore/qpair.h>
#include <QtCore/qpointer.h>
#include <QtCore/qset.h>
#include <QtCore/qvector.h>
#include <private/qfreelist_p.h>

QT_BEGIN_NAMESPACE

class QEvent;
#ifndef QT_NO_STATEMACHINE_EVENTFILTER
class QEventTransition;
#endif
class QSignalEventGenerator;
class QSignalTransition;
class QAbstractState;
class QAbstractTransition;
class QFinalState;
class QHistoryState;
class QState;

#ifndef QT_NO_ANIMATION
class QAbstractAnimation;
#endif

struct CalculationCache;
class QStateMachine;
class Q_CORE_EXPORT QStateMachinePrivate : public QStatePrivate
{
    Q_DECLARE_PUBLIC(QStateMachine)
public:
    enum State {
        NotRunning,
        Starting,
        Running
    };
    enum EventProcessingMode {
        DirectProcessing,
        QueuedProcessing
    };
    enum StopProcessingReason {
        EventQueueEmpty,
        Finished,
        Stopped
    };

    QStateMachinePrivate();
    ~QStateMachinePrivate();

    static QStateMachinePrivate *get(QStateMachine *q);

    QState *findLCA(const QList<QAbstractState*> &states, bool onlyCompound = false) const;
    QState *findLCCA(const QList<QAbstractState*> &states) const;

    static bool transitionStateEntryLessThan(QAbstractTransition *t1, QAbstractTransition *t2);
    static bool stateEntryLessThan(QAbstractState *s1, QAbstractState *s2);
    static bool stateExitLessThan(QAbstractState *s1, QAbstractState *s2);

    QAbstractState *findErrorState(QAbstractState *context);
    void setError(QStateMachine::Error error, QAbstractState *currentContext);

    // private slots
    void _q_start();
    void _q_process();
#ifndef QT_NO_ANIMATION
    void _q_animationFinished();
#endif
    void _q_startDelayedEventTimer(int id, int delay);
    void _q_killDelayedEventTimer(int id, int timerId);

    QState *rootState() const;

    void clearHistory();
    QAbstractTransition *createInitialTransition() const;

    void removeConflictingTransitions(QList<QAbstractTransition*> &enabledTransitions, CalculationCache *cache);
    void microstep(QEvent *event, const QList<QAbstractTransition*> &transitionList, CalculationCache *cache);
    QList<QAbstractTransition *> selectTransitions(QEvent *event, CalculationCache *cache);
    virtual void noMicrostep();
    virtual void processedPendingEvents(bool didChange);
    virtual void beginMacrostep();
    virtual void endMacrostep(bool didChange);
    void exitStates(QEvent *event, const QList<QAbstractState *> &statesToExit_sorted,
                    const QHash<QAbstractState*, QList<QPropertyAssignment> > &assignmentsForEnteredStates);
    QList<QAbstractState*> computeExitSet(const QList<QAbstractTransition*> &enabledTransitions, CalculationCache *cache);
    QSet<QAbstractState*> computeExitSet_Unordered(const QList<QAbstractTransition*> &enabledTransitions, CalculationCache *cache);
    QSet<QAbstractState*> computeExitSet_Unordered(QAbstractTransition *t, CalculationCache *cache);
    void executeTransitionContent(QEvent *event, const QList<QAbstractTransition*> &transitionList);
    void enterStates(QEvent *event, const QList<QAbstractState*> &exitedStates_sorted,
                     const QList<QAbstractState*> &statesToEnter_sorted,
                     const QSet<QAbstractState*> &statesForDefaultEntry,
                     QHash<QAbstractState *, QList<QPropertyAssignment> > &propertyAssignmentsForState
#ifndef QT_NO_ANIMATION
                     , const QList<QAbstractAnimation*> &selectedAnimations
#endif
                     );
    QList<QAbstractState*> computeEntrySet(const QList<QAbstractTransition*> &enabledTransitions,
                                           QSet<QAbstractState*> &statesForDefaultEntry, CalculationCache *cache);
    QAbstractState *getTransitionDomain(QAbstractTransition *t,
                                        const QList<QAbstractState *> &effectiveTargetStates,
                                        CalculationCache *cache) const;
    void addDescendantStatesToEnter(QAbstractState *state,
                                    QSet<QAbstractState*> &statesToEnter,
                                    QSet<QAbstractState*> &statesForDefaultEntry);
    void addStatesToEnter(QAbstractState *s, QState *root,
                          QSet<QAbstractState*> &statesToEnter,
                          QSet<QAbstractState*> &statesForDefaultEntry);
    void addAncestorStatesToEnter(QAbstractState *s, QAbstractState *ancestor,
                                  QSet<QAbstractState*> &statesToEnter,
                                  QSet<QAbstractState*> &statesForDefaultEntry);

    static QState *toStandardState(QAbstractState *state);
    static const QState *toStandardState(const QAbstractState *state);
    static QFinalState *toFinalState(QAbstractState *state);
    static QHistoryState *toHistoryState(QAbstractState *state);

    bool isInFinalState(QAbstractState *s) const;
    static bool isFinal(const QAbstractState *s);
    static bool isParallel(const QAbstractState *s);
    bool isCompound(const QAbstractState *s) const;
    bool isAtomic(const QAbstractState *s) const;

    void goToState(QAbstractState *targetState);

    void registerTransitions(QAbstractState *state);
    void maybeRegisterTransition(QAbstractTransition *transition);
    void registerTransition(QAbstractTransition *transition);
    void maybeRegisterSignalTransition(QSignalTransition *transition);
    void registerSignalTransition(QSignalTransition *transition);
    void unregisterSignalTransition(QSignalTransition *transition);
    void registerMultiThreadedSignalTransitions();
#ifndef QT_NO_STATEMACHINE_EVENTFILTER
    void maybeRegisterEventTransition(QEventTransition *transition);
    void registerEventTransition(QEventTransition *transition);
    void unregisterEventTransition(QEventTransition *transition);
    void handleFilteredEvent(QObject *watched, QEvent *event);
#endif
    void unregisterTransition(QAbstractTransition *transition);
    void unregisterAllTransitions();
    void handleTransitionSignal(QObject *sender, int signalIndex,
                                void **args);

    void postInternalEvent(QEvent *e);
    void postExternalEvent(QEvent *e);
    QEvent *dequeueInternalEvent();
    QEvent *dequeueExternalEvent();
    bool isInternalEventQueueEmpty();
    bool isExternalEventQueueEmpty();
    void processEvents(EventProcessingMode processingMode);
    void cancelAllDelayedEvents();

    virtual void emitStateFinished(QState *forState, QFinalState *guiltyState);
    virtual void startupHook();

#ifndef QT_NO_PROPERTIES
    class RestorableId {
        QPointer<QObject> guard;
        QObject *obj;
        QByteArray prop;
        // two overloads because friends can't have default arguments
        friend uint qHash(const RestorableId &key, uint seed)
            Q_DECL_NOEXCEPT_EXPR(noexcept(qHash(std::declval<QByteArray>())))
        { return qHash(qMakePair(key.obj, key.prop), seed); }
        friend uint qHash(const RestorableId &key) Q_DECL_NOEXCEPT_EXPR(noexcept(qHash(key, 0U)))
        { return qHash(key, 0U); }
        friend bool operator==(const RestorableId &lhs, const RestorableId &rhs) Q_DECL_NOTHROW
        { return lhs.obj == rhs.obj && lhs.prop == rhs.prop; }
        friend bool operator!=(const RestorableId &lhs, const RestorableId &rhs) Q_DECL_NOTHROW
        { return !operator==(lhs, rhs); }
    public:
        explicit RestorableId(QObject *o, QByteArray p) Q_DECL_NOTHROW : guard(o), obj(o), prop(qMove(p)) {}
        QObject *object() const Q_DECL_NOTHROW { return guard; }
        QByteArray propertyName() const Q_DECL_NOTHROW { return prop; }
    };
    QHash<QAbstractState*, QHash<RestorableId, QVariant> > registeredRestorablesForState;
    bool hasRestorable(QAbstractState *state, QObject *object, const QByteArray &propertyName) const;
    QVariant savedValueForRestorable(const QList<QAbstractState*> &exitedStates_sorted,
                                     QObject *object, const QByteArray &propertyName) const;
    void registerRestorable(QAbstractState *state, QObject *object, const QByteArray &propertyName,
                            const QVariant &value);
    void unregisterRestorables(const QList<QAbstractState*> &states, QObject *object,
                               const QByteArray &propertyName);
    QList<QPropertyAssignment> restorablesToPropertyList(const QHash<RestorableId, QVariant> &restorables) const;
    QHash<RestorableId, QVariant> computePendingRestorables(const QList<QAbstractState*> &statesToExit_sorted) const;
    QHash<QAbstractState*, QList<QPropertyAssignment> > computePropertyAssignments(
            const QList<QAbstractState*> &statesToEnter_sorted,
            QHash<RestorableId, QVariant> &pendingRestorables) const;
#endif

    State state;
    bool processing;
    bool processingScheduled;
    bool stop;
    StopProcessingReason stopProcessingReason;
    QSet<QAbstractState*> configuration;
    QList<QEvent*> internalEventQueue;
    QList<QEvent*> externalEventQueue;
    QMutex internalEventMutex;
    QMutex externalEventMutex;

    QStateMachine::Error error;
    QState::RestorePolicy globalRestorePolicy;

    QString errorString;
    QSet<QAbstractState *> pendingErrorStates;
    QSet<QAbstractState *> pendingErrorStatesForDefaultEntry;

#ifndef QT_NO_ANIMATION
    bool animated;

    QPair<QList<QAbstractAnimation*>, QList<QAbstractAnimation*> >
        initializeAnimation(QAbstractAnimation *abstractAnimation,
                            const QPropertyAssignment &prop);

    QHash<QAbstractState*, QList<QAbstractAnimation*> > animationsForState;
    QHash<QAbstractAnimation*, QPropertyAssignment> propertyForAnimation;
    QHash<QAbstractAnimation*, QAbstractState*> stateForAnimation;
    QSet<QAbstractAnimation*> resetAnimationEndValues;

    QList<QAbstractAnimation *> defaultAnimations;
    QMultiHash<QAbstractState *, QAbstractAnimation *> defaultAnimationsForSource;
    QMultiHash<QAbstractState *, QAbstractAnimation *> defaultAnimationsForTarget;

    QList<QAbstractAnimation *> selectAnimations(const QList<QAbstractTransition *> &transitionList) const;
    void terminateActiveAnimations(QAbstractState *state,
            const QHash<QAbstractState*, QList<QPropertyAssignment> > &assignmentsForEnteredStates);
    void initializeAnimations(QAbstractState *state, const QList<QAbstractAnimation*> &selectedAnimations,
                              const QList<QAbstractState *> &exitedStates_sorted,
                              QHash<QAbstractState *, QList<QPropertyAssignment> > &assignmentsForEnteredStates);
#endif // QT_NO_ANIMATION

    QSignalEventGenerator *signalEventGenerator;

    QHash<const QObject*, QVector<int> > connections;
    QMutex connectionsMutex;
#ifndef QT_NO_STATEMACHINE_EVENTFILTER
    QHash<QObject*, QHash<QEvent::Type, int> > qobjectEvents;
#endif
    QFreeList<void> delayedEventIdFreeList;
    struct DelayedEvent {
        QEvent *event;
        int timerId;
        DelayedEvent(QEvent *e, int tid)
            : event(e), timerId(tid) {}
        DelayedEvent()
            : event(0), timerId(0) {}
    };
    QHash<int, DelayedEvent> delayedEvents;
    QHash<int, int> timerIdToDelayedEventId;
    QMutex delayedEventsMutex;

    typedef QEvent* (*f_cloneEvent)(QEvent*);
    struct Handler {
        f_cloneEvent cloneEvent;
    };

    static const Handler *handler;
};

Q_CORE_EXPORT const QStateMachinePrivate::Handler *qcoreStateMachineHandler();

QT_END_NAMESPACE

#endif
