/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

#ifndef Patternist_XsdStateMachine_H
#define Patternist_XsdStateMachine_H

#include <private/qnamepool_p.h>

#include <QtCore/QHash>
#include <QtCore/QSet>
#include <QtCore/QTextStream>

QT_BEGIN_NAMESPACE

class QIODevice;

namespace QPatternist
{
    /**
     * @short A state machine used for evaluation.
     *
     * @ingroup Patternist_schema
     * @author Tobias Koenig <tobias.koenig@nokia.com>
     */
    template <typename TransitionType>
    class XsdStateMachine
    {
        public:
            typedef qint32 StateId;

            /**
             * Describes the type of state.
             */
            enum StateType
            {
                StartState,     ///< The state the machine will start with.
                StartEndState,  ///< The state the machine will start with, can be end state as well.
                InternalState,  ///< Any state that is not start or end state.
                EndState        ///< Any state where the machine is allowed to stop.
            };

            /**
             * Creates a new state machine object.
             */
            XsdStateMachine();

            /**
             * Creates a new state machine object.
             *
             * The name pool to use for accessing object names.
             */
            XsdStateMachine(const NamePool::Ptr &namePool);

            /**
             * Adds a new state of the given @p type to the state machine.
             *
             * @return The id of the new state.
             */
            StateId addState(StateType type);

            /**
             * Adds a new @p transition to the state machine.
             *
             * @param start The start state.
             * @param transition The transition to come from the start to the end state.
             * @param end The end state.
             */
            void addTransition(StateId start, TransitionType transition, StateId end);

            /**
             * Adds a new epsilon @p transition to the state machine.
             *
             * @param start The start state.
             * @param end The end state.
             */
            void addEpsilonTransition(StateId start, StateId end);

            /**
             * Resets the machine to the start state.
             */
            void reset();

            /**
             * Removes all states and transitions from the state machine.
             */
            void clear();

            /**
             * Continues execution of the machine with the given input @p transition.
             *
             * @return @c true if the transition was successful, @c false otherwise.
             */
            bool proceed(TransitionType transition);

            /**
             * Returns the list of transitions that are reachable from the current
             * state.
             */
            QList<TransitionType> possibleTransitions() const;

            /**
             * Continues execution of the machine with the given @p input.
             *
             * @note To use this method, inputEqualsTransition must be implemented
             *       to find the right transition to use.
             *
             * @return @c true if the transition was successful, @c false otherwise.
             */
            template <typename InputType>
            bool proceed(InputType input);

            /**
             * Returns whether the given @p input matches the given @p transition.
             */
            template <typename InputType>
            bool inputEqualsTransition(InputType input, TransitionType transition) const;

            /**
             * Returns whether the machine is in an allowed end state.
             */
            bool inEndState() const;

            /**
             * Returns the last transition that was taken.
             */
            TransitionType lastTransition() const;

            /**
             * Returns the start state of the machine.
             */
            StateId startState() const;

            /**
             * This method should be redefined by template specialization for every
             * concret TransitionType.
             */
            QString transitionTypeToString(TransitionType type) const;

            /**
             * Outputs the state machine in DOT format to the given
             * output @p device.
             */
            bool outputGraph(QIODevice *device, const QString &graphName) const;

            /**
             * Returns a DFA that is equal to the NFA of the state machine.
             */
            XsdStateMachine<TransitionType> toDFA() const;

            /**
             * Returns the information of all states of the state machine.
             */
            QHash<StateId, StateType> states() const;

            /**
             * Returns the information of all transitions of the state machine.
             */
            QHash<StateId, QHash<TransitionType, QVector<StateId> > > transitions() const;

        private:
            /**
             * Returns the DFA state for the given @p nfaStat from the given @p stateTable.
             * If there is no corresponding DFA state yet, a new one is created.
             */
            StateId dfaStateForNfaState(QSet<StateId> nfaState, QList< QPair< QSet<StateId>, StateId> > &stateTable, XsdStateMachine<TransitionType> &dfa) const;

            /**
             * Returns the set of all states that can be reached from the set of @p input states
             * by the epsilon transition.
             *
             * The implementation is inlined in order to workaround a compiler
             * bug on Symbian/winscw.
             */
            QSet<StateId> epsilonClosure(const QSet<StateId> &input) const
            {
                // every state can reach itself by epsilon transition, so include the input states
                // in the result as well
                QSet<StateId> result = input;

                // add the input states to the list of to be processed states
                QList<StateId> workStates = input.toList();
                while (!workStates.isEmpty()) { // while there are states to be processed left...

                    // dequeue one state from list
                    const StateId state = workStates.takeFirst();

                    // get the list of states that can be reached by the epsilon transition
                    // from the current 'state'
                    const QVector<StateId> targetStates = m_epsilonTransitions.value(state);
                    for (int i = 0; i < targetStates.count(); ++i) {
                        // if we have this target state not in our result set yet...
                        if (!result.contains(targetStates.at(i))) {
                            // ... add it to the result set
                            result.insert(targetStates.at(i));

                            // add the target state to the list of to be processed states as well,
                            // as we want to have the epsilon transitions not only for the first
                            // level of following states
                            workStates.append(targetStates.at(i));
                        }
                    }
                }

                return result;
            }

            /**
             * Returns the set of all states that can be reached from the set of given @p states
             * by the given @p input.
             *
             * The implementation is inlined in order to workaround a compiler
             * bug on Symbian/winscw.
             */
            QSet<StateId> move(const QSet<StateId> &states, TransitionType input) const
            {
                QSet<StateId> result;

                QSetIterator<StateId> it(states);
                while (it.hasNext()) { // iterate over all given states
                    const StateId state = it.next();

                    // get the transition table for the current state
                    const QHash<TransitionType, QVector<StateId> > transitions = m_transitions.value(state);

                    // get the target states for the given input
                    const QVector<StateId> targetStates = transitions.value(input);

                    // add all target states to the result
                    for (int i = 0; i < targetStates.size(); ++i)
                        result.insert(targetStates.at(i));
                }

                return result;
            }

            NamePool::Ptr                                             m_namePool;
            QHash<StateId, StateType>                                 m_states;
            QHash<StateId, QHash<TransitionType, QVector<StateId> > > m_transitions;
            QHash<StateId, QVector<StateId> >                         m_epsilonTransitions;
            StateId                                                   m_currentState;
            qint32                                                    m_counter;
            TransitionType                                            m_lastTransition;
    };

    #include "qxsdstatemachine_tpl_p.h"
}

QT_END_NAMESPACE

#endif
