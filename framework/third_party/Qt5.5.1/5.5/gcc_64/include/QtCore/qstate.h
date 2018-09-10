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

#ifndef QSTATE_H
#define QSTATE_H

#include <QtCore/qabstractstate.h>
#include <QtCore/qlist.h>
#include <QtCore/qmetaobject.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_STATEMACHINE

class QAbstractTransition;
class QSignalTransition;

class QStatePrivate;
class Q_CORE_EXPORT QState : public QAbstractState
{
    Q_OBJECT
    Q_PROPERTY(QAbstractState* initialState READ initialState WRITE setInitialState NOTIFY initialStateChanged)
    Q_PROPERTY(QAbstractState* errorState READ errorState WRITE setErrorState NOTIFY errorStateChanged)
    Q_PROPERTY(ChildMode childMode READ childMode WRITE setChildMode NOTIFY childModeChanged)
public:
    enum ChildMode {
        ExclusiveStates,
        ParallelStates
    };
    Q_ENUM(ChildMode)

    enum RestorePolicy {
        DontRestoreProperties,
        RestoreProperties
    };
    Q_ENUM(RestorePolicy)

    QState(QState *parent = 0);
    QState(ChildMode childMode, QState *parent = 0);
    ~QState();

    QAbstractState *errorState() const;
    void setErrorState(QAbstractState *state);

    void addTransition(QAbstractTransition *transition);
    QSignalTransition *addTransition(const QObject *sender, const char *signal, QAbstractState *target);
#ifdef Q_QDOC
    QSignalTransition *addTransition(const QObject *sender, PointerToMemberFunction signal,
                       QAbstractState *target);
#else
    template <typename Func>
    QSignalTransition *addTransition(const typename QtPrivate::FunctionPointer<Func>::Object *obj,
                      Func signal, QAbstractState *target)
    {
        const QMetaMethod signalMetaMethod = QMetaMethod::fromSignal(signal);
        return addTransition(obj, signalMetaMethod.methodSignature().constData(), target);
    }
#endif // Q_QDOC
    QAbstractTransition *addTransition(QAbstractState *target);
    void removeTransition(QAbstractTransition *transition);
    QList<QAbstractTransition*> transitions() const;

    QAbstractState *initialState() const;
    void setInitialState(QAbstractState *state);

    ChildMode childMode() const;
    void setChildMode(ChildMode mode);

#ifndef QT_NO_PROPERTIES
    void assignProperty(QObject *object, const char *name,
                        const QVariant &value);
#endif

Q_SIGNALS:
    void finished(QPrivateSignal);
    void propertiesAssigned(QPrivateSignal);
    void childModeChanged(QPrivateSignal);
    void initialStateChanged(QPrivateSignal);
    void errorStateChanged(QPrivateSignal);

protected:
    void onEntry(QEvent *event) Q_DECL_OVERRIDE;
    void onExit(QEvent *event) Q_DECL_OVERRIDE;

    bool event(QEvent *e) Q_DECL_OVERRIDE;

protected:
    QState(QStatePrivate &dd, QState *parent);

private:
    Q_DISABLE_COPY(QState)
    Q_DECLARE_PRIVATE(QState)
};

#endif //QT_NO_STATEMACHINE

QT_END_NAMESPACE

#endif
