/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKSTATE_P_H
#define QQUICKSTATE_P_H

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

#include "qquickstate_p.h"

#include "qquicktransitionmanager_p_p.h"

#include <private/qqmlproperty_p.h>
#include <private/qqmlguard_p.h>

#include <private/qqmlbinding_p.h>

#include <private/qobject_p.h>

QT_BEGIN_NAMESPACE

class QQuickSimpleAction
{
public:
    enum State { StartState, EndState };
    QQuickSimpleAction(const QQuickStateAction &a, State state = StartState)
    {
        m_property = a.property;
        m_specifiedObject = a.specifiedObject;
        m_specifiedProperty = a.specifiedProperty;
        m_event = a.event;
        if (state == StartState) {
            m_value = a.fromValue;
            if (QQmlPropertyPrivate::binding(m_property)) {
                m_binding = QQmlAbstractBinding::getPointer(QQmlPropertyPrivate::binding(m_property));
            }
            m_reverseEvent = true;
        } else {
            m_value = a.toValue;
            m_binding = a.toBinding;
            m_reverseEvent = false;
        }
    }

    ~QQuickSimpleAction()
    {
    }

    QQuickSimpleAction(const QQuickSimpleAction &other)
        :  m_property(other.m_property),
        m_value(other.m_value),
        m_binding(QQmlAbstractBinding::getPointer(other.binding())),
        m_specifiedObject(other.m_specifiedObject),
        m_specifiedProperty(other.m_specifiedProperty),
        m_event(other.m_event),
        m_reverseEvent(other.m_reverseEvent)
    {
    }

    QQuickSimpleAction &operator =(const QQuickSimpleAction &other)
    {
        m_property = other.m_property;
        m_value = other.m_value;
        m_binding = QQmlAbstractBinding::getPointer(other.binding());
        m_specifiedObject = other.m_specifiedObject;
        m_specifiedProperty = other.m_specifiedProperty;
        m_event = other.m_event;
        m_reverseEvent = other.m_reverseEvent;

        return *this;
    }

    void setProperty(const QQmlProperty &property)
    {
        m_property = property;
    }

    const QQmlProperty &property() const
    {
        return m_property;
    }

    void setValue(const QVariant &value)
    {
        m_value = value;
    }

    const QVariant &value() const
    {
        return m_value;
    }

    void setBinding(QQmlAbstractBinding *binding)
    {
        m_binding = QQmlAbstractBinding::getPointer(binding);
    }

    QQmlAbstractBinding *binding() const
    {
        return m_binding.data();
    }

    QObject *specifiedObject() const
    {
        return m_specifiedObject;
    }

    const QString &specifiedProperty() const
    {
        return m_specifiedProperty;
    }

    QQuickStateActionEvent *event() const
    {
        return m_event;
    }

    bool reverseEvent() const
    {
        return m_reverseEvent;
    }

private:
    QQmlProperty m_property;
    QVariant m_value;
    QQmlAbstractBinding::Pointer m_binding;
    QObject *m_specifiedObject;
    QString m_specifiedProperty;
    QQuickStateActionEvent *m_event;
    bool m_reverseEvent;
};

class QQuickRevertAction
{
public:
    QQuickRevertAction() : event(0) {}
    QQuickRevertAction(const QQmlProperty &prop) : property(prop), event(0) {}
    QQuickRevertAction(QQuickStateActionEvent *e) : event(e) {}
    QQmlProperty property;
    QQuickStateActionEvent *event;
};

class QQuickStateOperationPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QQuickStateOperation)

public:

    QQuickStateOperationPrivate()
    : m_state(0) {}

    QQuickState *m_state;
};

class QQuickStatePrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QQuickState)

public:
    QQuickStatePrivate()
    : when(0), named(false), inState(false), group(0) {}

    typedef QList<QQuickSimpleAction> SimpleActionList;

    QString name;
    QQmlBinding *when;
    bool named;

    struct OperationGuard : public QQmlGuard<QQuickStateOperation>
    {
        OperationGuard(QObject *obj, QList<OperationGuard> *l) : list(l) {
            setObject(static_cast<QQuickStateOperation *>(obj));
        }
        QList<OperationGuard> *list;
        void objectDestroyed(QQuickStateOperation *) {
            // we assume priv will always be destroyed after objectDestroyed calls
            list->removeOne(*this);
        }
    };
    QList<OperationGuard> operations;

    static void operations_append(QQmlListProperty<QQuickStateOperation> *prop, QQuickStateOperation *op) {
        QList<OperationGuard> *list = static_cast<QList<OperationGuard> *>(prop->data);
        op->setState(qobject_cast<QQuickState*>(prop->object));
        list->append(OperationGuard(op, list));
    }
    static void operations_clear(QQmlListProperty<QQuickStateOperation> *prop) {
        QList<OperationGuard> *list = static_cast<QList<OperationGuard> *>(prop->data);
        QMutableListIterator<OperationGuard> listIterator(*list);
        while(listIterator.hasNext())
            listIterator.next()->setState(0);
        list->clear();
    }
    static int operations_count(QQmlListProperty<QQuickStateOperation> *prop) {
        QList<OperationGuard> *list = static_cast<QList<OperationGuard> *>(prop->data);
        return list->count();
    }
    static QQuickStateOperation *operations_at(QQmlListProperty<QQuickStateOperation> *prop, int index) {
        QList<OperationGuard> *list = static_cast<QList<OperationGuard> *>(prop->data);
        return list->at(index);
    }

    QQuickTransitionManager transitionManager;

    SimpleActionList revertList;
    QList<QQuickRevertAction> reverting;
    QString extends;
    mutable bool inState;
    QQuickStateGroup *group;

    QQuickStateOperation::ActionList generateActionList() const;
    void complete();
};

QT_END_NAMESPACE

#endif // QQUICKSTATE_P_H
