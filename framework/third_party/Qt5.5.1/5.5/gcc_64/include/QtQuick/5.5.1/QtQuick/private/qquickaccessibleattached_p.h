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

#ifndef QQUICKACCESSIBLEATTACHED_H
#define QQUICKACCESSIBLEATTACHED_H

#include <QtQuick/qquickitem.h>

#include <QtCore/qobject.h>
#include <QtCore/qstring.h>

#ifndef QT_NO_ACCESSIBILITY

#include <QtGui/qaccessible.h>
#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE


#define STATE_PROPERTY(P) \
    Q_PROPERTY(bool P READ P WRITE set_ ## P NOTIFY P ## Changed FINAL) \
    bool P() const { return m_state.P ; } \
    void set_ ## P(bool arg) \
    { \
        if (m_state.P == arg) \
            return; \
        m_state.P = arg; \
        emit P ## Changed(arg); \
        QAccessible::State changedState; \
        changedState.P = true; \
        QAccessibleStateChangeEvent ev(parent(), changedState); \
        QAccessible::updateAccessibility(&ev); \
    } \
    Q_SIGNAL void P ## Changed(bool arg);


class Q_QUICK_PRIVATE_EXPORT QQuickAccessibleAttached : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QAccessible::Role role READ role WRITE setRole NOTIFY roleChanged FINAL)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged FINAL)
    Q_PROPERTY(QString description READ description WRITE setDescription NOTIFY descriptionChanged FINAL)
    Q_PROPERTY(bool ignored READ ignored WRITE setIgnored NOTIFY ignoredChanged FINAL)

public:
    Q_ENUMS(QAccessible::Role QAccessible::Event)
    STATE_PROPERTY(checkable)
    STATE_PROPERTY(checked)
    STATE_PROPERTY(editable)
    STATE_PROPERTY(focusable)
    STATE_PROPERTY(focused)
    STATE_PROPERTY(multiLine)
    STATE_PROPERTY(readOnly)
    STATE_PROPERTY(selected)
    STATE_PROPERTY(selectable)
    STATE_PROPERTY(pressed)
    STATE_PROPERTY(checkStateMixed)
    STATE_PROPERTY(defaultButton)
    STATE_PROPERTY(passwordEdit)
    STATE_PROPERTY(selectableText)
    STATE_PROPERTY(searchEdit)

    QQuickAccessibleAttached(QObject *parent);
    ~QQuickAccessibleAttached();

    QAccessible::Role role() const { return m_role; }
    void setRole(QAccessible::Role role)
    {
        if (role != m_role) {
            m_role = role;
            Q_EMIT roleChanged();
            // There is no way to signify role changes at the moment.
            // QAccessible::updateAccessibility(parent(), 0, QAccessible::);

            switch (role) {
            case QAccessible::CheckBox:
            case QAccessible::RadioButton:
                m_state.focusable = true;
                m_state.checkable = true;
                break;
            case QAccessible::Button:
            case QAccessible::MenuItem:
            case QAccessible::PageTab:
            case QAccessible::EditableText:
            case QAccessible::SpinBox:
            case QAccessible::ComboBox:
            case QAccessible::Terminal:
            case QAccessible::ScrollBar:
                m_state.focusable = true;
                break;
            default:
                break;
            }
        }
    }
    QString name() const {
        if (m_state.passwordEdit)
            return QString();
        return m_name;
    }
    void setName(const QString &name) {
        if (name != m_name) {
            m_name = name;
            Q_EMIT nameChanged();
            QAccessibleEvent ev(parent(), QAccessible::NameChanged);
            QAccessible::updateAccessibility(&ev);
        }
    }

    QString description() const { return m_description; }
    void setDescription(const QString &description)
    {
        if (m_description != description) {
            m_description = description;
            Q_EMIT descriptionChanged();
            QAccessibleEvent ev(parent(), QAccessible::DescriptionChanged);
            QAccessible::updateAccessibility(&ev);
        }
    }

    // Factory function
    static QQuickAccessibleAttached *qmlAttachedProperties(QObject *obj);

    static QQuickAccessibleAttached *attachedProperties(const QObject *obj)
    {
        return qobject_cast<QQuickAccessibleAttached*>(qmlAttachedPropertiesObject<QQuickAccessibleAttached>(obj, false));
    }

    // Property getter
    static QVariant property(const QObject *object, const char *propertyName)
    {
        if (QObject *attachedObject = QQuickAccessibleAttached::attachedProperties(object))
            return attachedObject->property(propertyName);
        return QVariant();
    }

    static bool setProperty(QObject *object, const char *propertyName, const QVariant &value)
    {
        QObject *obj = qmlAttachedPropertiesObject<QQuickAccessibleAttached>(object, true);
        if (!obj) {
            qWarning("cannot set property Accessible.%s of QObject %s", propertyName, object->metaObject()->className());
            return false;
        }
        return obj->setProperty(propertyName, value);
    }

    static QObject *findAccessible(QObject *object, QAccessible::Role role = QAccessible::NoRole)
    {
        while (object) {
            QQuickAccessibleAttached *att = QQuickAccessibleAttached::attachedProperties(object);
            if (att && (role == QAccessible::NoRole || att->role() == role)) {
                break;
            }
            object = object->parent();
        }
        return object;
    }

    QAccessible::State state() { return m_state; }
    bool ignored() const;
    bool doAction(const QString &actionName);
    void availableActions(QStringList *actions) const;

public Q_SLOTS:
    void valueChanged() {
        QAccessibleValueChangeEvent ev(parent(), parent()->property("value"));
        QAccessible::updateAccessibility(&ev);
    }
    void cursorPositionChanged() {
        QAccessibleTextCursorEvent ev(parent(), parent()->property("cursorPosition").toInt());
        QAccessible::updateAccessibility(&ev);
    }

    void setIgnored(bool ignored);

Q_SIGNALS:
    void roleChanged();
    void nameChanged();
    void descriptionChanged();
    void ignoredChanged();
    void pressAction();
    void toggleAction();
    void increaseAction();
    void decreaseAction();
    void scrollUpAction();
    void scrollDownAction();
    void scrollLeftAction();
    void scrollRightAction();
    void previousPageAction();
    void nextPageAction();

private:
    QQuickItem *item() const { return static_cast<QQuickItem*>(parent()); }

    QAccessible::Role m_role;
    QAccessible::State m_state;
    QString m_name;
    QString m_description;

    static QMetaMethod sigPress;
    static QMetaMethod sigToggle;
    static QMetaMethod sigIncrease;
    static QMetaMethod sigDecrease;
    static QMetaMethod sigScrollUp;
    static QMetaMethod sigScrollDown;
    static QMetaMethod sigScrollLeft;
    static QMetaMethod sigScrollRight;
    static QMetaMethod sigPreviousPage;
    static QMetaMethod sigNextPage;

public:
    using QObject::property;
};


QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickAccessibleAttached)
QML_DECLARE_TYPEINFO(QQuickAccessibleAttached, QML_HAS_ATTACHED_PROPERTIES)

#endif // QT_NO_ACCESSIBILITY

#endif
