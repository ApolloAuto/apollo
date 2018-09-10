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

#ifndef QQMLABSTRACTBINDING_P_H
#define QQMLABSTRACTBINDING_P_H

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

#include <QtCore/qsharedpointer.h>
#include <private/qtqmlglobal_p.h>
#include <private/qqmlproperty_p.h>
#include <private/qpointervaluepair_p.h>

QT_BEGIN_NAMESPACE

class QQmlObjectCreator;

class Q_QML_PRIVATE_EXPORT QQmlAbstractBinding
{
public:
    enum DestroyMode {

        // The binding should disconnect itself upon destroy
        DisconnectBinding,

        // The binding doesn't need to disconnect itself, but it can if it wants to.
        //
        // This is used in QQmlData::destroyed() - at the point at which the bindings are
        // destroyed, the notifiers are already disconnected, so no need to disconnect each
        // binding again.
        //
        // Bindings can use this flag to speed up destruction, especially for v4 bindings
        // disconnecting a single binding might be slow.
        KeepBindingConnected
    };

    struct VTable {
        void (*destroy)(QQmlAbstractBinding *, DestroyMode destroyMode);
        QString (*expression)(const QQmlAbstractBinding *);
        int (*propertyIndex)(const QQmlAbstractBinding *);
        QObject *(*object)(const QQmlAbstractBinding *);
        void (*setEnabled)(QQmlAbstractBinding *, bool, QQmlPropertyPrivate::WriteFlags);
        void (*update)(QQmlAbstractBinding *, QQmlPropertyPrivate::WriteFlags);
        void (*retargetBinding)(QQmlAbstractBinding *, QObject *, int);
    };

    typedef QWeakPointer<QQmlAbstractBinding> Pointer;

    enum BindingType { Binding = 0, ValueTypeProxy = 1 };
    inline BindingType bindingType() const;

    // Destroy the binding.  Use this instead of calling delete.
    // Bindings are free to implement their own memory management, so the delete operator is
    // not necessarily safe.  The default implementation clears the binding, removes it from
    // the object and calls delete.
    void destroy(DestroyMode destroyMode = DisconnectBinding)
    { vtable()->destroy(this, destroyMode); }

    QString expression() const { return vtable()->expression(this); }

    // Should return the encoded property index for the binding.  Should return this value
    // even if the binding is not enabled or added to an object.
    // Encoding is:  coreIndex | (valueTypeIndex << 16)
    int propertyIndex() const { return vtable()->propertyIndex(this); }

    // Should return the object for the binding.  Should return this object even if the
    // binding is not enabled or added to the object.
    QObject *object() const { return vtable()->object(this); }

    void setEnabled(bool e) { setEnabled(e, QQmlPropertyPrivate::DontRemoveBinding); }
    void setEnabled(bool e, QQmlPropertyPrivate::WriteFlags f) { vtable()->setEnabled(this, e, f); }

    void update() { update(QQmlPropertyPrivate::DontRemoveBinding); }
    void update(QQmlPropertyPrivate::WriteFlags f) { vtable()->update(this, f); }

    void addToObject();
    void removeFromObject();

    static inline Pointer getPointer(QQmlAbstractBinding *p);
    static void printBindingLoopError(QQmlProperty &prop);

    // Default implementation for some VTable functions
    template<typename T>
    static void default_destroy(QQmlAbstractBinding *, DestroyMode);
    static QString default_expression(const QQmlAbstractBinding *);
    static void default_retargetBinding(QQmlAbstractBinding *, QObject *, int);

protected:
    QQmlAbstractBinding(BindingType);
    ~QQmlAbstractBinding();
    void clear();

    // Called by QQmlPropertyPrivate to "move" a binding to a different property.
    // This is only used for alias properties. The default implementation qFatal()'s
    // to ensure that the method is never called for binding types that don't support it.
    void retargetBinding(QObject *o, int i) { vtable()->retargetBinding(this, o, i); }

private:
    Pointer weakPointer();

    friend class QQmlData;
    friend class QQmlComponentPrivate;
    friend class QQmlValueTypeProxyBinding;
    friend class QQmlPropertyPrivate;
    friend class QtSharedPointer::ExternalRefCount<QQmlAbstractBinding>;
    friend class QV4Bindings;
    friend class QQmlObjectCreator;

    typedef QSharedPointer<QQmlAbstractBinding> SharedPointer;
    // To save memory, we also store the rarely used weakPointer() instance in here
    // We also use the flag bits:
    //    m_mePtr.flag1: added to object
    QPointerValuePair<QQmlAbstractBinding*, SharedPointer> m_mePtr;

    inline void setAddedToObject(bool v);
    inline bool isAddedToObject() const;

    inline QQmlAbstractBinding *nextBinding() const;
    inline void setNextBinding(QQmlAbstractBinding *);

    // Pointer to the next binding in the linked list of bindings.
    // Being a pointer, the address is always aligned to at least 4 bytes, which means the last two
    // bits of the pointer are free to be used for something else. They are used to store the binding
    // type. The binding type serves as an index into the static vTables array, which is used instead
    // of a compiler-generated vTable. Instead of virtual functions, pointers to static functions in
    // the vTables array are used for dispatching.
    // This saves a compiler-generated pointer to a compiler-generated vTable, and thus reduces
    // the binding object size by sizeof(void*).
    qintptr m_nextBindingPtr;

    static VTable *vTables[];
    inline const VTable *vtable() const { return vTables[bindingType()]; }
};

QQmlAbstractBinding::Pointer
QQmlAbstractBinding::getPointer(QQmlAbstractBinding *p)
{
    return p ? p->weakPointer() : Pointer();
}

void QQmlAbstractBinding::setAddedToObject(bool v)
{
    m_mePtr.setFlagValue(v);
}

bool QQmlAbstractBinding::isAddedToObject() const
{
    return m_mePtr.flag();
}

QQmlAbstractBinding *QQmlAbstractBinding::nextBinding() const
{
    return (QQmlAbstractBinding *)(m_nextBindingPtr & ~0x3);
}

void QQmlAbstractBinding::setNextBinding(QQmlAbstractBinding *b)
{
    m_nextBindingPtr = qintptr(b) | (m_nextBindingPtr & 0x3);
}

QQmlAbstractBinding::BindingType QQmlAbstractBinding::bindingType() const
{
    return (BindingType)(m_nextBindingPtr & 0x3);
}

template<typename T>
void QQmlAbstractBinding::default_destroy(QQmlAbstractBinding *This, DestroyMode mode)
{
    // Assume the binding disconnects itself in the destructor, which for example QQmlBinding
    // does in the destructor of its base class, QQmlJavaScriptExpression
    Q_UNUSED(mode);

    This->removeFromObject();
    This->clear();
    delete static_cast<T *>(This);
}

QT_END_NAMESPACE

#endif // QQMLABSTRACTBINDING_P_H
