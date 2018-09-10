/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Designer of the Qt Toolkit.
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
// This file is not part of the Qt API.  It exists for the convenience
// of Qt Designer.  This header
// file may change from version to version without notice, or even be removed.
//
// We mean it.
//

#ifndef ABSTRACTMETAOBJECT_H
#define ABSTRACTMETAOBJECT_H

#include <QtDesigner/sdk_global.h>
#include <QtCore/QVariant>
#include <QtCore/QFlags>

QT_BEGIN_NAMESPACE

class QDESIGNER_SDK_EXPORT QDesignerMetaEnumInterface
{
public:
    QDesignerMetaEnumInterface();
    virtual ~QDesignerMetaEnumInterface();
    virtual bool isFlag() const = 0;
    virtual QString key(int index) const = 0;
    virtual int keyCount() const = 0;
    virtual int keyToValue(const QString &key) const = 0;
    virtual int keysToValue(const QString &keys) const = 0;
    virtual QString name() const = 0;
    virtual QString scope() const = 0;
    virtual QString separator() const = 0;
    virtual int value(int index) const = 0;
    virtual QString valueToKey(int value) const = 0;
    virtual QString valueToKeys(int value) const  = 0;
};

class QDESIGNER_SDK_EXPORT QDesignerMetaPropertyInterface
{
public:
    enum Kind { EnumKind, FlagKind, OtherKind };
    enum AccessFlag { ReadAccess = 0x0001, WriteAccess = 0x0002, ResetAccess = 0x0004 };
    enum Attribute { DesignableAttribute = 0x0001, ScriptableAttribute = 0x0002, StoredAttribute = 0x0004, UserAttribute = 0x0008};
    Q_DECLARE_FLAGS(Attributes, Attribute)
    Q_DECLARE_FLAGS(AccessFlags, AccessFlag)

    QDesignerMetaPropertyInterface();
    virtual ~QDesignerMetaPropertyInterface();

    virtual const QDesignerMetaEnumInterface *enumerator() const = 0;

    virtual Kind kind() const = 0;
    virtual AccessFlags accessFlags() const = 0;
    virtual Attributes attributes(const QObject *object = 0) const = 0;

    virtual QVariant::Type type() const = 0;
    virtual QString name() const = 0;
    virtual QString typeName() const = 0;
    virtual int userType() const = 0;
    virtual bool hasSetter() const = 0;

    virtual QVariant read(const QObject *object) const = 0;
    virtual bool reset(QObject *object) const = 0;
    virtual bool write(QObject *object, const QVariant &value) const  = 0;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QDesignerMetaPropertyInterface::AccessFlags)
Q_DECLARE_OPERATORS_FOR_FLAGS(QDesignerMetaPropertyInterface::Attributes)

class QDESIGNER_SDK_EXPORT QDesignerMetaMethodInterface
{
public:
    QDesignerMetaMethodInterface();
    virtual ~QDesignerMetaMethodInterface();

    enum MethodType { Method, Signal, Slot, Constructor };
    enum Access { Private, Protected, Public };

    virtual Access access() const  = 0;
    virtual MethodType methodType() const = 0;
    virtual QStringList parameterNames() const = 0;
    virtual QStringList parameterTypes() const = 0;
    virtual QString signature() const = 0;
    virtual QString normalizedSignature() const = 0;
    virtual QString tag() const = 0;
    virtual QString typeName() const  = 0;
};

class QDESIGNER_SDK_EXPORT QDesignerMetaObjectInterface {
public:
    QDesignerMetaObjectInterface();
    virtual ~QDesignerMetaObjectInterface();

    virtual QString className() const = 0;
    virtual const QDesignerMetaEnumInterface *enumerator(int index) const = 0;
    virtual int enumeratorCount() const = 0;
    virtual int enumeratorOffset() const = 0;

    virtual int indexOfEnumerator(const QString &name) const = 0;
    virtual int indexOfMethod(const QString &method) const = 0;
    virtual int indexOfProperty(const QString &name) const = 0;
    virtual int indexOfSignal(const QString &signal) const = 0;
    virtual int indexOfSlot(const QString &slot) const = 0;

    virtual const QDesignerMetaMethodInterface *method(int index) const = 0;
    virtual int methodCount() const = 0;
    virtual int methodOffset() const = 0;

    virtual const  QDesignerMetaPropertyInterface *property(int index) const = 0;
    virtual int propertyCount() const = 0;
    virtual int propertyOffset() const = 0;

    virtual const QDesignerMetaObjectInterface *superClass() const = 0;
    virtual const QDesignerMetaPropertyInterface *userProperty() const  = 0;
};

// To be obtained from core
class QDESIGNER_SDK_EXPORT QDesignerIntrospectionInterface {
public:
    QDesignerIntrospectionInterface();
    virtual ~QDesignerIntrospectionInterface();

    virtual const QDesignerMetaObjectInterface* metaObject(const QObject *object) const = 0;
};

QT_END_NAMESPACE

#endif // ABSTRACTMETAOBJECT_H
