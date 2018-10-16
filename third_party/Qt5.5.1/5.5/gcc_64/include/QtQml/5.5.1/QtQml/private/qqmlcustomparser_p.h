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

#ifndef QQMLCUSTOMPARSER_H
#define QQMLCUSTOMPARSER_H

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

#include "qqmlmetatype_p.h"
#include "qqmlerror.h"
#include "qqmlbinding_p.h"

#include <QtCore/qbytearray.h>
#include <QtCore/qxmlstream.h>

QT_BEGIN_NAMESPACE

class QQmlCompiledData;

struct QQmlCustomParserCompilerBackend
{
    virtual ~QQmlCustomParserCompilerBackend() {}
    virtual const QQmlImports &imports() const = 0;

    int evaluateEnum(const QString &scope, const QByteArray& enumValue, bool *ok) const;
    const QMetaObject *resolveType(const QString& name) const;
};

class Q_QML_PRIVATE_EXPORT QQmlCustomParser
{
public:
    enum Flag {
        NoFlag                    = 0x00000000,
        AcceptsAttachedProperties = 0x00000001,
        AcceptsSignalHandlers     = 0x00000002
    };
    Q_DECLARE_FLAGS(Flags, Flag)

    QQmlCustomParser() : compiler(0), m_flags(NoFlag) {}
    QQmlCustomParser(Flags f) : compiler(0), m_flags(f) {}
    virtual ~QQmlCustomParser() {}

    void clearErrors();
    Flags flags() const { return m_flags; }

    virtual void verifyBindings(const QV4::CompiledData::Unit *, const QList<const QV4::CompiledData::Binding *> &) = 0;
    virtual void applyBindings(QObject *, QQmlCompiledData *, const QList<const QV4::CompiledData::Binding *> &) = 0;

    QList<QQmlError> errors() const { return exceptions; }

protected:
    void error(const QV4::CompiledData::Binding *binding, const QString& description)
    { error(binding->location, description); }
    void error(const QV4::CompiledData::Object *object, const QString& description)
    { error(object->location, description); }
    void error(const QV4::CompiledData::Location &location, const QString& description);

    int evaluateEnum(const QByteArray&, bool *ok) const;

    const QMetaObject *resolveType(const QString&) const;

private:
    QList<QQmlError> exceptions;
    const QQmlCustomParserCompilerBackend *compiler;
    Flags m_flags;
    QBiPointer<const QQmlImports, QQmlTypeNameCache> imports;
    friend class QQmlPropertyValidator;
    friend class QQmlObjectCreator;
};
Q_DECLARE_OPERATORS_FOR_FLAGS(QQmlCustomParser::Flags)

#if 0
#define QML_REGISTER_CUSTOM_TYPE(URI, VERSION_MAJ, VERSION_MIN, NAME, TYPE, CUSTOMTYPE) \
            qmlRegisterCustomType<TYPE>(#URI, VERSION_MAJ, VERSION_MIN, #NAME, #TYPE, new CUSTOMTYPE)
#endif

QT_END_NAMESPACE

#endif
