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

#ifndef QQMLPROPERTY_P_H
#define QQMLPROPERTY_P_H

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

#include "qqmlproperty.h"
#include "qqmlengine.h"

#include <private/qobject_p.h>
#include <private/qtqmlglobal_p.h>
#include <private/qqmlpropertycache_p.h>
#include <private/qqmlboundsignalexpressionpointer_p.h>

QT_BEGIN_NAMESPACE

class QQmlContext;
class QQmlEnginePrivate;
class QQmlJavaScriptExpression;
class Q_QML_PRIVATE_EXPORT QQmlPropertyPrivate : public QQmlRefCount
{
public:
    enum WriteFlag {
        BypassInterceptor = 0x01,
        DontRemoveBinding = 0x02,
        RemoveBindingOnAliasWrite = 0x04
    };
    Q_DECLARE_FLAGS(WriteFlags, WriteFlag)

    QQmlContextData *context;
    QPointer<QQmlEngine> engine;
    QPointer<QObject> object;

    QQmlPropertyData core;

    bool isNameCached:1;
    QString nameCache;

    QQmlPropertyPrivate();

    inline QQmlContextData *effectiveContext() const;

    void initProperty(QObject *obj, const QString &name);
    void initDefault(QObject *obj);

    bool isValueType() const;
    int propertyType() const;
    QQmlProperty::Type type() const;
    QQmlProperty::PropertyTypeCategory propertyTypeCategory() const;

    QVariant readValueProperty();
    bool writeValueProperty(const QVariant &, WriteFlags);

    static QQmlMetaObject rawMetaObjectForType(QQmlEnginePrivate *, int);
    static bool writeEnumProperty(const QMetaProperty &prop, int idx, QObject *object,
                                  const QVariant &value, int flags);
    static bool writeValueProperty(QObject *,
                                   const QQmlPropertyData &,
                                   const QVariant &, QQmlContextData *,
                                   WriteFlags flags = 0);
    static bool write(QObject *, const QQmlPropertyData &, const QVariant &,
                      QQmlContextData *, WriteFlags flags = 0);
    static void findAliasTarget(QObject *, int, QObject **, int *);

    static QQmlAbstractBinding *setBinding(QObject *, int coreIndex,
                                                   int valueTypeIndex /* -1 */,
                                                   QQmlAbstractBinding *,
                                                   WriteFlags flags = DontRemoveBinding);
    static QQmlAbstractBinding *setBindingNoEnable(QObject *, int coreIndex,
                                                           int valueTypeIndex /* -1 */,
                                                           QQmlAbstractBinding *);
    static QQmlAbstractBinding *binding(QObject *, int coreIndex,
                                                int valueTypeIndex /* -1 */);

    static QQmlPropertyData saveValueType(const QQmlPropertyData &,
                                          const QMetaObject *, int,
                                          QQmlEngine *);
    static QQmlProperty restore(QObject *,
                                        const QQmlPropertyData &,
                                        QQmlContextData *);

    int signalIndex() const;

    static inline QQmlPropertyPrivate *get(const QQmlProperty &p) {
        return p.d;
    }

    // "Public" (to QML) methods
    static QQmlAbstractBinding *binding(const QQmlProperty &that);
    static QQmlAbstractBinding *setBinding(const QQmlProperty &that,
                                                   QQmlAbstractBinding *,
                                                   WriteFlags flags = DontRemoveBinding);
    static QQmlBoundSignalExpression *signalExpression(const QQmlProperty &that);
    static QQmlBoundSignalExpressionPointer setSignalExpression(const QQmlProperty &that,
                                                                QQmlBoundSignalExpression *);
    static QQmlBoundSignalExpressionPointer takeSignalExpression(const QQmlProperty &that,
                                                                 QQmlBoundSignalExpression *);
    static bool write(const QQmlProperty &that, const QVariant &, WriteFlags);
    static bool writeBinding(QObject *, const QQmlPropertyData &,
                             QQmlContextData *context,
                             QQmlJavaScriptExpression *expression,
                             const QV4::Value &result, bool isUndefined,
                             WriteFlags flags);
    static int valueTypeCoreIndex(const QQmlProperty &that);
    static int bindingIndex(const QQmlProperty &that);
    static int bindingIndex(const QQmlPropertyData &that);
    static QMetaMethod findSignalByName(const QMetaObject *mo, const QByteArray &);
    static bool connect(const QObject *sender, int signal_index,
                        const QObject *receiver, int method_index,
                        int type = 0, int *types = 0);
    static void flushSignal(const QObject *sender, int signal_index);
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QQmlPropertyPrivate::WriteFlags)

QT_END_NAMESPACE

#endif // QQMLPROPERTY_P_H
