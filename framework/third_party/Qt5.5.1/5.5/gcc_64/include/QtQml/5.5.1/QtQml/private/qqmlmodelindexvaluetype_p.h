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

#ifndef QQMLMODELINDEXVALUETYPE_P_H
#define QQMLMODELINDEXVALUETYPE_P_H

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

#include <QtCore/qabstractitemmodel.h>
#include <QtCore/qitemselectionmodel.h>

QT_BEGIN_NAMESPACE

struct QQmlModelIndexValueType
{
    QModelIndex v;

    Q_PROPERTY(int row READ row CONSTANT FINAL)
    Q_PROPERTY(int column READ column CONSTANT FINAL)
    Q_PROPERTY(QModelIndex parent READ parent FINAL)
    Q_PROPERTY(bool valid READ isValid CONSTANT FINAL)
    Q_PROPERTY(QAbstractItemModel *model READ model CONSTANT FINAL)
    Q_PROPERTY(quint64 internalId READ internalId CONSTANT FINAL)
    Q_GADGET

public:
    Q_INVOKABLE QString toString() const
    { return QLatin1String("QModelIndex") + propertiesString(v); }

    inline int row() const Q_DECL_NOTHROW { return v.row(); }
    inline int column() const Q_DECL_NOTHROW { return v.column(); }
    inline QModelIndex parent() const { return v.parent(); }
    inline bool isValid() const Q_DECL_NOTHROW { return v.isValid(); }
    inline QAbstractItemModel *model() const Q_DECL_NOTHROW
    { return const_cast<QAbstractItemModel *>(v.model()); }
    quint64 internalId() const { return v.internalId(); }

    static QString propertiesString(const QModelIndex &idx);

    static QPersistentModelIndex toPersistentModelIndex(const QModelIndex &index)
    { return QPersistentModelIndex(index); }
};

struct QQmlPersistentModelIndexValueType
{
    QPersistentModelIndex v;

    Q_PROPERTY(int row READ row FINAL)
    Q_PROPERTY(int column READ column FINAL)
    Q_PROPERTY(QModelIndex parent READ parent FINAL)
    Q_PROPERTY(bool valid READ isValid FINAL)
    Q_PROPERTY(QAbstractItemModel *model READ model FINAL)
    Q_PROPERTY(quint64 internalId READ internalId FINAL)
    Q_GADGET

public:
    Q_INVOKABLE QString toString() const
    { return QLatin1String("QPersistentModelIndex") + QQmlModelIndexValueType::propertiesString(v); }

    inline int row() const { return v.row(); }
    inline int column() const { return v.column(); }
    inline QModelIndex parent() const { return v.parent(); }
    inline bool isValid() const { return v.isValid(); }
    inline QAbstractItemModel *model() const { return const_cast<QAbstractItemModel *>(v.model()); }
    inline quint64 internalId() const { return v.internalId(); }

    static const QModelIndex &toModelIndex(const QPersistentModelIndex &index)
    { return index; }
};

struct QQmlItemSelectionRangeValueType
{
    QItemSelectionRange v;

    Q_PROPERTY(int top READ top FINAL)
    Q_PROPERTY(int left READ left FINAL)
    Q_PROPERTY(int bottom READ bottom FINAL)
    Q_PROPERTY(int right READ right FINAL)
    Q_PROPERTY(int width READ width FINAL)
    Q_PROPERTY(int height READ height FINAL)
    Q_PROPERTY(QPersistentModelIndex topLeft READ topLeft FINAL)
    Q_PROPERTY(QPersistentModelIndex bottomRight READ bottomRight FINAL)
    Q_PROPERTY(QModelIndex parent READ parent FINAL)
    Q_PROPERTY(bool valid READ isValid FINAL)
    Q_PROPERTY(bool empty READ isEmpty FINAL)
    Q_PROPERTY(QAbstractItemModel *model READ model FINAL)
    Q_GADGET

public:
    Q_INVOKABLE QString toString() const;
    Q_INVOKABLE inline bool contains(const QModelIndex &index) const
    { return v.contains(index); }
    Q_INVOKABLE inline bool contains(int row, int column, const QModelIndex &parentIndex) const
    { return v.contains(row, column, parentIndex); }
    Q_INVOKABLE inline bool intersects(const QItemSelectionRange &other) const
    { return v.intersects(other); }
    Q_INVOKABLE QItemSelectionRange intersected(const QItemSelectionRange &other) const
    { return v.intersected(other); }

    inline int top() const { return v.top(); }
    inline int left() const { return v.left(); }
    inline int bottom() const { return v.bottom(); }
    inline int right() const { return v.right(); }
    inline int width() const { return v.width(); }
    inline int height() const { return v.height(); }
    inline QPersistentModelIndex &topLeft() const { return const_cast<QPersistentModelIndex &>(v.topLeft()); }
    inline QPersistentModelIndex &bottomRight() const { return const_cast<QPersistentModelIndex &>(v.bottomRight()); }
    inline QModelIndex parent() const { return v.parent(); }
    inline QAbstractItemModel *model() const { return const_cast<QAbstractItemModel *>(v.model()); }
    inline bool isValid() const { return v.isValid(); }
    inline bool isEmpty() const { return v.isEmpty(); }
};

#undef QLISTVALUETYPE_INVOKABLE_API

QT_END_NAMESPACE

#endif // QQMLMODELINDEXVALUETYPE_P_H

