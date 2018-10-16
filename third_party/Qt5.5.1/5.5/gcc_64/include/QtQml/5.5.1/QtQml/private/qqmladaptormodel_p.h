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

#ifndef QQMLADAPTORMODEL_P_H
#define QQMLADAPTORMODEL_P_H

#include <QtCore/qabstractitemmodel.h>

#include "private/qqmllistaccessor_p.h"

#include <private/qqmlguard_p.h>

QT_BEGIN_NAMESPACE

class QQmlEngine;

class QQmlDelegateModel;
class QQmlDelegateModelItem;
class QQmlDelegateModelItemMetaType;

class QQmlAdaptorModel : public QQmlGuard<QObject>
{
public:
    class Accessors
    {
    public:
        inline Accessors() {}
        virtual ~Accessors();
        virtual int count(const QQmlAdaptorModel &) const { return 0; }
        virtual void cleanup(QQmlAdaptorModel &, QQmlDelegateModel * = 0) const {}

        virtual QVariant value(const QQmlAdaptorModel &, int, const QString &) const {
            return QVariant(); }

        virtual QQmlDelegateModelItem *createItem(
                QQmlAdaptorModel &,
                QQmlDelegateModelItemMetaType *,
                QQmlEngine *,
                int) const { return 0; }

        virtual bool notify(
                const QQmlAdaptorModel &,
                const QList<QQmlDelegateModelItem *> &,
                int,
                int,
                const QVector<int> &) const { return false; }
        virtual void replaceWatchedRoles(
                QQmlAdaptorModel &,
                const QList<QByteArray> &,
                const QList<QByteArray> &) const {}
        virtual QVariant parentModelIndex(const QQmlAdaptorModel &) const {
            return QVariant(); }
        virtual QVariant modelIndex(const QQmlAdaptorModel &, int) const {
            return QVariant(); }
        virtual bool canFetchMore(const QQmlAdaptorModel &) const { return false; }
        virtual void fetchMore(QQmlAdaptorModel &) const {}
    };

    const Accessors *accessors;
    QPersistentModelIndex rootIndex;
    QQmlListAccessor list;

    QQmlAdaptorModel();
    ~QQmlAdaptorModel();

    inline QVariant model() const { return list.list(); }
    void setModel(const QVariant &variant, QQmlDelegateModel *vdm, QQmlEngine *engine);
    void invalidateModel(QQmlDelegateModel *vdm);

    bool isValid() const;

    inline QAbstractItemModel *aim() { return static_cast<QAbstractItemModel *>(object()); }
    inline const QAbstractItemModel *aim() const { return static_cast<const QAbstractItemModel *>(object()); }

    inline int count() const { return qMax(0, accessors->count(*this)); }
    inline QVariant value(int index, const QString &role) const {
        return accessors->value(*this, index, role); }
    inline QQmlDelegateModelItem *createItem(QQmlDelegateModelItemMetaType *metaType, QQmlEngine *engine, int index) {
        return accessors->createItem(*this, metaType, engine, index); }
    inline bool hasProxyObject() const {
        return list.type() == QQmlListAccessor::Instance || list.type() == QQmlListAccessor::ListProperty; }

    inline bool notify(
            const QList<QQmlDelegateModelItem *> &items,
            int index,
            int count,
            const QVector<int> &roles) const {
        return accessors->notify(*this, items, index, count, roles); }
    inline void replaceWatchedRoles(
            const QList<QByteArray> &oldRoles, const QList<QByteArray> &newRoles) {
        accessors->replaceWatchedRoles(*this, oldRoles, newRoles); }

    inline QVariant modelIndex(int index) const { return accessors->modelIndex(*this, index); }
    inline QVariant parentModelIndex() const { return accessors->parentModelIndex(*this); }
    inline bool canFetchMore() const { return accessors->canFetchMore(*this); }
    inline void fetchMore() { return accessors->fetchMore(*this); }

protected:
    void objectDestroyed(QObject *);
};

class QQmlAdaptorModelProxyInterface
{
public:
    virtual ~QQmlAdaptorModelProxyInterface() {}

    virtual QObject *proxiedObject() = 0;
};

#define QQmlAdaptorModelProxyInterface_iid "org.qt-project.Qt.QQmlAdaptorModelProxyInterface"

Q_DECLARE_INTERFACE(QQmlAdaptorModelProxyInterface, QQmlAdaptorModelProxyInterface_iid)

QT_END_NAMESPACE

#endif
