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

#ifndef QQMLDATAMODEL_P_H
#define QQMLDATAMODEL_P_H

#include <private/qtqmlglobal_p.h>
#include <private/qqmllistcompositor_p.h>
#include <private/qqmlobjectmodel_p.h>

#include <QtCore/qabstractitemmodel.h>
#include <QtCore/qstringlist.h>

#include <private/qv8engine_p.h>
#include <private/qqmlglobal_p.h>

QT_BEGIN_NAMESPACE

class QQmlChangeSet;
class QQmlComponent;
class QQuickPackage;
class QQmlV4Function;
class QQmlDelegateModelGroup;
class QQmlDelegateModelAttached;
class QQmlDelegateModelPrivate;


class Q_QML_PRIVATE_EXPORT QQmlDelegateModel : public QQmlInstanceModel, public QQmlParserStatus
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQmlDelegateModel)

    Q_PROPERTY(QVariant model READ model WRITE setModel)
    Q_PROPERTY(QQmlComponent *delegate READ delegate WRITE setDelegate)
    Q_PROPERTY(QString filterOnGroup READ filterGroup WRITE setFilterGroup NOTIFY filterGroupChanged RESET resetFilterGroup)
    Q_PROPERTY(QQmlDelegateModelGroup *items READ items CONSTANT) //TODO : worth renaming?
    Q_PROPERTY(QQmlDelegateModelGroup *persistedItems READ persistedItems CONSTANT)
    Q_PROPERTY(QQmlListProperty<QQmlDelegateModelGroup> groups READ groups CONSTANT)
    Q_PROPERTY(QObject *parts READ parts CONSTANT)
    Q_PROPERTY(QVariant rootIndex READ rootIndex WRITE setRootIndex NOTIFY rootIndexChanged)
    Q_CLASSINFO("DefaultProperty", "delegate")
    Q_INTERFACES(QQmlParserStatus)
public:
    QQmlDelegateModel();
    QQmlDelegateModel(QQmlContext *, QObject *parent=0);
    virtual ~QQmlDelegateModel();

    void classBegin();
    void componentComplete();

    QVariant model() const;
    void setModel(const QVariant &);

    QQmlComponent *delegate() const;
    void setDelegate(QQmlComponent *);

    QVariant rootIndex() const;
    void setRootIndex(const QVariant &root);

    Q_INVOKABLE QVariant modelIndex(int idx) const;
    Q_INVOKABLE QVariant parentModelIndex() const;

    int count() const;
    bool isValid() const { return delegate() != 0; }
    QObject *object(int index, bool asynchronous=false);
    ReleaseFlags release(QObject *object);
    void cancel(int index);
    virtual QString stringValue(int index, const QString &role);
    virtual void setWatchedRoles(QList<QByteArray> roles);

    int indexOf(QObject *object, QObject *objectContext) const;

    QString filterGroup() const;
    void setFilterGroup(const QString &group);
    void resetFilterGroup();

    QQmlDelegateModelGroup *items();
    QQmlDelegateModelGroup *persistedItems();
    QQmlListProperty<QQmlDelegateModelGroup> groups();
    QObject *parts();

    bool event(QEvent *);

    static QQmlDelegateModelAttached *qmlAttachedProperties(QObject *obj);

Q_SIGNALS:
    void filterGroupChanged();
    void defaultGroupsChanged();
    void rootIndexChanged();

private Q_SLOTS:
    void _q_itemsChanged(int index, int count, const QVector<int> &roles);
    void _q_itemsInserted(int index, int count);
    void _q_itemsRemoved(int index, int count);
    void _q_itemsMoved(int from, int to, int count);
    void _q_modelReset();
    void _q_rowsInserted(const QModelIndex &,int,int);
    void _q_rowsAboutToBeRemoved(const QModelIndex &parent, int begin, int end);
    void _q_rowsRemoved(const QModelIndex &,int,int);
    void _q_rowsMoved(const QModelIndex &, int, int, const QModelIndex &, int);
    void _q_dataChanged(const QModelIndex&,const QModelIndex&,const QVector<int> &);
    void _q_layoutAboutToBeChanged(const QList<QPersistentModelIndex>&, QAbstractItemModel::LayoutChangeHint);
    void _q_layoutChanged(const QList<QPersistentModelIndex>&, QAbstractItemModel::LayoutChangeHint);

private:
    bool isDescendantOf(const QPersistentModelIndex &desc, const QList<QPersistentModelIndex> &parents) const;

    Q_DISABLE_COPY(QQmlDelegateModel)
};

class QQmlDelegateModelGroupPrivate;
class Q_QML_PRIVATE_EXPORT QQmlDelegateModelGroup : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(bool includeByDefault READ defaultInclude WRITE setDefaultInclude NOTIFY defaultIncludeChanged)
public:
    QQmlDelegateModelGroup(QObject *parent = 0);
    QQmlDelegateModelGroup(const QString &name, QQmlDelegateModel *model, int compositorType, QObject *parent = 0);
    ~QQmlDelegateModelGroup();

    QString name() const;
    void setName(const QString &name);

    int count() const;

    bool defaultInclude() const;
    void setDefaultInclude(bool include);

    Q_INVOKABLE QQmlV4Handle get(int index);

public Q_SLOTS:
    void insert(QQmlV4Function *);
    void create(QQmlV4Function *);
    void resolve(QQmlV4Function *);
    void remove(QQmlV4Function *);
    void addGroups(QQmlV4Function *);
    void removeGroups(QQmlV4Function *);
    void setGroups(QQmlV4Function *);
    void move(QQmlV4Function *);

Q_SIGNALS:
    void countChanged();
    void nameChanged();
    void defaultIncludeChanged();
    void changed(const QQmlV4Handle &removed, const QQmlV4Handle &inserted);
private:
    Q_DECLARE_PRIVATE(QQmlDelegateModelGroup)
};

class QQmlDelegateModelItem;
class QQmlDelegateModelAttachedMetaObject;
class QQmlDelegateModelAttached : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QQmlDelegateModel *model READ model CONSTANT)
    Q_PROPERTY(QStringList groups READ groups WRITE setGroups NOTIFY groupsChanged)
    Q_PROPERTY(bool isUnresolved READ isUnresolved NOTIFY unresolvedChanged)
public:
    QQmlDelegateModelAttached(QObject *parent);
    QQmlDelegateModelAttached(QQmlDelegateModelItem *cacheItem, QObject *parent);
    ~QQmlDelegateModelAttached() {}

    void setCacheItem(QQmlDelegateModelItem *item);

    QQmlDelegateModel *model() const;

    QStringList groups() const;
    void setGroups(const QStringList &groups);

    bool isUnresolved() const;

    void emitChanges();

    void emitUnresolvedChanged() { Q_EMIT unresolvedChanged(); }

Q_SIGNALS:
    void groupsChanged();
    void unresolvedChanged();

public:
    QQmlDelegateModelItem *m_cacheItem;
    int m_previousGroups;
    int m_currentIndex[QQmlListCompositor::MaximumGroupCount];
    int m_previousIndex[QQmlListCompositor::MaximumGroupCount];

    friend class QQmlDelegateModelAttachedMetaObject;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQmlDelegateModel)
QML_DECLARE_TYPEINFO(QQmlDelegateModel, QML_HAS_ATTACHED_PROPERTIES)
QML_DECLARE_TYPE(QQmlDelegateModelGroup)

#endif // QQMLDATAMODEL_P_H
