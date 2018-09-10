/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QSORTFILTERPROXYMODEL_H
#define QSORTFILTERPROXYMODEL_H

#include <QtCore/qabstractproxymodel.h>

#ifndef QT_NO_SORTFILTERPROXYMODEL

#include <QtCore/qregexp.h>

QT_BEGIN_NAMESPACE


class QSortFilterProxyModelPrivate;
class QSortFilterProxyModelLessThan;
class QSortFilterProxyModelGreaterThan;

class Q_CORE_EXPORT QSortFilterProxyModel : public QAbstractProxyModel
{
    friend class QSortFilterProxyModelLessThan;
    friend class QSortFilterProxyModelGreaterThan;

    Q_OBJECT
    Q_PROPERTY(QRegExp filterRegExp READ filterRegExp WRITE setFilterRegExp)
    Q_PROPERTY(int filterKeyColumn READ filterKeyColumn WRITE setFilterKeyColumn)
    Q_PROPERTY(bool dynamicSortFilter READ dynamicSortFilter WRITE setDynamicSortFilter)
    Q_PROPERTY(Qt::CaseSensitivity filterCaseSensitivity READ filterCaseSensitivity WRITE setFilterCaseSensitivity)
    Q_PROPERTY(Qt::CaseSensitivity sortCaseSensitivity READ sortCaseSensitivity WRITE setSortCaseSensitivity)
    Q_PROPERTY(bool isSortLocaleAware READ isSortLocaleAware WRITE setSortLocaleAware)
    Q_PROPERTY(int sortRole READ sortRole WRITE setSortRole)
    Q_PROPERTY(int filterRole READ filterRole WRITE setFilterRole)

public:
    explicit QSortFilterProxyModel(QObject *parent = 0);
    ~QSortFilterProxyModel();

    void setSourceModel(QAbstractItemModel *sourceModel) Q_DECL_OVERRIDE;

    QModelIndex mapToSource(const QModelIndex &proxyIndex) const Q_DECL_OVERRIDE;
    QModelIndex mapFromSource(const QModelIndex &sourceIndex) const Q_DECL_OVERRIDE;

    QItemSelection mapSelectionToSource(const QItemSelection &proxySelection) const Q_DECL_OVERRIDE;
    QItemSelection mapSelectionFromSource(const QItemSelection &sourceSelection) const Q_DECL_OVERRIDE;

    QRegExp filterRegExp() const;
    void setFilterRegExp(const QRegExp &regExp);

    int filterKeyColumn() const;
    void setFilterKeyColumn(int column);

    Qt::CaseSensitivity filterCaseSensitivity() const;
    void setFilterCaseSensitivity(Qt::CaseSensitivity cs);

    Qt::CaseSensitivity sortCaseSensitivity() const;
    void setSortCaseSensitivity(Qt::CaseSensitivity cs);

    bool isSortLocaleAware() const;
    void setSortLocaleAware(bool on);

    int sortColumn() const;
    Qt::SortOrder sortOrder() const;

    bool dynamicSortFilter() const;
    void setDynamicSortFilter(bool enable);

    int sortRole() const;
    void setSortRole(int role);

    int filterRole() const;
    void setFilterRole(int role);

public Q_SLOTS:
    void setFilterRegExp(const QString &pattern);
    void setFilterWildcard(const QString &pattern);
    void setFilterFixedString(const QString &pattern);
    void clear();
    void invalidate();

protected:
    virtual bool filterAcceptsRow(int source_row, const QModelIndex &source_parent) const;
    virtual bool filterAcceptsColumn(int source_column, const QModelIndex &source_parent) const;
    virtual bool lessThan(const QModelIndex &source_left, const QModelIndex &source_right) const;

    void filterChanged();
    void invalidateFilter();

public:
    using QObject::parent;

    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    QModelIndex parent(const QModelIndex &child) const Q_DECL_OVERRIDE;
    QModelIndex sibling(int row, int column, const QModelIndex &idx) const Q_DECL_OVERRIDE;

    int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    bool hasChildren(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) Q_DECL_OVERRIDE;

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    bool setHeaderData(int section, Qt::Orientation orientation,
            const QVariant &value, int role = Qt::EditRole) Q_DECL_OVERRIDE;

    QMimeData *mimeData(const QModelIndexList &indexes) const Q_DECL_OVERRIDE;
    bool dropMimeData(const QMimeData *data, Qt::DropAction action,
                      int row, int column, const QModelIndex &parent) Q_DECL_OVERRIDE;

    bool insertRows(int row, int count, const QModelIndex &parent = QModelIndex()) Q_DECL_OVERRIDE;
    bool insertColumns(int column, int count, const QModelIndex &parent = QModelIndex()) Q_DECL_OVERRIDE;
    bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) Q_DECL_OVERRIDE;
    bool removeColumns(int column, int count, const QModelIndex &parent = QModelIndex()) Q_DECL_OVERRIDE;

    void fetchMore(const QModelIndex &parent) Q_DECL_OVERRIDE;
    bool canFetchMore(const QModelIndex &parent) const Q_DECL_OVERRIDE;
    Qt::ItemFlags flags(const QModelIndex &index) const Q_DECL_OVERRIDE;

    QModelIndex buddy(const QModelIndex &index) const Q_DECL_OVERRIDE;
    QModelIndexList match(const QModelIndex &start, int role,
                          const QVariant &value, int hits = 1,
                          Qt::MatchFlags flags =
                          Qt::MatchFlags(Qt::MatchStartsWith|Qt::MatchWrap)) const Q_DECL_OVERRIDE;
    QSize span(const QModelIndex &index) const Q_DECL_OVERRIDE;
    void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) Q_DECL_OVERRIDE;

    QStringList mimeTypes() const Q_DECL_OVERRIDE;
    Qt::DropActions supportedDropActions() const Q_DECL_OVERRIDE;
private:
    Q_DECLARE_PRIVATE(QSortFilterProxyModel)
    Q_DISABLE_COPY(QSortFilterProxyModel)

    Q_PRIVATE_SLOT(d_func(), void _q_sourceDataChanged(const QModelIndex &source_top_left, const QModelIndex &source_bottom_right, const QVector<int> &roles))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceHeaderDataChanged(Qt::Orientation orientation, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceAboutToBeReset())
    Q_PRIVATE_SLOT(d_func(), void _q_sourceReset())
    Q_PRIVATE_SLOT(d_func(), void _q_sourceLayoutAboutToBeChanged(const QList<QPersistentModelIndex> &sourceParents, QAbstractItemModel::LayoutChangeHint hint))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceLayoutChanged(const QList<QPersistentModelIndex> &sourceParents, QAbstractItemModel::LayoutChangeHint hint))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeInserted(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsInserted(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeRemoved(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsRemoved(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceRowsMoved(QModelIndex,int,int,QModelIndex,int))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsAboutToBeInserted(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsInserted(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsAboutToBeRemoved(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsRemoved(const QModelIndex &source_parent, int start, int end))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsAboutToBeMoved(QModelIndex,int,int,QModelIndex,int))
    Q_PRIVATE_SLOT(d_func(), void _q_sourceColumnsMoved(QModelIndex,int,int,QModelIndex,int))
    Q_PRIVATE_SLOT(d_func(), void _q_clearMapping())
};

QT_END_NAMESPACE

#endif // QT_NO_SORTFILTERPROXYMODEL

#endif // QSORTFILTERPROXYMODEL_H
