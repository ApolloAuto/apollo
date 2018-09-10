/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWidgets module of the Qt Toolkit.
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

#ifndef QTABLEVIEW_H
#define QTABLEVIEW_H

#include <QtWidgets/qabstractitemview.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_TABLEVIEW

class QHeaderView;
class QTableViewPrivate;

class Q_WIDGETS_EXPORT QTableView : public QAbstractItemView
{
    Q_OBJECT
    Q_PROPERTY(bool showGrid READ showGrid WRITE setShowGrid)
    Q_PROPERTY(Qt::PenStyle gridStyle READ gridStyle WRITE setGridStyle)
    Q_PROPERTY(bool sortingEnabled READ isSortingEnabled WRITE setSortingEnabled)
    Q_PROPERTY(bool wordWrap READ wordWrap WRITE setWordWrap)
    Q_PROPERTY(bool cornerButtonEnabled READ isCornerButtonEnabled WRITE setCornerButtonEnabled)

public:
    explicit QTableView(QWidget *parent = 0);
    ~QTableView();

    void setModel(QAbstractItemModel *model) Q_DECL_OVERRIDE;
    void setRootIndex(const QModelIndex &index) Q_DECL_OVERRIDE;
    void setSelectionModel(QItemSelectionModel *selectionModel) Q_DECL_OVERRIDE;
    void doItemsLayout() Q_DECL_OVERRIDE;

    QHeaderView *horizontalHeader() const;
    QHeaderView *verticalHeader() const;
    void setHorizontalHeader(QHeaderView *header);
    void setVerticalHeader(QHeaderView *header);

    int rowViewportPosition(int row) const;
    int rowAt(int y) const;

    void setRowHeight(int row, int height);
    int rowHeight(int row) const;

    int columnViewportPosition(int column) const;
    int columnAt(int x) const;

    void setColumnWidth(int column, int width);
    int columnWidth(int column) const;

    bool isRowHidden(int row) const;
    void setRowHidden(int row, bool hide);

    bool isColumnHidden(int column) const;
    void setColumnHidden(int column, bool hide);

    void setSortingEnabled(bool enable);
    bool isSortingEnabled() const;

    bool showGrid() const;

    Qt::PenStyle gridStyle() const;
    void setGridStyle(Qt::PenStyle style);

    void setWordWrap(bool on);
    bool wordWrap() const;

    void setCornerButtonEnabled(bool enable);
    bool isCornerButtonEnabled() const;

    QRect visualRect(const QModelIndex &index) const Q_DECL_OVERRIDE;
    void scrollTo(const QModelIndex &index, ScrollHint hint = EnsureVisible) Q_DECL_OVERRIDE;
    QModelIndex indexAt(const QPoint &p) const Q_DECL_OVERRIDE;

    void setSpan(int row, int column, int rowSpan, int columnSpan);
    int rowSpan(int row, int column) const;
    int columnSpan(int row, int column) const;
    void clearSpans();

    void sortByColumn(int column, Qt::SortOrder order);

public Q_SLOTS:
    void selectRow(int row);
    void selectColumn(int column);
    void hideRow(int row);
    void hideColumn(int column);
    void showRow(int row);
    void showColumn(int column);
    void resizeRowToContents(int row);
    void resizeRowsToContents();
    void resizeColumnToContents(int column);
    void resizeColumnsToContents();
    void sortByColumn(int column);
    void setShowGrid(bool show);

protected Q_SLOTS:
    void rowMoved(int row, int oldIndex, int newIndex);
    void columnMoved(int column, int oldIndex, int newIndex);
    void rowResized(int row, int oldHeight, int newHeight);
    void columnResized(int column, int oldWidth, int newWidth);
    void rowCountChanged(int oldCount, int newCount);
    void columnCountChanged(int oldCount, int newCount);

protected:
    QTableView(QTableViewPrivate &, QWidget *parent);
    void scrollContentsBy(int dx, int dy) Q_DECL_OVERRIDE;

    QStyleOptionViewItem viewOptions() const Q_DECL_OVERRIDE;
    void paintEvent(QPaintEvent *e) Q_DECL_OVERRIDE;

    void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

    int horizontalOffset() const Q_DECL_OVERRIDE;
    int verticalOffset() const Q_DECL_OVERRIDE;
    QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers) Q_DECL_OVERRIDE;

    void setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags command) Q_DECL_OVERRIDE;
    QRegion visualRegionForSelection(const QItemSelection &selection) const Q_DECL_OVERRIDE;
    QModelIndexList selectedIndexes() const Q_DECL_OVERRIDE;

    void updateGeometries() Q_DECL_OVERRIDE;

    QSize viewportSizeHint() const Q_DECL_OVERRIDE;

    int sizeHintForRow(int row) const Q_DECL_OVERRIDE;
    int sizeHintForColumn(int column) const Q_DECL_OVERRIDE;

    void verticalScrollbarAction(int action) Q_DECL_OVERRIDE;
    void horizontalScrollbarAction(int action) Q_DECL_OVERRIDE;

    bool isIndexHidden(const QModelIndex &index) const Q_DECL_OVERRIDE;

    void selectionChanged(const QItemSelection &selected,
                          const QItemSelection &deselected) Q_DECL_OVERRIDE;
    void currentChanged(const QModelIndex &current,
                          const QModelIndex &previous) Q_DECL_OVERRIDE;

private:
    friend class QAccessibleItemView;
    int visualIndex(const QModelIndex &index) const;

    Q_DECLARE_PRIVATE(QTableView)
    Q_DISABLE_COPY(QTableView)
    Q_PRIVATE_SLOT(d_func(), void _q_selectRow(int))
    Q_PRIVATE_SLOT(d_func(), void _q_selectColumn(int))
    Q_PRIVATE_SLOT(d_func(), void _q_updateSpanInsertedRows(QModelIndex,int,int))
    Q_PRIVATE_SLOT(d_func(), void _q_updateSpanInsertedColumns(QModelIndex,int,int))
    Q_PRIVATE_SLOT(d_func(), void _q_updateSpanRemovedRows(QModelIndex,int,int))
    Q_PRIVATE_SLOT(d_func(), void _q_updateSpanRemovedColumns(QModelIndex,int,int))
};

#endif // QT_NO_TABLEVIEW

QT_END_NAMESPACE

#endif // QTABLEVIEW_H
