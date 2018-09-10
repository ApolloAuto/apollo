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

#ifndef QCOLUMNVIEW_H
#define QCOLUMNVIEW_H

#include <QtWidgets/qabstractitemview.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_COLUMNVIEW

class QColumnViewPrivate;

class Q_WIDGETS_EXPORT QColumnView : public QAbstractItemView {

Q_OBJECT
    Q_PROPERTY(bool resizeGripsVisible READ resizeGripsVisible WRITE setResizeGripsVisible)

Q_SIGNALS:
    void updatePreviewWidget(const QModelIndex &index);

public:
    explicit QColumnView(QWidget *parent = 0);
    ~QColumnView();

    // QAbstractItemView overloads
    QModelIndex indexAt(const QPoint &point) const Q_DECL_OVERRIDE;
    void scrollTo(const QModelIndex &index, ScrollHint hint = EnsureVisible) Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;
    QRect visualRect(const QModelIndex &index) const Q_DECL_OVERRIDE;
    void setModel(QAbstractItemModel *model) Q_DECL_OVERRIDE;
    void setSelectionModel(QItemSelectionModel * selectionModel) Q_DECL_OVERRIDE;
    void setRootIndex(const QModelIndex &index) Q_DECL_OVERRIDE;
    void selectAll() Q_DECL_OVERRIDE;

    // QColumnView functions
    void setResizeGripsVisible(bool visible);
    bool resizeGripsVisible() const;

    QWidget *previewWidget() const;
    void setPreviewWidget(QWidget *widget);

    void setColumnWidths(const QList<int> &list);
    QList<int> columnWidths() const;

protected:
    QColumnView(QColumnViewPrivate &dd, QWidget *parent = 0);

    // QAbstractItemView overloads
    bool isIndexHidden(const QModelIndex &index) const Q_DECL_OVERRIDE;
    QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;
    void setSelection(const QRect & rect, QItemSelectionModel::SelectionFlags command) Q_DECL_OVERRIDE;
    QRegion visualRegionForSelection(const QItemSelection &selection) const Q_DECL_OVERRIDE;
    int horizontalOffset() const Q_DECL_OVERRIDE;
    int verticalOffset() const Q_DECL_OVERRIDE;
    void rowsInserted(const QModelIndex &parent, int start, int end) Q_DECL_OVERRIDE;
    void currentChanged(const QModelIndex &current, const QModelIndex &previous) Q_DECL_OVERRIDE;

    // QColumnView functions
    void scrollContentsBy(int dx, int dy) Q_DECL_OVERRIDE;
    virtual QAbstractItemView* createColumn(const QModelIndex &rootIndex);
    void initializeColumn(QAbstractItemView *column) const;

private:
    Q_DECLARE_PRIVATE(QColumnView)
    Q_DISABLE_COPY(QColumnView)
    Q_PRIVATE_SLOT(d_func(), void _q_gripMoved(int))
    Q_PRIVATE_SLOT(d_func(), void _q_changeCurrentColumn())
    Q_PRIVATE_SLOT(d_func(), void _q_clicked(const QModelIndex &))
};

#endif // QT_NO_COLUMNVIEW

QT_END_NAMESPACE

#endif // QCOLUMNVIEW_H

