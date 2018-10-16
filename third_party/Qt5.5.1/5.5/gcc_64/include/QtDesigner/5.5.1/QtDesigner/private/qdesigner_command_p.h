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

#ifndef QDESIGNER_COMMAND_H
#define QDESIGNER_COMMAND_H

#include "shared_global_p.h"
#include "shared_enums_p.h"
#include "layoutinfo_p.h"
#include "qdesigner_utils_p.h"
#include "qdesigner_formwindowcommand_p.h"
#include "qdesigner_formeditorcommand_p.h"

#include <QtDesigner/layoutdecoration.h>

#include <QtGui/QIcon>
#include <QtCore/QObject>
#include <QtCore/QPair>
#include <QtCore/QMap>
#include <QtCore/QHash>
#include <QtCore/QPoint>
#include <QtCore/QRect>

QT_BEGIN_NAMESPACE

class QDesignerContainerExtension;
class QDesignerMetaDataBaseItemInterface;
class QDesignerMenu;

class QMenuBar;
class QStatusBar;
class QToolBar;
class QToolBox;
class QTabWidget;
class QTableWidget;
class QTableWidgetItem;
class QTreeWidget;
class QTreeWidgetItem;
class QListWidget;
class QListWidgetItem;
class QComboBox;
class QStackedWidget;
class QDockWidget;
class QMainWindow;
class QFormLayout;

namespace qdesigner_internal {

class Layout;
class LayoutHelper;
class PropertySheetIconValue;
class DesignerIconCache;
struct LayoutProperties;

class QDESIGNER_SHARED_EXPORT InsertWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit InsertWidgetCommand(QDesignerFormWindowInterface *formWindow);
    ~InsertWidgetCommand();

    void init(QWidget *widget, bool already_in_form = false, int layoutRow = -1, int layoutColumn = -1);

    virtual void redo();
    virtual void undo();

private:
    void refreshBuddyLabels();

    QPointer<QWidget> m_widget;
    QDesignerLayoutDecorationExtension::InsertMode m_insertMode;
    QPair<int, int> m_cell;
    LayoutHelper* m_layoutHelper;
    bool m_widgetWasManaged;
};

class QDESIGNER_SHARED_EXPORT ChangeZOrderCommand: public QDesignerFormWindowCommand
{

public:
    explicit ChangeZOrderCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget);

    virtual void redo();
    virtual void undo();
protected:
    virtual QWidgetList reorderWidget(const QWidgetList &list, QWidget *widget) const = 0;
    virtual void reorder(QWidget *widget) const = 0;

private:
    QPointer<QWidget> m_widget;
    QPointer<QWidget> m_oldPreceding;
    QList<QWidget *> m_oldParentZOrder;
};

class QDESIGNER_SHARED_EXPORT RaiseWidgetCommand: public ChangeZOrderCommand
{

public:
    explicit RaiseWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget);

protected:
    QWidgetList reorderWidget(const QWidgetList &list, QWidget *widget) const Q_DECL_OVERRIDE;
    void reorder(QWidget *widget) const Q_DECL_OVERRIDE;
};

class QDESIGNER_SHARED_EXPORT LowerWidgetCommand: public ChangeZOrderCommand
{

public:
    explicit LowerWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget);

protected:
    QWidgetList reorderWidget(const QWidgetList &list, QWidget *widget) const Q_DECL_OVERRIDE;
    void reorder(QWidget *widget) const Q_DECL_OVERRIDE;
};

class QDESIGNER_SHARED_EXPORT AdjustWidgetSizeCommand: public QDesignerFormWindowCommand
{

public:
    explicit AdjustWidgetSizeCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget);

    virtual void redo();
    virtual void undo();

private:
    QWidget *widgetForAdjust() const;
    bool adjustNonLaidOutMainContainer(QWidget *integrationContainer);
    void updatePropertyEditor() const;

    QPointer<QWidget> m_widget;
    QRect m_geometry;
};

// Helper to correctly unmanage a widget and its children for delete operations
class  QDESIGNER_SHARED_EXPORT ManageWidgetCommandHelper {
public:
    typedef QVector<QWidget*> WidgetVector;

    ManageWidgetCommandHelper();
    void init(const QDesignerFormWindowInterface *fw, QWidget *widget);
    void init(QWidget *widget, const WidgetVector &managedChildren);

    void manage(QDesignerFormWindowInterface *fw);
    void unmanage(QDesignerFormWindowInterface *fw);

    const WidgetVector &managedChildren() const { return m_managedChildren; }
private:
    QWidget *m_widget;
    WidgetVector m_managedChildren;
};

class QDESIGNER_SHARED_EXPORT DeleteWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit DeleteWidgetCommand(QDesignerFormWindowInterface *formWindow);
    ~DeleteWidgetCommand();

    enum DeleteFlags { DoNotUnmanage = 0x1, DoNotSimplifyLayout = 0x2 };

    void init(QWidget *widget, unsigned flags = 0);

    virtual void redo();
    virtual void undo();

private:
    QPointer<QWidget> m_widget;
    QPointer<QWidget> m_parentWidget;
    QRect m_geometry;
    LayoutInfo::Type m_layoutType;
    LayoutHelper* m_layoutHelper;
    unsigned m_flags;
    QRect m_layoutPosition;
    int m_splitterIndex;
    bool m_layoutSimplified;
    QDesignerMetaDataBaseItemInterface *m_formItem;
    int m_tabOrderIndex;
    int m_widgetOrderIndex;
    int m_zOrderIndex;
    ManageWidgetCommandHelper m_manageHelper;
};

class QDESIGNER_SHARED_EXPORT ReparentWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit ReparentWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget, QWidget *parentWidget);

    virtual void redo();
    virtual void undo();

private:
    QPointer<QWidget> m_widget;
    QPoint m_oldPos;
    QPoint m_newPos;
    QPointer<QWidget> m_oldParentWidget;
    QPointer<QWidget> m_newParentWidget;
    QList<QWidget *> m_oldParentList;
    QList<QWidget *> m_oldParentZOrder;
};

class QDESIGNER_SHARED_EXPORT ChangeFormLayoutItemRoleCommand : public QDesignerFormWindowCommand
{
public:
    enum Operation { SpanningToLabel = 0x1, SpanningToField = 0x2, LabelToSpanning = 0x4, FieldToSpanning =0x8 };

    explicit ChangeFormLayoutItemRoleCommand(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget, Operation op);

    virtual void redo();
    virtual void undo();

    // Return a mask of possible operations of that item
    static unsigned possibleOperations(QDesignerFormEditorInterface *core, QWidget *w);

private:
    static QFormLayout *managedFormLayoutOf(QDesignerFormEditorInterface *core, QWidget *w);
    static Operation reverseOperation(Operation op);
    void doOperation(Operation op);

    QPointer<QWidget> m_widget;
    Operation m_operation;
};

class QDESIGNER_SHARED_EXPORT ChangeLayoutItemGeometry: public QDesignerFormWindowCommand
{

public:
    explicit ChangeLayoutItemGeometry(QDesignerFormWindowInterface *formWindow);

    void init(QWidget *widget, int row, int column, int rowspan, int colspan);

    virtual void redo();
    virtual void undo();

protected:
    void changeItemPosition(const QRect &g);

private:
    QPointer<QWidget> m_widget;
    QRect m_oldInfo;
    QRect m_newInfo;
};

class QDESIGNER_SHARED_EXPORT TabOrderCommand: public QDesignerFormWindowCommand
{

public:
    explicit TabOrderCommand(QDesignerFormWindowInterface *formWindow);

    void init(const QList<QWidget*> &newTabOrder);

    inline QList<QWidget*> oldTabOrder() const
    { return m_oldTabOrder; }

    inline QList<QWidget*> newTabOrder() const
    { return m_newTabOrder; }

    virtual void redo();
    virtual void undo();

private:
    QDesignerMetaDataBaseItemInterface *m_widgetItem;
    QList<QWidget*> m_oldTabOrder;
    QList<QWidget*> m_newTabOrder;
};

class QDESIGNER_SHARED_EXPORT PromoteToCustomWidgetCommand : public QDesignerFormWindowCommand
{
public:
    typedef QList<QPointer<QWidget> > WidgetList;

    explicit PromoteToCustomWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(const WidgetList &widgets, const QString &customClassName);
    virtual void redo();
    virtual void undo();

private:
    void updateSelection();
    WidgetList m_widgets;
    QString m_customClassName;
};

class QDESIGNER_SHARED_EXPORT DemoteFromCustomWidgetCommand : public QDesignerFormWindowCommand
{
public:
    typedef PromoteToCustomWidgetCommand::WidgetList WidgetList;

    explicit DemoteFromCustomWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(const WidgetList &promoted);
    virtual void redo();
    virtual void undo();
private:
    PromoteToCustomWidgetCommand m_promote_cmd;
};

// Mixin class for storing the selection state
class QDESIGNER_SHARED_EXPORT CursorSelectionState {
    Q_DISABLE_COPY(CursorSelectionState)
public:
    CursorSelectionState();

    void save(const QDesignerFormWindowInterface *formWindow);
    void restore(QDesignerFormWindowInterface *formWindow) const;

private:
    typedef QList<QPointer<QWidget> > WidgetPointerList;
    WidgetPointerList m_selection;
    QPointer<QWidget> m_current;
};

class QDESIGNER_SHARED_EXPORT LayoutCommand: public QDesignerFormWindowCommand
{

public:
    explicit LayoutCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~LayoutCommand();

    inline QWidgetList widgets() const { return m_widgets; }

    void init(QWidget *parentWidget, const QWidgetList &widgets, LayoutInfo::Type layoutType,
              QWidget *layoutBase = 0,
              // Reparent/Hide instances of QLayoutWidget.
              bool reparentLayoutWidget = true);

    virtual void redo();
    virtual void undo();

private:
    QPointer<QWidget> m_parentWidget;
    QWidgetList m_widgets;
    QPointer<QWidget> m_layoutBase;
    QPointer<Layout> m_layout;
    CursorSelectionState m_cursorSelectionState;
    bool m_setup;
};

class QDESIGNER_SHARED_EXPORT BreakLayoutCommand: public QDesignerFormWindowCommand
{

public:
    explicit BreakLayoutCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~BreakLayoutCommand();

    inline QWidgetList widgets() const { return m_widgets; }

    void init(const QWidgetList &widgets, QWidget *layoutBase,
              // Reparent/Hide instances of QLayoutWidget.
              bool reparentLayoutWidget = true);

    virtual void redo();
    virtual void undo();

    // Access the properties of the layout, 0 in case of splitters.
    const LayoutProperties *layoutProperties() const;
    int propertyMask() const;

private:
    QWidgetList m_widgets;
    QPointer<QWidget> m_layoutBase;
    QPointer<Layout> m_layout;
    LayoutHelper* m_layoutHelper;
    LayoutProperties *m_properties;
    int m_propertyMask;
    CursorSelectionState m_cursorSelectionState;
};

class QDESIGNER_SHARED_EXPORT SimplifyLayoutCommand: public QDesignerFormWindowCommand
{
public:
    explicit SimplifyLayoutCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~SimplifyLayoutCommand();

    bool init(QWidget *layoutBase);

    // Quick check
    static bool canSimplify(QDesignerFormEditorInterface *core, const QWidget *w, int *layoutType = 0);

    virtual void redo();
    virtual void undo();

private:
    const QRect m_area;
    QWidget *m_layoutBase;
    LayoutHelper* m_layoutHelper;
    bool m_layoutSimplified;
};

class QDESIGNER_SHARED_EXPORT ToolBoxCommand: public QDesignerFormWindowCommand
{

public:
    explicit ToolBoxCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~ToolBoxCommand();

    void init(QToolBox *toolBox);

    virtual void removePage();
    virtual void addPage();

protected:
    QPointer<QToolBox> m_toolBox;
    QPointer<QWidget> m_widget;
    int m_index;
    QString m_itemText;
    QIcon m_itemIcon;
};

class QDESIGNER_SHARED_EXPORT MoveToolBoxPageCommand: public ToolBoxCommand
{

public:
    explicit MoveToolBoxPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~MoveToolBoxPageCommand();

    void init(QToolBox *toolBox, QWidget *page, int newIndex);

    virtual void redo();
    virtual void undo();

private:
    int m_newIndex;
    int m_oldIndex;
};

class QDESIGNER_SHARED_EXPORT DeleteToolBoxPageCommand: public ToolBoxCommand
{

public:
    explicit DeleteToolBoxPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~DeleteToolBoxPageCommand();

    void init(QToolBox *toolBox);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT AddToolBoxPageCommand: public ToolBoxCommand
{

public:
    enum InsertionMode {
        InsertBefore,
        InsertAfter
    };
    explicit AddToolBoxPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~AddToolBoxPageCommand();

    void init(QToolBox *toolBox);
    void init(QToolBox *toolBox, InsertionMode mode);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT TabWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit TabWidgetCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~TabWidgetCommand();

    void init(QTabWidget *tabWidget);

    virtual void removePage();
    virtual void addPage();

protected:
    QPointer<QTabWidget> m_tabWidget;
    QPointer<QWidget> m_widget;
    int m_index;
    QString m_itemText;
    QIcon m_itemIcon;
};

class QDESIGNER_SHARED_EXPORT DeleteTabPageCommand: public TabWidgetCommand
{

public:
    explicit DeleteTabPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~DeleteTabPageCommand();

    void init(QTabWidget *tabWidget);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT AddTabPageCommand: public TabWidgetCommand
{

public:
    enum InsertionMode {
        InsertBefore,
        InsertAfter
    };
    explicit AddTabPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~AddTabPageCommand();

    void init(QTabWidget *tabWidget);
    void init(QTabWidget *tabWidget, InsertionMode mode);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT MoveTabPageCommand: public TabWidgetCommand
{

public:
    explicit MoveTabPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~MoveTabPageCommand();

    void init(QTabWidget *tabWidget, QWidget *page,
                      const QIcon &icon, const QString &label,
                      int index, int newIndex);

    virtual void redo();
    virtual void undo();

private:
    int m_newIndex;
    int m_oldIndex;
    QPointer<QWidget> m_page;
    QString m_label;
    QIcon m_icon;
};

class QDESIGNER_SHARED_EXPORT StackedWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit StackedWidgetCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~StackedWidgetCommand();

    void init(QStackedWidget *stackedWidget);

    virtual void removePage();
    virtual void addPage();

protected:
    QPointer<QStackedWidget> m_stackedWidget;
    QPointer<QWidget> m_widget;
    int m_index;
};

class QDESIGNER_SHARED_EXPORT MoveStackedWidgetCommand: public StackedWidgetCommand
{

public:
    explicit MoveStackedWidgetCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~MoveStackedWidgetCommand();

    void init(QStackedWidget *stackedWidget, QWidget *page, int newIndex);

    virtual void redo();
    virtual void undo();

private:
    int m_newIndex;
    int m_oldIndex;
};

class QDESIGNER_SHARED_EXPORT DeleteStackedWidgetPageCommand: public StackedWidgetCommand
{

public:
    explicit DeleteStackedWidgetPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~DeleteStackedWidgetPageCommand();

    void init(QStackedWidget *stackedWidget);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT AddStackedWidgetPageCommand: public StackedWidgetCommand
{

public:
    enum InsertionMode {
        InsertBefore,
        InsertAfter
    };
    explicit AddStackedWidgetPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~AddStackedWidgetPageCommand();

    void init(QStackedWidget *stackedWidget);
    void init(QStackedWidget *stackedWidget, InsertionMode mode);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT CreateMenuBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit CreateMenuBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QMainWindow *mainWindow);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QMenuBar> m_menuBar;
};

class QDESIGNER_SHARED_EXPORT DeleteMenuBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit DeleteMenuBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QMenuBar *menuBar);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QMenuBar> m_menuBar;
};

class QDESIGNER_SHARED_EXPORT CreateStatusBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit CreateStatusBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QMainWindow *mainWindow);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QStatusBar> m_statusBar;
};

class QDESIGNER_SHARED_EXPORT DeleteStatusBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit DeleteStatusBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QStatusBar *statusBar);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QStatusBar> m_statusBar;
};

class QDESIGNER_SHARED_EXPORT AddToolBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit AddToolBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QMainWindow *mainWindow);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QToolBar> m_toolBar;
};

class QDESIGNER_SHARED_EXPORT DeleteToolBarCommand: public QDesignerFormWindowCommand
{

public:
    explicit DeleteToolBarCommand(QDesignerFormWindowInterface *formWindow);

    void init(QToolBar *toolBar);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QToolBar> m_toolBar;
};

class QDESIGNER_SHARED_EXPORT DockWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit DockWidgetCommand(const QString &description, QDesignerFormWindowInterface *formWindow);
    virtual ~DockWidgetCommand();

    void init(QDockWidget *dockWidget);

protected:
    QPointer<QDockWidget> m_dockWidget;
};

class QDESIGNER_SHARED_EXPORT AddDockWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit AddDockWidgetCommand(QDesignerFormWindowInterface *formWindow);

    void init(QMainWindow *mainWindow, QDockWidget *dockWidget);
    void init(QMainWindow *mainWindow);

    virtual void undo();
    virtual void redo();

private:
    QPointer<QMainWindow> m_mainWindow;
    QPointer<QDockWidget> m_dockWidget;
};

class QDESIGNER_SHARED_EXPORT ContainerWidgetCommand: public QDesignerFormWindowCommand
{

public:
    explicit ContainerWidgetCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~ContainerWidgetCommand();

    QDesignerContainerExtension *containerExtension() const;

    void init(QWidget *containerWidget);

    virtual void removePage();
    virtual void addPage();

protected:
    QPointer<QWidget> m_containerWidget;
    QPointer<QWidget> m_widget;
    int m_index;
};

class QDESIGNER_SHARED_EXPORT DeleteContainerWidgetPageCommand: public ContainerWidgetCommand
{

public:
    explicit DeleteContainerWidgetPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~DeleteContainerWidgetPageCommand();

    void init(QWidget *containerWidget, ContainerType ct);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT AddContainerWidgetPageCommand: public ContainerWidgetCommand
{

public:
    enum InsertionMode {
        InsertBefore,
        InsertAfter
    };
    explicit AddContainerWidgetPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~AddContainerWidgetPageCommand();

    void init(QWidget *containerWidget, ContainerType ct, InsertionMode mode);

    virtual void redo();
    virtual void undo();
};

class QDESIGNER_SHARED_EXPORT ChangeCurrentPageCommand: public QDesignerFormWindowCommand
{

public:
    explicit ChangeCurrentPageCommand(QDesignerFormWindowInterface *formWindow);
    virtual ~ChangeCurrentPageCommand();

    QDesignerContainerExtension *containerExtension() const;

    void init(QWidget *containerWidget, int newIndex);

    virtual void redo();
    virtual void undo();

protected:
    QPointer<QWidget> m_containerWidget;
    QPointer<QWidget> m_widget;
    int m_oldIndex;
    int m_newIndex;
};

struct QDESIGNER_SHARED_EXPORT ItemData {
    ItemData() {}

    ItemData(const QListWidgetItem *item, bool editor);
    ItemData(const QTableWidgetItem *item, bool editor);
    ItemData(const QTreeWidgetItem *item, int column);
    QListWidgetItem *createListItem(DesignerIconCache *iconCache, bool editor) const;
    QTableWidgetItem *createTableItem(DesignerIconCache *iconCache, bool editor) const;
    void fillTreeItemColumn(QTreeWidgetItem *item, int column, DesignerIconCache *iconCache) const;

    bool isValid() const { return !m_properties.isEmpty(); }
    bool operator==(const ItemData &rhs) const { return m_properties == rhs.m_properties; }
    bool operator!=(const ItemData &rhs) const { return m_properties != rhs.m_properties; }

    QHash<int, QVariant> m_properties;
};

struct QDESIGNER_SHARED_EXPORT ListContents {
    ListContents() {}

    ListContents(const QTreeWidgetItem *item);
    QTreeWidgetItem *createTreeItem(DesignerIconCache *iconCache) const;

    void createFromListWidget(const QListWidget *listWidget, bool editor);
    void applyToListWidget(QListWidget *listWidget, DesignerIconCache *iconCache, bool editor) const;
    void createFromComboBox(const QComboBox *listWidget);
    void applyToComboBox(QComboBox *listWidget, DesignerIconCache *iconCache) const;

    bool operator==(const ListContents &rhs) const { return m_items == rhs.m_items; }
    bool operator!=(const ListContents &rhs) const { return m_items != rhs.m_items; }

    QList<ItemData> m_items;
};

// Data structure representing the contents of a QTableWidget with
// methods to retrieve and apply for ChangeTableContentsCommand
struct QDESIGNER_SHARED_EXPORT TableWidgetContents {

    typedef QPair<int, int> CellRowColumnAddress;
    typedef QMap<CellRowColumnAddress, ItemData> TableItemMap;

    TableWidgetContents();
    void clear();

    void fromTableWidget(const QTableWidget *tableWidget, bool editor);
    void applyToTableWidget(QTableWidget *tableWidget, DesignerIconCache *iconCache, bool editor) const;

    bool operator==(const TableWidgetContents &rhs) const;
    bool operator!=(const TableWidgetContents &rhs) const { return !(*this == rhs); }

    static bool nonEmpty(const QTableWidgetItem *item, int headerColumn);
    static QString defaultHeaderText(int i);
    static void insertHeaderItem(const QTableWidgetItem *item, int i, ListContents *header, bool editor);

    int m_columnCount;
    int m_rowCount;
    ListContents m_horizontalHeader;
    ListContents m_verticalHeader;
    TableItemMap m_items;
};

class QDESIGNER_SHARED_EXPORT ChangeTableContentsCommand: public QDesignerFormWindowCommand
{
public:
    explicit ChangeTableContentsCommand(QDesignerFormWindowInterface *formWindow);

    void init(QTableWidget *tableWidget, const TableWidgetContents &oldCont, const TableWidgetContents &newCont);
    virtual void redo();
    virtual void undo();

private:
    QPointer<QTableWidget> m_tableWidget;
    TableWidgetContents m_oldContents;
    TableWidgetContents m_newContents;
    DesignerIconCache *m_iconCache;
};

// Data structure representing the contents of a QTreeWidget with
// methods to retrieve and apply for ChangeTreeContentsCommand
struct QDESIGNER_SHARED_EXPORT TreeWidgetContents {

    struct ItemContents : public ListContents {
        ItemContents() : m_itemFlags(-1) {}
        ItemContents(const QTreeWidgetItem *item, bool editor);
        QTreeWidgetItem *createTreeItem(DesignerIconCache *iconCache, bool editor) const;

        bool operator==(const ItemContents &rhs) const;
        bool operator!=(const ItemContents &rhs) const { return !(*this == rhs); }

        int m_itemFlags;
        //bool m_firstColumnSpanned:1;
        //bool m_hidden:1;
        //bool m_expanded:1;
        QList<ItemContents> m_children;
    };

    void clear();

    void fromTreeWidget(const QTreeWidget *treeWidget, bool editor);
    void applyToTreeWidget(QTreeWidget *treeWidget, DesignerIconCache *iconCache, bool editor) const;

    bool operator==(const TreeWidgetContents &rhs) const;
    bool operator!=(const TreeWidgetContents &rhs) const { return !(*this == rhs); }

    ListContents m_headerItem;
    QList<ItemContents> m_rootItems;
};

class QDESIGNER_SHARED_EXPORT ChangeTreeContentsCommand: public QDesignerFormWindowCommand
{

public:
    explicit ChangeTreeContentsCommand(QDesignerFormWindowInterface *formWindow);

    void init(QTreeWidget *treeWidget, const TreeWidgetContents &oldState, const TreeWidgetContents &newState);
    virtual void redo();
    virtual void undo();
    enum ApplyIconStrategy {
        SetIconStrategy,
        ResetIconStrategy
    };
private:
    QPointer<QTreeWidget> m_treeWidget;
    TreeWidgetContents m_oldState;
    TreeWidgetContents m_newState;
    DesignerIconCache *m_iconCache;
};

class QDESIGNER_SHARED_EXPORT ChangeListContentsCommand: public QDesignerFormWindowCommand
{

public:
    explicit ChangeListContentsCommand(QDesignerFormWindowInterface *formWindow);

    void init(QListWidget *listWidget, const ListContents &oldItems, const ListContents &items);
    void init(QComboBox *comboBox, const ListContents &oldItems, const ListContents &items);
    virtual void redo();
    virtual void undo();
private:
    QPointer<QListWidget> m_listWidget;
    QPointer<QComboBox> m_comboBox;
    ListContents m_oldItemsState;
    ListContents m_newItemsState;
    DesignerIconCache *m_iconCache;
};

class QDESIGNER_SHARED_EXPORT AddActionCommand : public QDesignerFormWindowCommand
{

public:
    explicit AddActionCommand(QDesignerFormWindowInterface *formWindow);
    void init(QAction *action);
    virtual void redo();
    virtual void undo();
private:
    QAction *m_action;
};

// Note: This command must be executed within a macro since it
// makes the form emit objectRemoved() which might cause other components
// to add commands (for example, removal of signals and slots
class QDESIGNER_SHARED_EXPORT RemoveActionCommand : public QDesignerFormWindowCommand
{

public:
    explicit RemoveActionCommand(QDesignerFormWindowInterface *formWindow);
    void init(QAction *action);
    virtual void redo();
    virtual void undo();

    struct ActionDataItem {
        ActionDataItem(QAction *_before = 0, QWidget *_widget = 0)
            : before(_before), widget(_widget) {}
        QAction *before;
        QWidget *widget;
    };
    typedef QList<ActionDataItem> ActionData;

private:
    QAction *m_action;

    ActionData m_actionData;
};

class QDESIGNER_SHARED_EXPORT ActionInsertionCommand : public QDesignerFormWindowCommand
{

protected:
    ActionInsertionCommand(const QString &text, QDesignerFormWindowInterface *formWindow);

public:
    void init(QWidget *parentWidget, QAction *action, QAction *beforeAction = 0, bool update = true);

protected:
    void insertAction();
    void removeAction();

private:
    QWidget *m_parentWidget;
    QAction *m_action;
    QAction *m_beforeAction;
    bool m_update;
};

class QDESIGNER_SHARED_EXPORT InsertActionIntoCommand : public ActionInsertionCommand
{

public:
    explicit InsertActionIntoCommand(QDesignerFormWindowInterface *formWindow);

    virtual void redo() {  insertAction(); }
    virtual void undo() {  removeAction(); }
};

class QDESIGNER_SHARED_EXPORT RemoveActionFromCommand : public ActionInsertionCommand
{

public:
    explicit RemoveActionFromCommand(QDesignerFormWindowInterface *formWindow);

    virtual void redo()  {  removeAction(); }
    virtual void undo()  {  insertAction(); }
};

class QDESIGNER_SHARED_EXPORT MenuActionCommand : public QDesignerFormWindowCommand
{
public:
    void init(QAction *action, QAction *actionBefore, QWidget *associatedWidget, QWidget *objectToSelect);

protected:
    MenuActionCommand(const QString &text, QDesignerFormWindowInterface *formWindow);
    void insertMenu();
    void removeMenu();

private:
    QAction *m_action;
    QAction *m_actionBefore;
    QWidget *m_menuParent;
    QWidget *m_associatedWidget;
    QWidget *m_objectToSelect;
};

class QDESIGNER_SHARED_EXPORT AddMenuActionCommand : public MenuActionCommand
{

public:
    explicit AddMenuActionCommand(QDesignerFormWindowInterface *formWindow);

    virtual void redo() { insertMenu(); }
    virtual void undo() { removeMenu(); }
};

class QDESIGNER_SHARED_EXPORT RemoveMenuActionCommand : public MenuActionCommand
{

public:
    explicit RemoveMenuActionCommand(QDesignerFormWindowInterface *formWindow);

    virtual void redo() { removeMenu(); }
    virtual void undo() { insertMenu(); }
};

class QDESIGNER_SHARED_EXPORT CreateSubmenuCommand : public QDesignerFormWindowCommand
{

public:
    explicit CreateSubmenuCommand(QDesignerFormWindowInterface *formWindow);
    void init(QDesignerMenu *menu, QAction *action, QObject *m_objectToSelect = 0);
    virtual void redo();
    virtual void undo();
private:
    QAction *m_action;
    QDesignerMenu *m_menu;
    QObject *m_objectToSelect;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // QDESIGNER_COMMAND_H
