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

#ifndef ACTIONEDITOR_H
#define ACTIONEDITOR_H

#include "shared_global_p.h"
#include <QtDesigner/QDesignerActionEditorInterface>

#include <QtCore/QPointer>

QT_BEGIN_NAMESPACE

class QDesignerPropertyEditorInterface;
class QDesignerSettingsInterface;
class QMenu;
class QActionGroup;
class QSignalMapper;
class QItemSelection;
class QListWidget;
class QPushButton;
class QLineEdit;
class QToolButton;

namespace qdesigner_internal {

class ActionView;
class ResourceMimeData;

class QDESIGNER_SHARED_EXPORT ActionEditor: public QDesignerActionEditorInterface
{
    Q_OBJECT
public:
    explicit ActionEditor(QDesignerFormEditorInterface *core, QWidget *parent = 0, Qt::WindowFlags flags = 0);
    virtual ~ActionEditor();

    QDesignerFormWindowInterface *formWindow() const;
    void setFormWindow(QDesignerFormWindowInterface *formWindow) Q_DECL_OVERRIDE;

    QDesignerFormEditorInterface *core() const Q_DECL_OVERRIDE;

    QAction *actionNew() const;
    QAction *actionDelete() const;

    QString filter() const;

    void manageAction(QAction *action) Q_DECL_OVERRIDE;
    void unmanageAction(QAction *action) Q_DECL_OVERRIDE;

    static QString actionTextToName(const QString &text, const QString &prefix = QLatin1String("action"));

    // Utility to create a configure button with menu for usage on toolbars
    static QToolButton *createConfigureMenuButton(const QString &t, QMenu **ptrToMenu);

public slots:
    void setFilter(const QString &filter);
    void mainContainerChanged();

private slots:
    void slotCurrentItemChanged(QAction *item);
    void slotSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);
    void editAction(QAction *item);
    void editCurrentAction();
    void navigateToSlotCurrentAction();
    void slotActionChanged();
    void slotNewAction();
    void slotDelete();
    void resourceImageDropped(const QString &path, QAction *action);
    void slotContextMenuRequested(QContextMenuEvent *, QAction *);
    void slotViewMode(QAction *a);
    void slotSelectAssociatedWidget(QWidget *w);
#ifndef QT_NO_CLIPBOARD
    void slotCopy();
    void slotCut();
    void slotPaste();
#endif

signals:
    void itemActivated(QAction *item);
    // Context menu for item or global menu if item == 0.
    void contextMenuRequested(QMenu *menu, QAction *item);

private:
    typedef QList<QAction *> ActionList;
    void deleteActions(QDesignerFormWindowInterface *formWindow, const ActionList &);
#ifndef QT_NO_CLIPBOARD
    void copyActions(QDesignerFormWindowInterface *formWindow, const ActionList &);
#endif

    void restoreSettings();
    void saveSettings();

    void updateViewModeActions();

    QDesignerFormEditorInterface *m_core;
    QPointer<QDesignerFormWindowInterface> m_formWindow;
    QListWidget *m_actionGroups;

    ActionView *m_actionView;

    QAction *m_actionNew;
    QAction *m_actionEdit;
    QAction *m_actionNavigateToSlot;
#ifndef QT_NO_CLIPBOARD
    QAction *m_actionCopy;
    QAction *m_actionCut;
    QAction *m_actionPaste;
#endif
    QAction *m_actionSelectAll;
    QAction *m_actionDelete;

    QActionGroup *m_viewModeGroup;
    QAction *m_iconViewAction;
    QAction *m_listViewAction;

    QString m_filter;
    QWidget *m_filterWidget;
    QSignalMapper *m_selectAssociatedWidgetsMapper;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // ACTIONEDITOR_H
