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

#ifndef QDESIGNER_MENUBAR_H
#define QDESIGNER_MENUBAR_H

#include "shared_global_p.h"

#include <QtWidgets/QAction>
#include <QtWidgets/QMenuBar>

#include <QtCore/QPointer>
#include <QtCore/QMimeData>

QT_BEGIN_NAMESPACE

class QDesignerFormWindowInterface;
class QDesignerActionProviderExtension;

class QLineEdit;
class QMimeData;

namespace qdesigner_internal {
class PromotionTaskMenu;

class SpecialMenuAction: public QAction
{
    Q_OBJECT
public:
    SpecialMenuAction(QObject *parent = 0);
    virtual ~SpecialMenuAction();
};

} // namespace qdesigner_internal

class QDESIGNER_SHARED_EXPORT QDesignerMenuBar: public QMenuBar
{
    Q_OBJECT
public:
    QDesignerMenuBar(QWidget *parent = 0);
    virtual ~QDesignerMenuBar();

    bool eventFilter(QObject *object, QEvent *event);

    QDesignerFormWindowInterface *formWindow() const;
    QDesignerActionProviderExtension *actionProvider();

    void adjustSpecialActions();
    bool interactive(bool i);
    bool dragging() const;

    void moveLeft(bool ctrl = false);
    void moveRight(bool ctrl = false);
    void moveUp();
    void moveDown();

    // Helpers for MenuTaskMenu/MenuBarTaskMenu extensions
    QList<QAction *> contextMenuActions();
    void deleteMenuAction(QAction *action);

private slots:
    void deleteMenu();
    void slotRemoveMenuBar();

protected:
    void actionEvent(QActionEvent *event) Q_DECL_OVERRIDE;
    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dragLeaveEvent(QDragLeaveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void focusOutEvent(QFocusEvent *event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

    bool handleEvent(QWidget *widget, QEvent *event);
    bool handleMouseDoubleClickEvent(QWidget *widget, QMouseEvent *event);
    bool handleMousePressEvent(QWidget *widget, QMouseEvent *event);
    bool handleMouseReleaseEvent(QWidget *widget, QMouseEvent *event);
    bool handleMouseMoveEvent(QWidget *widget, QMouseEvent *event);
    bool handleContextMenuEvent(QWidget *widget, QContextMenuEvent *event);
    bool handleKeyPressEvent(QWidget *widget, QKeyEvent *event);

    void startDrag(const QPoint &pos);

    enum ActionDragCheck { NoActionDrag, ActionDragOnSubMenu, AcceptActionDrag };
    ActionDragCheck checkAction(QAction *action) const;

    void adjustIndicator(const QPoint &pos);
    int findAction(const QPoint &pos) const;

    QAction *currentAction() const;
    int realActionCount() const;

    enum LeaveEditMode {
        Default = 0,
        ForceAccept
    };

    void enterEditMode();
    void leaveEditMode(LeaveEditMode mode);
    void showLineEdit();

    void showMenu(int index = -1);
    void hideMenu(int index = -1);

    QAction *safeActionAt(int index) const;

    bool swapActions(int a, int b);

private:
    void updateCurrentAction(bool selectAction);
    void movePrevious(bool ctrl);
    void moveNext(bool ctrl);

    QAction *m_addMenu;
    QPointer<QMenu> m_activeMenu;
    QPoint m_startPosition;
    int m_currentIndex;
    bool m_interactive;
    QLineEdit *m_editor;
    bool m_dragging;
    int m_lastMenuActionIndex;
    QPointer<QWidget> m_lastFocusWidget;
    qdesigner_internal::PromotionTaskMenu* m_promotionTaskMenu;
};

QT_END_NAMESPACE

#endif // QDESIGNER_MENUBAR_H
