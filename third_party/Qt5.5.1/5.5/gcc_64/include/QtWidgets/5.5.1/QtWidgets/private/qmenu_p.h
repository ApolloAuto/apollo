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

#ifndef QMENU_P_H
#define QMENU_P_H

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

#include "QtWidgets/qmenubar.h"
#include "QtWidgets/qstyleoption.h"
#include "QtCore/qdatetime.h"
#include "QtCore/qmap.h"
#include "QtCore/qhash.h"
#include "QtCore/qbasictimer.h"
#include "private/qwidget_p.h"

#include <qpa/qplatformmenu.h>

QT_BEGIN_NAMESPACE

#ifndef QT_NO_MENU

class QTornOffMenu;
class QEventLoop;

#ifdef Q_OS_WINCE
struct QWceMenuAction {
    uint command;
    QPointer<QAction> action;
    HMENU menuHandle;
    QWceMenuAction() : menuHandle(0), command(0) {}
};
#endif

template <typename T>
class QSetValueOnDestroy
{
public:
    QSetValueOnDestroy(T &toSet, T value)
        : toSet(toSet)
        , value(value)
    { }

    ~QSetValueOnDestroy() { toSet = value; }
private:
    T &toSet;
    T value;
};

class QMenuSloppyState
{
    Q_DISABLE_COPY(QMenuSloppyState)
public:
    QMenuSloppyState()
        : m_menu(Q_NULLPTR)
        , m_enabled(false)
        , m_uni_directional(false)
        , m_select_other_actions(false)
        , m_first_mouse(true)
        , m_init_guard(false)
        , m_uni_dir_discarded_count(0)
        , m_uni_dir_fail_at_count(0)
        , m_timeout(0)
        , m_reset_action(Q_NULLPTR)
        , m_origin_action(Q_NULLPTR)
        , m_parent(Q_NULLPTR)
    { }

    ~QMenuSloppyState() { reset(); }

    void initialize(QMenu *menu)
    {
        m_menu = menu;
        m_uni_directional = menu->style()->styleHint(QStyle::SH_Menu_SubMenuUniDirection, 0, menu);
        m_uni_dir_fail_at_count = menu->style()->styleHint(QStyle::SH_Menu_SubMenuUniDirectionFailCount, 0, menu);
        m_select_other_actions = menu->style()->styleHint(QStyle::SH_Menu_SubMenuSloppySelectOtherActions, 0 , menu);
        m_timeout = menu->style()->styleHint(QStyle::SH_Menu_SubMenuSloppyCloseTimeout);
        m_discard_state_when_entering_parent = menu->style()->styleHint(QStyle::SH_Menu_SubMenuResetWhenReenteringParent);
        m_dont_start_time_on_leave = menu->style()->styleHint(QStyle::SH_Menu_SubMenuDontStartSloppyOnLeave);
        reset();
    }

    void reset();
    bool enabled() const { return m_enabled; }

    void setResetAction(QAction *action) { m_reset_action = action; }

    enum MouseEventResult {
        EventIsProcessed,
        EventShouldBePropogated,
        EventDiscardsSloppyState
    };

    void startTimer()
    {
        if (m_enabled)
            m_time.start(m_timeout, m_menu);
    }

    void startTimerIfNotRunning()
    {
        if (!m_time.isActive())
            startTimer();
    }

    void stopTimer()
    {
        m_time.stop();
    }

    void enter();

    void childEnter()
    {
        stopTimer();
        if (m_parent)
            m_parent->childEnter();
    }

    void leave()
    {
        if (m_dont_start_time_on_leave)
            return;
        if (m_parent)
            m_parent->childLeave();
        startTimer();
    }
    void childLeave();

    static float slope(const QPointF &p1, const QPointF &p2)
    {
        const QPointF slope = p2 - p1;
        if (slope.x()== 0)
            return 9999;
        return slope.y()/slope.x();
    }

    bool checkSlope(qreal oldS, qreal newS, bool wantSteeper)
    {
        if (wantSteeper)
            return oldS <= newS;
        return newS <= oldS;
    }

    MouseEventResult processMouseEvent(const QPointF &mousePos, QAction *resetAction, QAction *currentAction)
    {
        if (m_parent)
            m_parent->stopTimer();

        if (!m_enabled)
            return EventShouldBePropogated;

        if (!m_time.isActive())
            startTimer();

        if (!m_sub_menu) {
            reset();
            return EventShouldBePropogated;
        }

        QSetValueOnDestroy<bool> setFirstMouse(m_first_mouse, false);
        QSetValueOnDestroy<QPointF> setPreviousPoint(m_previous_point, mousePos);

        if (resetAction && resetAction->isSeparator())
            m_reset_action = Q_NULLPTR;
        else {
            m_reset_action = resetAction;
        }

        if (m_action_rect.contains(mousePos)) {
            startTimer();
            return currentAction == m_menu->menuAction() ? EventIsProcessed : EventShouldBePropogated;
        }

        if (m_uni_directional && !m_first_mouse && resetAction != m_origin_action) {
            bool left_to_right = m_menu->layoutDirection() == Qt::LeftToRight;
            QRect sub_menu_rect = m_sub_menu->geometry();
            QPoint sub_menu_top =
                    left_to_right? sub_menu_rect.topLeft() : sub_menu_rect.topRight();
            QPoint sub_menu_bottom =
                    left_to_right? sub_menu_rect.bottomLeft() : sub_menu_rect.bottomRight();
            qreal prev_slope_top = slope(m_previous_point, sub_menu_top);
            qreal prev_slope_bottom = slope(m_previous_point, sub_menu_bottom);

            qreal current_slope_top = slope(mousePos, sub_menu_top);
            qreal current_slope_bottom = slope(mousePos, sub_menu_bottom);

            bool slopeTop = checkSlope(prev_slope_top, current_slope_top, sub_menu_top.y() < mousePos.y());
            bool slopeBottom = checkSlope(prev_slope_bottom, current_slope_bottom, sub_menu_bottom.y() > mousePos.y());
            bool rightDirection = false;
            int mouseDir = m_previous_point.y() - mousePos.y();
            if (mouseDir >= 0) {
                rightDirection = rightDirection || slopeTop;
            }
            if (mouseDir <= 0) {
                rightDirection = rightDirection || slopeBottom;
            }

            if (m_uni_dir_discarded_count >= m_uni_dir_fail_at_count && !rightDirection) {
                m_uni_dir_discarded_count = 0;
                return EventDiscardsSloppyState;
            }

            if (!rightDirection)
                m_uni_dir_discarded_count++;
            else
                m_uni_dir_discarded_count = 0;

        }

        return m_select_other_actions ? EventShouldBePropogated : EventIsProcessed;
    }

    void setSubMenuPopup(const QRect &actionRect, QAction *resetAction, QMenu *subMenu);
    bool hasParentActiveDelayTimer() const;
    void timeout();
    int timeForTimeout() const { return m_timeout; }

    bool isTimerId(int timerId) const { return m_time.timerId() == timerId; }
    QMenu *subMenu() const { return m_sub_menu; }

private:
    QMenu *m_menu;
    bool m_enabled;
    bool m_uni_directional;
    bool m_select_other_actions;
    bool m_first_mouse;
    bool m_init_guard;
    bool m_discard_state_when_entering_parent;
    bool m_dont_start_time_on_leave;
    short m_uni_dir_discarded_count;
    short m_uni_dir_fail_at_count;
    short m_timeout;
    QBasicTimer m_time;
    QAction *m_reset_action;
    QAction *m_origin_action;
    QRectF m_action_rect;
    QPointF m_previous_point;
    QPointer<QMenu> m_sub_menu;
    QMenuSloppyState *m_parent;
};

class QMenuPrivate : public QWidgetPrivate
{
    Q_DECLARE_PUBLIC(QMenu)
public:
    QMenuPrivate() : itemsDirty(0), maxIconWidth(0), tabWidth(0), ncols(0),
                      collapsibleSeparators(true), toolTipsVisible(false),
                      activationRecursionGuard(false), delayedPopupGuard(false),
                      hasReceievedEnter(false),
                      hasHadMouse(0), aboutToHide(0), motions(0),
                      currentAction(0),
#ifdef QT_KEYPAD_NAVIGATION
                      selectAction(0),
                      cancelAction(0),
#endif
                      scroll(0), eventLoop(0), tearoff(0), tornoff(0), tearoffHighlighted(0),
                      hasCheckableItems(0), doChildEffects(false), platformMenu(0)

#if defined(Q_OS_WINCE) && !defined(QT_NO_MENUBAR)
                      ,wce_menu(0)
#endif
    { }
    ~QMenuPrivate()
    {
        delete scroll;
        if (!platformMenu.isNull() && !platformMenu->parent())
            delete platformMenu.data();
#if defined(Q_OS_WINCE) && !defined(QT_NO_MENUBAR)
        delete wce_menu;
#endif
    }
    void init();
    void setPlatformMenu(QPlatformMenu *menu);
    void syncPlatformMenu();
#ifdef Q_OS_OSX
    void moveWidgetToPlatformItem(QWidget *w, QPlatformMenuItem* item);
#endif

    static QMenuPrivate *get(QMenu *m) { return m->d_func(); }
    int scrollerHeight() const;

    //item calculations
    mutable uint itemsDirty : 1;
    mutable uint maxIconWidth, tabWidth;
    QRect actionRect(QAction *) const;

    mutable QVector<QRect> actionRects;
    mutable QHash<QAction *, QWidget *> widgetItems;
    void updateActionRects() const;
    void updateActionRects(const QRect &screen) const;
    QRect popupGeometry(const QWidget *widget) const;
    QRect popupGeometry(int screen = -1) const;
    mutable uint ncols : 4; //4 bits is probably plenty
    uint collapsibleSeparators : 1;
    uint toolTipsVisible : 1;
    QSize adjustMenuSizeForScreen(const QRect & screen);
    int getLastVisibleAction() const;

    bool activationRecursionGuard;
    bool delayedPopupGuard;
    bool hasReceievedEnter;

    //selection
    static QMenu *mouseDown;
    QPoint mousePopupPos;
    uint hasHadMouse : 1;
    uint aboutToHide : 1;
    int motions;
    int mousePopupDelay;
    QAction *currentAction;
#ifdef QT_KEYPAD_NAVIGATION
    QAction *selectAction;
    QAction *cancelAction;
#endif
    struct DelayState {
        DelayState()
            : parent(0)
            , action(0)
        { }
        void initialize(QMenu *parent)
        {
            this->parent = parent;
        }

        void start(int timeout, QAction *toStartAction)
        {
            if (timer.isActive() && toStartAction == action)
                return;
            action = toStartAction;
            timer.start(timeout,parent);
        }
        void stop()
        {
            action = 0;
            timer.stop();
        }

        QMenu *parent;
        QBasicTimer timer;
        QAction *action;
    } delayState;
    enum SelectionReason {
        SelectedFromKeyboard,
        SelectedFromElsewhere
    };
    QWidget *topCausedWidget() const;
    QAction *actionAt(QPoint p) const;
    void setFirstActionActive();
    void setCurrentAction(QAction *, int popup = -1, SelectionReason reason = SelectedFromElsewhere, bool activateFirst = false);
    void popupAction(QAction *, int, bool);
    void setSyncAction();

    //scrolling support
    struct QMenuScroller {
        enum ScrollLocation { ScrollStay, ScrollBottom, ScrollTop, ScrollCenter };
        enum ScrollDirection { ScrollNone=0, ScrollUp=0x01, ScrollDown=0x02 };
        uint scrollFlags : 2, scrollDirection : 2;
        int scrollOffset;
        QBasicTimer scrollTimer;

        QMenuScroller() : scrollFlags(ScrollNone), scrollDirection(ScrollNone), scrollOffset(0) { }
        ~QMenuScroller() { }
    } *scroll;
    void scrollMenu(QMenuScroller::ScrollLocation location, bool active=false);
    void scrollMenu(QMenuScroller::ScrollDirection direction, bool page=false, bool active=false);
    void scrollMenu(QAction *action, QMenuScroller::ScrollLocation location, bool active=false);

    //synchronous operation (ie exec())
    QEventLoop *eventLoop;
    QPointer<QAction> syncAction;

    //search buffer
    QString searchBuffer;
    QBasicTimer searchBufferTimer;

    //passing of mouse events up the parent hierarchy
    QPointer<QMenu> activeMenu;
    bool mouseEventTaken(QMouseEvent *);

    //used to walk up the popup list
    struct QMenuCaused {
        QPointer<QWidget> widget;
        QPointer<QAction> action;
    };
    virtual QList<QPointer<QWidget> > calcCausedStack() const;
    QMenuCaused causedPopup;
    void hideUpToMenuBar();
    void hideMenu(QMenu *menu);

    //index mappings
    inline QAction *actionAt(int i) const { return q_func()->actions().at(i); }
    inline int indexOf(QAction *act) const { return q_func()->actions().indexOf(act); }

    //tear off support
    uint tearoff : 1, tornoff : 1, tearoffHighlighted : 1;
    QPointer<QTornOffMenu> tornPopup;

    mutable bool hasCheckableItems;

    QMenuSloppyState sloppyState;

    //default action
    QPointer<QAction> defaultAction;

    QAction *menuAction;
    QAction *defaultMenuAction;

    void setOverrideMenuAction(QAction *);
    void _q_overrideMenuActionDestroyed();

    //firing of events
    void activateAction(QAction *, QAction::ActionEvent, bool self=true);
    void activateCausedStack(const QList<QPointer<QWidget> > &, QAction *, QAction::ActionEvent, bool);

    void _q_actionTriggered();
    void _q_actionHovered();
    void _q_platformMenuAboutToShow();

    bool hasMouseMoved(const QPoint &globalPos);

    void updateLayoutDirection();

    //menu fading/scrolling effects
    bool doChildEffects;

    QPointer<QPlatformMenu> platformMenu;

    QPointer<QAction> actionAboutToTrigger;

#if defined(Q_OS_WINCE) && !defined(QT_NO_MENUBAR)
    struct QWceMenuPrivate {
        QList<QWceMenuAction*> actionItems;
        HMENU menuHandle;
        QWceMenuPrivate();
        ~QWceMenuPrivate();
        void addAction(QAction *, QWceMenuAction* =0);
        void addAction(QWceMenuAction *, QWceMenuAction* =0);
        void syncAction(QWceMenuAction *);
        inline void syncAction(QAction *a) { syncAction(findAction(a)); }
        void removeAction(QWceMenuAction *);
        void rebuild();
        inline void removeAction(QAction *a) { removeAction(findAction(a)); }
        inline QWceMenuAction *findAction(QAction *a) {
            for(int i = 0; i < actionItems.size(); i++) {
                QWceMenuAction *act = actionItems[i];
                if(a == act->action)
                    return act;
            }
            return 0;
        }
    } *wce_menu;
    HMENU wceMenu();
    QAction* wceCommands(uint command);
#endif
    QPointer<QWidget> noReplayFor;
    static QPointer<QMenu> previousMouseMenu;
};

#endif // QT_NO_MENU

QT_END_NAMESPACE

#endif // QMENU_P_H
