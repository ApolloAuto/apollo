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

#ifndef QMDIAREA_H
#define QMDIAREA_H

#include <QtWidgets/qabstractscrollarea.h>
#include <QtWidgets/qtabwidget.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_MDIAREA

class QMdiSubWindow;

class QMdiAreaPrivate;
class Q_WIDGETS_EXPORT QMdiArea : public QAbstractScrollArea
{
    Q_OBJECT
    Q_PROPERTY(QBrush background READ background WRITE setBackground)
    Q_PROPERTY(WindowOrder activationOrder READ activationOrder WRITE setActivationOrder)
    Q_PROPERTY(ViewMode viewMode READ viewMode WRITE setViewMode)
#ifndef QT_NO_TABBAR
    Q_PROPERTY(bool documentMode READ documentMode WRITE setDocumentMode)
    Q_PROPERTY(bool tabsClosable READ tabsClosable WRITE setTabsClosable)
    Q_PROPERTY(bool tabsMovable READ tabsMovable WRITE setTabsMovable)
#endif
#ifndef QT_NO_TABWIDGET
    Q_PROPERTY(QTabWidget::TabShape tabShape READ tabShape WRITE setTabShape)
    Q_PROPERTY(QTabWidget::TabPosition tabPosition READ tabPosition WRITE setTabPosition)
#endif
public:
    enum AreaOption {
        DontMaximizeSubWindowOnActivation = 0x1
    };
    Q_DECLARE_FLAGS(AreaOptions, AreaOption)

    enum WindowOrder {
        CreationOrder,
        StackingOrder,
        ActivationHistoryOrder
    };
    Q_ENUM(WindowOrder)

    enum ViewMode {
        SubWindowView,
        TabbedView
    };
    Q_ENUM(ViewMode)

    QMdiArea(QWidget *parent = 0);
    ~QMdiArea();

    QSize sizeHint() const Q_DECL_OVERRIDE;
    QSize minimumSizeHint() const Q_DECL_OVERRIDE;

    QMdiSubWindow *currentSubWindow() const;
    QMdiSubWindow *activeSubWindow() const;
    QList<QMdiSubWindow *> subWindowList(WindowOrder order = CreationOrder) const;

    QMdiSubWindow *addSubWindow(QWidget *widget, Qt::WindowFlags flags = 0);
    void removeSubWindow(QWidget *widget);

    QBrush background() const;
    void setBackground(const QBrush &background);

    WindowOrder activationOrder() const;
    void setActivationOrder(WindowOrder order);

    void setOption(AreaOption option, bool on = true);
    bool testOption(AreaOption opton) const;

    void setViewMode(ViewMode mode);
    ViewMode viewMode() const;

#ifndef QT_NO_TABBAR
    bool documentMode() const;
    void setDocumentMode(bool enabled);

    void setTabsClosable(bool closable);
    bool tabsClosable() const;

    void setTabsMovable(bool movable);
    bool tabsMovable() const;
#endif
#ifndef QT_NO_TABWIDGET
    void setTabShape(QTabWidget::TabShape shape);
    QTabWidget::TabShape tabShape() const;

    void setTabPosition(QTabWidget::TabPosition position);
    QTabWidget::TabPosition tabPosition() const;
#endif

Q_SIGNALS:
    void subWindowActivated(QMdiSubWindow *);

public Q_SLOTS:
    void setActiveSubWindow(QMdiSubWindow *window);
    void tileSubWindows();
    void cascadeSubWindows();
    void closeActiveSubWindow();
    void closeAllSubWindows();
    void activateNextSubWindow();
    void activatePreviousSubWindow();

protected Q_SLOTS:
    void setupViewport(QWidget *viewport) Q_DECL_OVERRIDE;

protected:
    bool event(QEvent *event) Q_DECL_OVERRIDE;
    bool eventFilter(QObject *object, QEvent *event) Q_DECL_OVERRIDE;
    void paintEvent(QPaintEvent *paintEvent) Q_DECL_OVERRIDE;
    void childEvent(QChildEvent *childEvent) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *resizeEvent) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *timerEvent) Q_DECL_OVERRIDE;
    void showEvent(QShowEvent *showEvent) Q_DECL_OVERRIDE;
    bool viewportEvent(QEvent *event) Q_DECL_OVERRIDE;
    void scrollContentsBy(int dx, int dy) Q_DECL_OVERRIDE;

private:
    Q_DISABLE_COPY(QMdiArea)
    Q_DECLARE_PRIVATE(QMdiArea)
    Q_PRIVATE_SLOT(d_func(), void _q_deactivateAllWindows())
    Q_PRIVATE_SLOT(d_func(), void _q_processWindowStateChanged(Qt::WindowStates, Qt::WindowStates))
    Q_PRIVATE_SLOT(d_func(), void _q_currentTabChanged(int))
    Q_PRIVATE_SLOT(d_func(), void _q_closeTab(int))
    Q_PRIVATE_SLOT(d_func(), void _q_moveTab(int, int))
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QMdiArea::AreaOptions)

QT_END_NAMESPACE

#endif // QT_NO_MDIAREA
#endif // QMDIAREA_H
