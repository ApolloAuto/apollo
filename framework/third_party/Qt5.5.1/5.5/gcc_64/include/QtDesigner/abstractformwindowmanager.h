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

#ifndef ABSTRACTFORMWINDOWMANAGER_H
#define ABSTRACTFORMWINDOWMANAGER_H

#include <QtDesigner/sdk_global.h>
#include <QtDesigner/abstractformwindow.h>

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>

QT_BEGIN_NAMESPACE

class QDesignerFormEditorInterface;
class QDesignerDnDItemInterface;

class QWidget;
class QPixmap;
class QAction;
class QActionGroup;

class QDESIGNER_SDK_EXPORT QDesignerFormWindowManagerInterface: public QObject
{
    Q_OBJECT
public:
    explicit QDesignerFormWindowManagerInterface(QObject *parent = 0);
    virtual ~QDesignerFormWindowManagerInterface();

    enum Action
    {
        CutAction = 100,
        CopyAction,
        PasteAction,
        DeleteAction,
        SelectAllAction,

        LowerAction = 200,
        RaiseAction,

        UndoAction =  300,
        RedoAction,

        HorizontalLayoutAction = 400,
        VerticalLayoutAction,
        SplitHorizontalAction,
        SplitVerticalAction,
        GridLayoutAction,
        FormLayoutAction,
        BreakLayoutAction,
        AdjustSizeAction,
        SimplifyLayoutAction,

        DefaultPreviewAction = 500,

        FormWindowSettingsDialogAction =  600
    };

    enum ActionGroup
    {
        StyledPreviewActionGroup = 100
    };

    virtual QAction *action(Action action) const = 0;
    virtual QActionGroup *actionGroup(ActionGroup actionGroup) const = 0;

    QAction *actionCut() const;
    QAction *actionCopy() const;
    QAction *actionPaste() const;
    QAction *actionDelete() const;
    QAction *actionSelectAll() const;
    QAction *actionLower() const;
    QAction *actionRaise() const;
    QAction *actionUndo() const;
    QAction *actionRedo() const;

    QAction *actionHorizontalLayout() const;
    QAction *actionVerticalLayout() const;
    QAction *actionSplitHorizontal() const;
    QAction *actionSplitVertical() const;
    QAction *actionGridLayout() const;
    QAction *actionFormLayout() const;
    QAction *actionBreakLayout() const;
    QAction *actionAdjustSize() const;
    QAction *actionSimplifyLayout() const;

    virtual QDesignerFormWindowInterface *activeFormWindow() const = 0;

    virtual int formWindowCount() const = 0;
    virtual QDesignerFormWindowInterface *formWindow(int index) const = 0;

    virtual QDesignerFormWindowInterface *createFormWindow(QWidget *parentWidget = 0, Qt::WindowFlags flags = 0) = 0;

    virtual QDesignerFormEditorInterface *core() const = 0;

    virtual void dragItems(const QList<QDesignerDnDItemInterface*> &item_list) = 0;

    virtual QPixmap createPreviewPixmap() const = 0;

Q_SIGNALS:
    void formWindowAdded(QDesignerFormWindowInterface *formWindow);
    void formWindowRemoved(QDesignerFormWindowInterface *formWindow);
    void activeFormWindowChanged(QDesignerFormWindowInterface *formWindow);
    void formWindowSettingsChanged(QDesignerFormWindowInterface *fw);

public Q_SLOTS:
    virtual void addFormWindow(QDesignerFormWindowInterface *formWindow) = 0;
    virtual void removeFormWindow(QDesignerFormWindowInterface *formWindow) = 0;
    virtual void setActiveFormWindow(QDesignerFormWindowInterface *formWindow) = 0;
    virtual void showPreview() = 0;
    virtual void closeAllPreviews() = 0;
    virtual void showPluginDialog() = 0;

private:
    QScopedPointer<int> d;
};

QT_END_NAMESPACE

#endif // ABSTRACTFORMWINDOWMANAGER_H
