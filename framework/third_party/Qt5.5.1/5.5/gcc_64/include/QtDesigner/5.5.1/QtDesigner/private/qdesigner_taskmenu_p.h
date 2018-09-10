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

#ifndef QDESIGNER_TASKMENU_H
#define QDESIGNER_TASKMENU_H

#include "shared_global_p.h"
#include "extensionfactory_p.h"
#include <QtDesigner/QDesignerTaskMenuExtension>

#include <QtCore/QObject>
#include <QtCore/QPointer>
#include <QtCore/QList>

QT_BEGIN_NAMESPACE

class QDesignerFormWindowInterface;
class QDesignerFormEditorInterface;

class QWidget;
class QSignalMapper;

namespace qdesigner_internal {
class QDesignerTaskMenuPrivate;

class QDESIGNER_SHARED_EXPORT QDesignerTaskMenu: public QObject, public QDesignerTaskMenuExtension
{
    Q_OBJECT
    Q_INTERFACES(QDesignerTaskMenuExtension)
public:
    QDesignerTaskMenu(QWidget *widget, QObject *parent);
    virtual ~QDesignerTaskMenu();

    QWidget *widget() const;

    QList<QAction*> taskActions() const Q_DECL_OVERRIDE;

    enum PropertyMode { CurrentWidgetMode, MultiSelectionMode };

    static bool isSlotNavigationEnabled(const QDesignerFormEditorInterface *core);
    static void navigateToSlot(QDesignerFormEditorInterface *core, QObject *o,
                               const QString &defaultSignal = QString());

protected:

    QDesignerFormWindowInterface *formWindow() const;
    void changeTextProperty(const QString &propertyName, const QString &windowTitle, PropertyMode pm, Qt::TextFormat desiredFormat);

    QAction *createSeparator();

    /* Retrieve the list of objects the task menu is supposed to act on. Note that a task menu can be invoked for
     * an unmanaged widget [as of 4.5], in which case it must not use the cursor selection,
     * but the unmanaged selection of the object inspector. */
    QObjectList applicableObjects(const QDesignerFormWindowInterface *fw, PropertyMode pm) const;
    QList<QWidget *> applicableWidgets(const QDesignerFormWindowInterface *fw, PropertyMode pm) const;

    void setProperty(QDesignerFormWindowInterface *fw, PropertyMode pm, const QString &name, const QVariant &newValue);

private slots:
    void changeObjectName();
    void changeToolTip();
    void changeWhatsThis();
    void changeStyleSheet();
    void createMenuBar();
    void addToolBar();
    void createStatusBar();
    void removeStatusBar();
    void containerFakeMethods();
    void slotNavigateToSlot();
    void applySize(QAction *a);
    void slotLayoutAlignment();

private:
    QDesignerTaskMenuPrivate *d;
};

typedef ExtensionFactory<QDesignerTaskMenuExtension, QWidget, QDesignerTaskMenu>  QDesignerTaskMenuFactory;

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // QDESIGNER_TASKMENU_H
