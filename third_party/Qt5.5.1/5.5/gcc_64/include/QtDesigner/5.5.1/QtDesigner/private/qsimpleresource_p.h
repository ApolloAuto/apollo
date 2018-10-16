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

#ifndef QSIMPLERESOURCE_H
#define QSIMPLERESOURCE_H

#include "shared_global_p.h"
#include "abstractformbuilder.h"
#include <QtCore/QStringList>

QT_BEGIN_NAMESPACE

class DomScript;
class DomCustomWidgets;
class DomCustomWidget;
class DomSlots;

class QDesignerFormEditorInterface;

namespace qdesigner_internal {

class WidgetDataBaseItem;

class QDESIGNER_SHARED_EXPORT QSimpleResource : public QAbstractFormBuilder
{
public:
    explicit QSimpleResource(QDesignerFormEditorInterface *core);
    virtual ~QSimpleResource();

    QBrush setupBrush(DomBrush *brush);
    DomBrush *saveBrush(const QBrush &brush);

    inline QDesignerFormEditorInterface *core() const
    { return m_core; }

    // Query extensions for additional data
    static void addExtensionDataToDOM(QAbstractFormBuilder *afb,
                                      QDesignerFormEditorInterface *core,
                                      DomWidget *ui_widget, QWidget *widget);
    static void applyExtensionDataFromDOM(QAbstractFormBuilder *afb,
                                          QDesignerFormEditorInterface *core,
                                          DomWidget *ui_widget, QWidget *widget);
    // Return the script returned by the CustomWidget codeTemplate API
    static QString customWidgetScript(QDesignerFormEditorInterface *core, QObject *object);
    static QString customWidgetScript(QDesignerFormEditorInterface *core, const QString &className);
    static bool hasCustomWidgetScript(QDesignerFormEditorInterface *core, QObject *object);

    // Implementation for FormBuilder::createDomCustomWidgets() that adds
    // the custom widgets to the widget database
    static void handleDomCustomWidgets(const QDesignerFormEditorInterface *core,
                                       const DomCustomWidgets *dom_custom_widgets);

protected:
    enum ScriptSource { ScriptDesigner, ScriptExtension, ScriptCustomWidgetPlugin };
    static DomScript*createScript(const QString &script, ScriptSource source);
    typedef QList<DomScript*> DomScripts;
    static void addScript(const QString &script, ScriptSource source, DomScripts &domScripts);

    static bool addFakeMethods(const DomSlots *domSlots, QStringList &fakeSlots, QStringList &fakeSignals);

private:
    static void addCustomWidgetsToWidgetDatabase(const QDesignerFormEditorInterface *core,
                                                 QList<DomCustomWidget*>& custom_widget_list);
    static void addFakeMethodsToWidgetDataBase(const DomCustomWidget *domCustomWidget, WidgetDataBaseItem *item);

    static bool m_warningsEnabled;
    QDesignerFormEditorInterface *m_core;
};

// Contents of clipboard for formbuilder copy and paste operations
// (Actions and widgets)
struct QDESIGNER_SHARED_EXPORT FormBuilderClipboard {
    typedef QList<QAction*> ActionList;

    FormBuilderClipboard() {}
    FormBuilderClipboard(QWidget *w);

    bool empty() const;

    QWidgetList m_widgets;
    ActionList m_actions;
};

// Base class for a form builder used in the editor that
// provides copy and paste.(move into base interface)
class QDESIGNER_SHARED_EXPORT QEditorFormBuilder : public QSimpleResource
{
public:
    explicit QEditorFormBuilder(QDesignerFormEditorInterface *core) : QSimpleResource(core) {}

    virtual bool copy(QIODevice *dev, const FormBuilderClipboard &selection) = 0;
    virtual DomUI *copy(const FormBuilderClipboard &selection) = 0;

    // A widget parent needs to be specified, otherwise, the widget factory cannot locate the form window via parent
    // and thus is not able to construct special widgets (QLayoutWidget).
    virtual FormBuilderClipboard paste(DomUI *ui, QWidget *widgetParent, QObject *actionParent = 0) = 0;
    virtual FormBuilderClipboard paste(QIODevice *dev, QWidget *widgetParent, QObject *actionParent = 0) = 0;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif
