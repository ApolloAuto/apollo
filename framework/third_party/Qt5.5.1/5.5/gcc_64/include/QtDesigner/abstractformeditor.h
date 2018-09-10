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

#ifndef ABSTRACTFORMEDITOR_H
#define ABSTRACTFORMEDITOR_H

#include <QtDesigner/sdk_global.h>

#include <QtCore/QObject>
#include <QtCore/QPointer>

QT_BEGIN_NAMESPACE

class QDesignerWidgetBoxInterface;
class QDesignerPropertyEditorInterface;
class QDesignerFormWindowManagerInterface;
class QDesignerWidgetDataBaseInterface;
class QDesignerMetaDataBaseInterface;
class QDesignerWidgetFactoryInterface;
class QDesignerObjectInspectorInterface;
class QDesignerPromotionInterface;
class QDesignerActionEditorInterface;
class QDesignerIntegrationInterface;
class QDesignerPluginManager;
class QDesignerIntrospectionInterface;
class QDesignerDialogGuiInterface;
class QDesignerSettingsInterface;
class QDesignerOptionsPageInterface;
class QtResourceModel;
class QtGradientManager;

class QWidget;
class QIcon;

class QExtensionManager;

class  QDesignerFormEditorInterfacePrivate;

class QDESIGNER_SDK_EXPORT QDesignerFormEditorInterface : public QObject
{
    Q_OBJECT
public:
    QDesignerFormEditorInterface(QObject *parent = 0);
    virtual ~QDesignerFormEditorInterface();

    QExtensionManager *extensionManager() const;

    QWidget *topLevel() const;
    QDesignerWidgetBoxInterface *widgetBox() const;
    QDesignerPropertyEditorInterface *propertyEditor() const;
    QDesignerObjectInspectorInterface *objectInspector() const;
    QDesignerFormWindowManagerInterface *formWindowManager() const;
    QDesignerWidgetDataBaseInterface *widgetDataBase() const;
    QDesignerMetaDataBaseInterface *metaDataBase() const;
    QDesignerPromotionInterface *promotion() const;
    QDesignerWidgetFactoryInterface *widgetFactory() const;
    QDesignerActionEditorInterface *actionEditor() const;
    QDesignerIntegrationInterface *integration() const;
    QDesignerPluginManager *pluginManager() const;
    QDesignerIntrospectionInterface *introspection() const;
    QDesignerDialogGuiInterface *dialogGui() const;
    QDesignerSettingsInterface *settingsManager() const;
    QString resourceLocation() const;
    QtResourceModel *resourceModel() const;
    QtGradientManager *gradientManager() const;
    QList<QDesignerOptionsPageInterface*> optionsPages() const;

    void setTopLevel(QWidget *topLevel);
    void setWidgetBox(QDesignerWidgetBoxInterface *widgetBox);
    void setPropertyEditor(QDesignerPropertyEditorInterface *propertyEditor);
    void setObjectInspector(QDesignerObjectInspectorInterface *objectInspector);
    void setPluginManager(QDesignerPluginManager *pluginManager);
    void setActionEditor(QDesignerActionEditorInterface *actionEditor);
    void setIntegration(QDesignerIntegrationInterface *integration);
    void setIntrospection(QDesignerIntrospectionInterface *introspection);
    void setDialogGui(QDesignerDialogGuiInterface *dialogGui);
    void setSettingsManager(QDesignerSettingsInterface *settingsManager);
    void setResourceModel(QtResourceModel *model);
    void setGradientManager(QtGradientManager *manager);
    void setOptionsPages(const QList<QDesignerOptionsPageInterface*> &optionsPages);

    QObjectList pluginInstances() const;

    static QIcon createIcon(const QString &name);

protected:
    void setFormManager(QDesignerFormWindowManagerInterface *formWindowManager);
    void setMetaDataBase(QDesignerMetaDataBaseInterface *metaDataBase);
    void setWidgetDataBase(QDesignerWidgetDataBaseInterface *widgetDataBase);
    void setPromotion(QDesignerPromotionInterface *promotion);
    void setWidgetFactory(QDesignerWidgetFactoryInterface *widgetFactory);
    void setExtensionManager(QExtensionManager *extensionManager);

private:
    QScopedPointer<QDesignerFormEditorInterfacePrivate> d;

private:
    QDesignerFormEditorInterface(const QDesignerFormEditorInterface &other);
    void operator = (const QDesignerFormEditorInterface &other);
};

QT_END_NAMESPACE

#endif // ABSTRACTFORMEDITOR_H
