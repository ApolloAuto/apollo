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

#ifndef FORMBUILDER_H
#define FORMBUILDER_H

#include "uilib_global.h"
#include "abstractformbuilder.h"

QT_BEGIN_NAMESPACE
#if 0
// pragma for syncqt, don't remove.

#pragma qt_class(QFormBuilder)
#endif

class QDesignerCustomWidgetInterface;

#ifdef QFORMINTERNAL_NAMESPACE
namespace QFormInternal
{
#endif

class QDESIGNER_UILIB_EXPORT QFormBuilder: public QAbstractFormBuilder
{
public:
    QFormBuilder();
    virtual ~QFormBuilder();

    QStringList pluginPaths() const;

    void clearPluginPaths();
    void addPluginPath(const QString &pluginPath);
    void setPluginPath(const QStringList &pluginPaths);

    QList<QDesignerCustomWidgetInterface*> customWidgets() const;

protected:
    QWidget *create(DomUI *ui, QWidget *parentWidget) Q_DECL_OVERRIDE;
    QWidget *create(DomWidget *ui_widget, QWidget *parentWidget) Q_DECL_OVERRIDE;
    QLayout *create(DomLayout *ui_layout, QLayout *layout, QWidget *parentWidget) Q_DECL_OVERRIDE;
    QLayoutItem *create(DomLayoutItem *ui_layoutItem, QLayout *layout, QWidget *parentWidget) Q_DECL_OVERRIDE;
    QAction *create(DomAction *ui_action, QObject *parent) Q_DECL_OVERRIDE;
    QActionGroup *create(DomActionGroup *ui_action_group, QObject *parent) Q_DECL_OVERRIDE;

    QWidget *createWidget(const QString &widgetName, QWidget *parentWidget, const QString &name) Q_DECL_OVERRIDE;
    QLayout *createLayout(const QString &layoutName, QObject *parent, const QString &name) Q_DECL_OVERRIDE;

    void createConnections(DomConnections *connections, QWidget *widget) Q_DECL_OVERRIDE;

    bool addItem(DomLayoutItem *ui_item, QLayoutItem *item, QLayout *layout) Q_DECL_OVERRIDE;
    bool addItem(DomWidget *ui_widget, QWidget *widget, QWidget *parentWidget) Q_DECL_OVERRIDE;

    virtual void updateCustomWidgets();
    void applyProperties(QObject *o, const QList<DomProperty*> &properties) Q_DECL_OVERRIDE;

    static QWidget *widgetByName(QWidget *topLevel, const QString &name);

private:
};

#ifdef QFORMINTERNAL_NAMESPACE
}
#endif

QT_END_NAMESPACE

#endif // FORMBUILDER_H
