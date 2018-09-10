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


#ifndef WIDGETFACTORY_H
#define WIDGETFACTORY_H

#include "shared_global_p.h"
#include "pluginmanager_p.h"

#include <QtDesigner/QDesignerWidgetFactoryInterface>

#include <QtCore/QMap>
#include <QtCore/QHash>
#include <QtCore/QVariant>
#include <QtCore/QPointer>

QT_BEGIN_NAMESPACE

class QObject;
class QWidget;
class QLayout;
class QDesignerFormEditorInterface;
class QDesignerCustomWidgetInterface;
class QDesignerFormWindowInterface;
class QStyle;

namespace qdesigner_internal {

class QDESIGNER_SHARED_EXPORT WidgetFactory: public QDesignerWidgetFactoryInterface
{
    Q_OBJECT
public:
    explicit WidgetFactory(QDesignerFormEditorInterface *core, QObject *parent = 0);
    ~WidgetFactory();

    QWidget* containerOfWidget(QWidget *widget) const Q_DECL_OVERRIDE;
    QWidget* widgetOfContainer(QWidget *widget) const Q_DECL_OVERRIDE;

    QObject* createObject(const QString &className, QObject* parent) const;

    QWidget *createWidget(const QString &className, QWidget *parentWidget) const Q_DECL_OVERRIDE;
    QLayout *createLayout(QWidget *widget, QLayout *layout, int type) const Q_DECL_OVERRIDE;

    bool isPassiveInteractor(QWidget *widget) Q_DECL_OVERRIDE;
    void initialize(QObject *object) const Q_DECL_OVERRIDE;
    void initializeCommon(QWidget *object) const;
    void initializePreview(QWidget *object) const;


    QDesignerFormEditorInterface *core() const Q_DECL_OVERRIDE;

    static QString classNameOf(QDesignerFormEditorInterface *core, const QObject* o);

    QDesignerFormWindowInterface *currentFormWindow(QDesignerFormWindowInterface *fw);

    static QLayout *createUnmanagedLayout(QWidget *parentWidget, int type);

    // The widget factory maintains a cache of styles which it owns.
    QString styleName() const;
    void setStyleName(const QString &styleName);

    /* Return a cached style matching the name or QApplication's style if
     * it is the default. */
    QStyle *getStyle(const QString &styleName);
    // Return the current style used by the factory. This either a cached one
    // or QApplication's style */
    QStyle *style() const;

    // Apply one of the cached styles or QApplication's style to a toplevel widget.
    void applyStyleTopLevel(const QString &styleName, QWidget *w);
    static void applyStyleToTopLevel(QStyle *style, QWidget *widget);

    // Return whether object was created by the factory for the form editor.
    static bool isFormEditorObject(const QObject *o);

    // Boolean dynamic property to set on widgets to prevent custom
    // styles from interfering
    static const char *disableStyleCustomPaintingPropertyC;

public slots:
    void loadPlugins();

private slots:
    void activeFormWindowChanged(QDesignerFormWindowInterface *formWindow);
    void formWindowAdded(QDesignerFormWindowInterface *formWindow);

private:
    struct Strings { // Reduce string allocations by storing predefined strings
        Strings();
        const QString m_alignment;
        const QString m_bottomMargin;
        const QString m_geometry;
        const QString m_leftMargin;
        const QString m_line;
        const QString m_objectName;
        const QString m_spacerName;
        const QString m_orientation;
        const QString m_qAction;
        const QString m_qButtonGroup;
        const QString m_qAxWidget;
        const QString m_qDialog;
        const QString m_qDockWidget;
        const QString m_qLayoutWidget;
        const QString m_qMenu;
        const QString m_qMenuBar;
        const QString m_qWidget;
        const QString m_rightMargin;
        const QString m_sizeHint;
        const QString m_spacer;
        const QString m_text;
        const QString m_title;
        const QString m_topMargin;
        const QString m_windowIcon;
        const QString m_windowTitle;
    };

    QWidget* createCustomWidget(const QString &className, QWidget *parentWidget, bool *creationError) const;
    QDesignerFormWindowInterface *findFormWindow(QWidget *parentWidget) const;
    void setFormWindowStyle(QDesignerFormWindowInterface *formWindow);

    const Strings m_strings;
    QDesignerFormEditorInterface *m_core;
    typedef QMap<QString, QDesignerCustomWidgetInterface*> CustomWidgetFactoryMap;
    CustomWidgetFactoryMap m_customFactory;
    QDesignerFormWindowInterface *m_formWindow;

    // Points to the cached style or 0 if the default (qApp) is active
    QStyle *m_currentStyle;
    typedef QHash<QString, QStyle *> StyleCache;
    StyleCache m_styleCache;

    static QPointer<QWidget> *m_lastPassiveInteractor;
    static bool m_lastWasAPassiveInteractor;
};

} // namespace qdesigner_internal

QT_END_NAMESPACE

#endif // WIDGETFACTORY_H
