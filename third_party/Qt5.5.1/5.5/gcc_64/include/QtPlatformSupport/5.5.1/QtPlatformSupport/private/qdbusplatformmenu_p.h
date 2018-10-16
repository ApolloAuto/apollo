/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QDBUSPLATFORMMENU_H
#define QDBUSPLATFORMMENU_H
//
//  W A R N I N G
//  -------------
//
// This file is part of the DBus menu support and is not meant to be used
// in applications. Usage of this API may make your code
// source and binary incompatible with future versions of Qt.
//

#include <qpa/qplatformmenu.h>
#include <QLoggingCategory>
#include "qdbusmenutypes_p.h"

QT_BEGIN_NAMESPACE
Q_DECLARE_LOGGING_CATEGORY(qLcMenu)

class QDBusPlatformMenu;

class QDBusPlatformMenuItem : public QPlatformMenuItem
{
    Q_OBJECT

public:
    QDBusPlatformMenuItem(quintptr tag = 0LL);
    ~QDBusPlatformMenuItem();

    quintptr tag()const Q_DECL_OVERRIDE { return m_tag; }
    void setTag(quintptr tag) Q_DECL_OVERRIDE;

    const QString text() const { return m_text; }
    void setText(const QString &text) Q_DECL_OVERRIDE;
    QIcon icon() const { return m_icon; }
    void setIcon(const QIcon &icon) Q_DECL_OVERRIDE;
    const QPlatformMenu *menu() const { return m_subMenu; }
    void setMenu(QPlatformMenu *menu) Q_DECL_OVERRIDE;
    bool isEnabled() const { return m_isEnabled; }
    void setEnabled(bool enabled) Q_DECL_OVERRIDE;
    bool isVisible() const { return m_isVisible; }
    void setVisible(bool isVisible) Q_DECL_OVERRIDE;
    bool isSeparator() const { return m_isSeparator; }
    void setIsSeparator(bool isSeparator) Q_DECL_OVERRIDE;
    void setFont(const QFont &font) Q_DECL_OVERRIDE { Q_UNUSED(font); }
    void setRole(MenuRole role) Q_DECL_OVERRIDE;
    bool isCheckable() const { return m_isCheckable; }
    void setCheckable(bool checkable) Q_DECL_OVERRIDE;
    bool isChecked() const { return m_isChecked; }
    void setChecked(bool isChecked) Q_DECL_OVERRIDE;
    QKeySequence shortcut() const { return m_shortcut; }
    void setShortcut(const QKeySequence& shortcut) Q_DECL_OVERRIDE;
    void setIconSize(int size) Q_DECL_OVERRIDE { Q_UNUSED(size); }
    void setNativeContents(WId item) Q_DECL_OVERRIDE { Q_UNUSED(item); }

    int dbusID() const { return m_dbusID; }

    void trigger();

    bool operator==(const QDBusPlatformMenuItem& other) { return m_tag == other.m_tag; }

    static QDBusPlatformMenuItem *byId(int id);
    static QList<const QDBusPlatformMenuItem *> byIds(const QList<int> &ids);

private:
    quintptr m_tag;
    QString m_text;
    QIcon m_icon;
    QPlatformMenu *m_subMenu;
    MenuRole m_role : 4;
    bool m_isEnabled : 1;
    bool m_isVisible : 1;
    bool m_isSeparator : 1;
    bool m_isCheckable : 1;
    bool m_isChecked : 1;
    int m_dbusID : 16;
    int m_reserved : 7;
    QKeySequence m_shortcut;
};

class QDBusPlatformMenu : public QPlatformMenu
{
    Q_OBJECT

public:
    QDBusPlatformMenu(quintptr tag = 0LL);
    ~QDBusPlatformMenu();
    void insertMenuItem(QPlatformMenuItem *menuItem, QPlatformMenuItem *before) Q_DECL_OVERRIDE;
    void removeMenuItem(QPlatformMenuItem *menuItem) Q_DECL_OVERRIDE;
    void syncMenuItem(QPlatformMenuItem *menuItem) Q_DECL_OVERRIDE;
    void syncSeparatorsCollapsible(bool enable) Q_DECL_OVERRIDE { Q_UNUSED(enable); }

    quintptr tag()const Q_DECL_OVERRIDE { return m_tag; }
    void setTag(quintptr tag) Q_DECL_OVERRIDE;

    const QString text() const { return m_text; }
    void setText(const QString &text) Q_DECL_OVERRIDE;
    void setIcon(const QIcon &icon) Q_DECL_OVERRIDE;
    void setEnabled(bool enabled) Q_DECL_OVERRIDE;
    void setVisible(bool visible) Q_DECL_OVERRIDE;
    void setMinimumWidth(int width) Q_DECL_OVERRIDE { Q_UNUSED(width); }
    void setFont(const QFont &font) Q_DECL_OVERRIDE { Q_UNUSED(font); }
    void setMenuType(MenuType type) Q_DECL_OVERRIDE { Q_UNUSED(type); }

    int dbusID() const { return m_dbusID; }

    void showPopup(const QWindow *parentWindow, const QRect &targetRect, const QPlatformMenuItem *item) Q_DECL_OVERRIDE
    {
        Q_UNUSED(parentWindow);
        Q_UNUSED(targetRect);
        Q_UNUSED(item);
        setVisible(true);
    }

    void dismiss() Q_DECL_OVERRIDE { } // Closes this and all its related menu popups

    QPlatformMenuItem *menuItemAt(int position) const Q_DECL_OVERRIDE;
    QPlatformMenuItem *menuItemForTag(quintptr tag) const Q_DECL_OVERRIDE;
    const QList<QDBusPlatformMenuItem *> items() const;

    QPlatformMenuItem *createMenuItem() const Q_DECL_OVERRIDE;
    QPlatformMenu *createSubMenu() const Q_DECL_OVERRIDE;

    bool operator==(const QDBusPlatformMenu& other) { return m_tag == other.m_tag; }

    static QDBusPlatformMenu* byId(int id);
    static QList<QDBusPlatformMenu *> topLevelMenus() { return m_topLevelMenus; }

    uint revision() const { return m_revision; }

    void emitUpdated();

signals:
    void updated(uint revision, int dbusId);
    void propertiesUpdated(QDBusMenuItemList updatedProps, QDBusMenuItemKeysList removedProps);

private:
    quintptr m_tag;
    QString m_text;
    QIcon m_icon;
    bool m_isEnabled;
    bool m_isVisible;
    bool m_isSeparator;
    int m_dbusID;
    uint m_revision;
    QHash<quintptr, QDBusPlatformMenuItem *> m_itemsByTag;
    QList<QDBusPlatformMenuItem *> m_items;
    QDBusPlatformMenuItem *m_containingMenuItem;
    static QList<QDBusPlatformMenu *> m_topLevelMenus;
};

QT_END_NAMESPACE

#endif

