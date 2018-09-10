/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
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

#ifndef QGENERICUNIXTHEMES_H
#define QGENERICUNIXTHEMES_H

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

#include <qpa/qplatformtheme.h>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtGui/QFont>

QT_BEGIN_NAMESPACE

class ResourceHelper
{
public:
    ResourceHelper();
    ~ResourceHelper() { clear(); }

    void clear();

    QPalette *palettes[QPlatformTheme::NPalettes];
    QFont *fonts[QPlatformTheme::NFonts];
};

class QGenericUnixThemePrivate;

class QGenericUnixTheme : public QPlatformTheme
{
    Q_DECLARE_PRIVATE(QGenericUnixTheme)
public:
    QGenericUnixTheme();

    static QPlatformTheme *createUnixTheme(const QString &name);
    static QStringList themeNames();

    virtual const QFont *font(Font type) const Q_DECL_OVERRIDE;
    virtual QVariant themeHint(ThemeHint hint) const Q_DECL_OVERRIDE;

    static QStringList xdgIconThemePaths();
#if !defined(QT_NO_DBUS) && !defined(QT_NO_SYSTEMTRAYICON)
    QPlatformSystemTrayIcon *createPlatformSystemTrayIcon() const Q_DECL_OVERRIDE;
#endif

    static const char *name;
};

#ifndef QT_NO_SETTINGS
class QKdeThemePrivate;

class QKdeTheme : public QPlatformTheme
{
    Q_DECLARE_PRIVATE(QKdeTheme)
public:
    QKdeTheme(const QStringList& kdeDirs, int kdeVersion);

    static QPlatformTheme *createKdeTheme();
    virtual QVariant themeHint(ThemeHint hint) const Q_DECL_OVERRIDE;

    virtual const QPalette *palette(Palette type = SystemPalette) const Q_DECL_OVERRIDE;

    virtual const QFont *font(Font type) const Q_DECL_OVERRIDE;
#if !defined(QT_NO_DBUS) && !defined(QT_NO_SYSTEMTRAYICON)
    QPlatformSystemTrayIcon *createPlatformSystemTrayIcon() const Q_DECL_OVERRIDE;
#endif

    static const char *name;
};
#endif // QT_NO_SETTINGS

class QGnomeThemePrivate;

class QGnomeTheme : public QPlatformTheme
{
    Q_DECLARE_PRIVATE(QGnomeTheme)
public:
    QGnomeTheme();
    virtual QVariant themeHint(ThemeHint hint) const Q_DECL_OVERRIDE;
    virtual const QFont *font(Font type) const Q_DECL_OVERRIDE;
    QString standardButtonText(int button) const Q_DECL_OVERRIDE;

    virtual QString gtkFontName() const;
#if !defined(QT_NO_DBUS) && !defined(QT_NO_SYSTEMTRAYICON)
    QPlatformSystemTrayIcon *createPlatformSystemTrayIcon() const Q_DECL_OVERRIDE;
#endif

    static const char *name;
};

QPlatformTheme *qt_createUnixTheme();

QT_END_NAMESPACE

#endif // QGENERICUNIXTHEMES_H
