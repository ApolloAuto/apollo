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

#ifndef QXCBWINDOWFUNCTIONS_H
#define QXCBWINDOWFUNCTIONS_H

#include <QtCore/QByteArray>
#include <QtGui/QGuiApplication>

QT_BEGIN_NAMESPACE

class QWindow;

class QXcbWindowFunctions {
public:
    enum WmWindowType {
        Normal       = 0x000001,
        Desktop      = 0x000002,
        Dock         = 0x000004,
        Toolbar      = 0x000008,
        Menu         = 0x000010,
        Utility      = 0x000020,
        Splash       = 0x000040,
        Dialog       = 0x000080,
        DropDownMenu = 0x000100,
        PopupMenu    = 0x000200,
        Tooltip      = 0x000400,
        Notification = 0x000800,
        Combo        = 0x001000,
        Dnd          = 0x002000,
        KdeOverride  = 0x004000
    };

    Q_DECLARE_FLAGS(WmWindowTypes, WmWindowType)

    typedef void (*SetWmWindowType)(QWindow *window, QXcbWindowFunctions::WmWindowTypes windowType);
    static const QByteArray setWmWindowTypeIdentifier() { return QByteArrayLiteral("XcbSetWmWindowType"); }

    static void setWmWindowType(QWindow *window, WmWindowType type)
    {
        SetWmWindowType func = reinterpret_cast<SetWmWindowType>(QGuiApplication::platformFunction(setWmWindowTypeIdentifier()));
        if (func)
            func(window, type);
    }

    typedef uint (*VisualId)(QWindow *window);
    static const QByteArray visualIdIdentifier() { return QByteArrayLiteral("XcbVisualId"); }

    static uint visualId(QWindow *window)
    {
        QXcbWindowFunctions::VisualId func = reinterpret_cast<VisualId>(QGuiApplication::platformFunction(visualIdIdentifier()));
        if (func)
            return func(window);
        return UINT_MAX;
    }
};


QT_END_NAMESPACE

#endif // QXCBWINDOWFUNCTIONS_H
