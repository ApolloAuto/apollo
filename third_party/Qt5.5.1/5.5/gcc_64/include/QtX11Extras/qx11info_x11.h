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

#ifndef QX11INFO_X11_H
#define QX11INFO_X11_H

#include <QtCore/qnamespace.h>
#include "QtX11Extras/qtx11extrasglobal.h"

typedef struct _XDisplay Display;
struct xcb_connection_t;

QT_BEGIN_NAMESPACE

class Q_X11EXTRAS_EXPORT QX11Info
{
public:
    static bool isPlatformX11();

    static int appDpiX(int screen=-1);
    static int appDpiY(int screen=-1);

    static unsigned long appRootWindow(int screen=-1);
    static int appScreen();

    static unsigned long appTime();
    static unsigned long appUserTime();

    static void setAppTime(unsigned long time);
    static void setAppUserTime(unsigned long time);

    static unsigned long getTimestamp();

    static QByteArray nextStartupId();
    static void setNextStartupId(const QByteArray &id);

    static Display *display();
    static xcb_connection_t *connection();

private:
    QX11Info();
};

QT_END_NAMESPACE

#endif // QX11INFO_X11_H

