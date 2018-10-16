/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtTest module of the Qt Toolkit.
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

#ifndef QTEST_WIDGETS_H
#define QTEST_WIDGETS_H

// enable WIDGETS features
#ifndef QT_WIDGETS_LIB
#define QT_WIDGETS_LIB
#endif
#if 0
#pragma qt_class(QtTestWidgets)
#endif

#include <QtTest/qtest_gui.h>

#if 0
// inform syncqt
#pragma qt_no_master_include
#endif

#include <QtWidgets/QSizePolicy>
#include <QtCore/QMetaEnum>

QT_BEGIN_NAMESPACE

namespace QTest
{

//
// QSizePolicy & friends:
//

namespace Internal
{

inline const char *toString(QSizePolicy::Policy p)
{
    static const QMetaEnum me = QSizePolicy::staticMetaObject.enumerator(QSizePolicy::staticMetaObject.indexOfEnumerator("Policy"));
    return me.valueToKey(int(p));
}

inline QByteArray toString(QSizePolicy::ControlTypes ct)
{
    static const QMetaEnum me = QSizePolicy::staticMetaObject.enumerator(QSizePolicy::staticMetaObject.indexOfEnumerator("ControlTypes"));
    return me.valueToKeys(int(ct));
}

inline QByteArray toString(QSizePolicy sp)
{
    static const char comma[] = ", ";
    return QByteArray("QSizePolicy(")
            + Internal::toString(sp.horizontalPolicy()) + comma
            + Internal::toString(sp.verticalPolicy()) + comma
            + QByteArray::number(sp.horizontalStretch()) + comma
            + QByteArray::number(sp.verticalStretch()) + comma
            + Internal::toString(QSizePolicy::ControlTypes(sp.controlType())) + comma
            + "height for width: " + (sp.hasHeightForWidth() ? "yes" : "no") + comma
            + "width for height: " + (sp.hasWidthForHeight() ? "yes" : "no") + comma
            + (sp.retainSizeWhenHidden() ? "" : "don't " ) + "retain size when hidden"
            + ')';
}

} // namespace Internal

inline char *toString(QSizePolicy::Policy p)
{
    return qstrdup(Internal::toString(p));
}

inline char *toString(QSizePolicy::ControlTypes ct)
{
    return qstrdup(Internal::toString(ct).constData());
}

inline char *toString(QSizePolicy::ControlType ct)
{
    return toString(QSizePolicy::ControlTypes(ct));
}

inline char *toString(QSizePolicy sp)
{
    return qstrdup(Internal::toString(sp).constData());
}

} // namespace QTest

QT_END_NAMESPACE

#endif

