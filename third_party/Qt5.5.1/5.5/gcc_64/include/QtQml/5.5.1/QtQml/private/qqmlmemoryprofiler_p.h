/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQMLMEMORYPROFILER_H
#define QQMLMEMORYPROFILER_H

#include <private/qtqmlglobal_p.h>

QT_BEGIN_NAMESPACE

class QUrl;

class Q_QML_PRIVATE_EXPORT QQmlMemoryScope
{
public:
    explicit QQmlMemoryScope(const QUrl &url);
    explicit QQmlMemoryScope(const char *string);
    ~QQmlMemoryScope();

private:
    bool pushed;
};

class Q_QML_PRIVATE_EXPORT QQmlMemoryProfiler
{
public:
    static void enable();
    static void disable();
    static bool isEnabled();

    static void clear();
    static void stats(int *allocCount, int *bytesAllocated);
    static void save(const char *filename);
};

#define QML_MEMORY_SCOPE_URL(url)       QQmlMemoryScope _qml_memory_scope(url)
#define QML_MEMORY_SCOPE_STRING(s)      QQmlMemoryScope _qml_memory_scope(s)

QT_END_NAMESPACE
#endif // QQMLMEMORYPROFILER_H
