/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
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

#ifndef QSOURCELOCATION_H
#define QSOURCELOCATION_H

#include <QtCore/QMetaType>
#include <QtCore/QUrl>
#include <QtXmlPatterns/qtxmlpatternsglobal.h>

QT_BEGIN_NAMESPACE


class QSourceLocationPrivate;

class Q_XMLPATTERNS_EXPORT QSourceLocation
{
public:
    QSourceLocation();
    QSourceLocation(const QSourceLocation &other);
    QSourceLocation(const QUrl &uri, int line = -1, int column = -1);
    ~QSourceLocation();
    QSourceLocation &operator=(const QSourceLocation &other);
    bool operator==(const QSourceLocation &other) const;
    bool operator!=(const QSourceLocation &other) const;

    qint64 column() const;
    void setColumn(qint64 newColumn);

    qint64 line() const;
    void setLine(qint64 newLine);

    QUrl uri() const;
    void setUri(const QUrl &newUri);
    bool isNull() const;

private:
    union
    {
        qint64 m_line;
        QSourceLocationPrivate *m_ptr;
    };
    qint64 m_column;
    QUrl m_uri;
};

Q_XMLPATTERNS_EXPORT uint qHash(const QSourceLocation &location);

#ifndef QT_NO_DEBUG_STREAM
Q_XMLPATTERNS_EXPORT QDebug operator<<(QDebug debug, const QSourceLocation &sourceLocation);
#endif

Q_DECLARE_TYPEINFO(QSourceLocation, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QSourceLocation) /* This macro must appear after QT_END_NAMESPACE. */

#endif
