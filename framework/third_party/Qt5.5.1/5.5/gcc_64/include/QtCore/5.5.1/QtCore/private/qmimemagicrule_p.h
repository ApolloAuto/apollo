/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
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

#ifndef QMIMEMAGICRULE_P_H
#define QMIMEMAGICRULE_P_H

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

#include <QtCore/qglobal.h>

#ifndef QT_NO_MIMETYPE

#include <QtCore/qbytearray.h>
#include <QtCore/qscopedpointer.h>
#include <QtCore/qlist.h>

QT_BEGIN_NAMESPACE

class QMimeMagicRulePrivate;
class QMimeMagicRule
{
public:
    enum Type { Invalid = 0, String, Host16, Host32, Big16, Big32, Little16, Little32, Byte };

    QMimeMagicRule(const QString &typeStr, const QByteArray &value, const QString &offsets,
                   const QByteArray &mask, QString *errorString);
    QMimeMagicRule(const QMimeMagicRule &other);
    ~QMimeMagicRule();

    QMimeMagicRule &operator=(const QMimeMagicRule &other);

    bool operator==(const QMimeMagicRule &other) const;

    Type type() const;
    QByteArray value() const;
    int startPos() const;
    int endPos() const;
    QByteArray mask() const;

    bool isValid() const;

    bool matches(const QByteArray &data) const;

    QList<QMimeMagicRule> m_subMatches;

    static Type type(const QByteArray &type);
    static QByteArray typeName(Type type);

    static bool matchSubstring(const char *dataPtr, int dataSize, int rangeStart, int rangeLength, int valueLength, const char *valueData, const char *mask);

private:
    const QScopedPointer<QMimeMagicRulePrivate> d;
};
Q_DECLARE_TYPEINFO(QMimeMagicRule, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

#endif // QT_NO_MIMETYPE
#endif // QMIMEMAGICRULE_H
