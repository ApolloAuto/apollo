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

#ifndef QXMLNAME_H
#define QXMLNAME_H

#include <QtCore/QString>
#include <QtCore/QMetaType>
#include <QtXmlPatterns/qtxmlpatternsglobal.h>

QT_BEGIN_NAMESPACE


class QXmlName;
class QXmlNamePool;
Q_XMLPATTERNS_EXPORT uint qHash(const QXmlName &name);

class Q_XMLPATTERNS_EXPORT QXmlName
{
private:
    enum Constant
    {
        LocalNameOffset     = 0,
        LocalNameLength     = 12,
        NamespaceOffset     = LocalNameLength,
        NamespaceLength     = 9,
        PrefixLength        = 9,
        InvalidCode         = 1 << 31,
        NamespaceMask       = ((1 << ((NamespaceOffset + NamespaceLength) - NamespaceOffset)) - 1) << NamespaceOffset,
        LocalNameMask       = ((1 << ((LocalNameOffset + LocalNameLength) - LocalNameOffset)) - 1) << LocalNameOffset,
        PrefixOffset        = LocalNameLength + NamespaceLength,
        PrefixMask          = ((1 << ((PrefixOffset + PrefixLength) - PrefixOffset)) - 1) << PrefixOffset,
        MaximumPrefixes     = (PrefixMask >> PrefixOffset) - 1,
        MaximumLocalNames   = (LocalNameMask >> LocalNameOffset) - 1,
        MaximumNamespaces   = (NamespaceMask >> NamespaceOffset) - 1,
        ExpandedNameMask    = LocalNameMask | NamespaceMask,
        LexicalQNameMask    = LocalNameMask | PrefixMask
    };

public:

    typedef qint16 NamespaceCode;
    typedef NamespaceCode PrefixCode;
    typedef NamespaceCode LocalNameCode;

    QXmlName();

    QXmlName(QXmlNamePool &namePool,
             const QString &localName,
             const QString &namespaceURI = QString(),
             const QString &prefix = QString());

    QString namespaceUri(const QXmlNamePool &query) const;
    QString prefix(const QXmlNamePool &query) const;
    QString localName(const QXmlNamePool &query) const;
    QString toClarkName(const QXmlNamePool &query) const;
    bool operator==(const QXmlName &other) const;
    bool operator!=(const QXmlName &other) const;
    QXmlName &operator=(const QXmlName &other);
    bool isNull() const;
    static bool isNCName(const QString &candidate);
    static QXmlName fromClarkName(const QString &clarkName,
                                  const QXmlNamePool &namePool);

    /* The members below are internal, not part of the public API, and
     * unsupported. Using them leads to undefined behavior. */
    typedef qint64 Code;

    inline QXmlName(const NamespaceCode uri,
                    const LocalNameCode ln,
                    const PrefixCode p = 0);
    /* The implementation for these functions are in utils/qnamepool_p.h. */
    inline LocalNameCode localName() const;
    inline PrefixCode prefix() const;
    inline bool hasPrefix() const;
    inline bool hasNamespace() const;
    inline NamespaceCode namespaceURI() const;
    inline bool isLexicallyEqual(const QXmlName &other) const;
    inline void setPrefix(const PrefixCode c);
    inline void setNamespaceURI(const NamespaceCode c);
    inline void setLocalName(const LocalNameCode c);
    inline Code code() const;

    friend Q_XMLPATTERNS_EXPORT uint qHash(const QXmlName &);

private:
    inline QXmlName(const int c) : m_qNameCode(c)
    {
    }

    Code m_qNameCode;
};

Q_DECLARE_TYPEINFO(QXmlName, Q_MOVABLE_TYPE);

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QXmlName) /* This macro must appear after QT_END_NAMESPACE. */

#endif
