/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Assistant of the Qt Toolkit.
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

#ifndef QHELPDBREADER_H
#define QHELPDBREADER_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists for the convenience
// of the help generator tools. This header file may change from version
// to version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtCore/QUrl>
#include <QtCore/QByteArray>
#include <QtCore/QSet>

QT_BEGIN_NAMESPACE

class QSqlQuery;

class QHelpDBReader : public QObject
{
    Q_OBJECT

public:
    QHelpDBReader(const QString &dbName);
    QHelpDBReader(const QString &dbName, const QString &uniqueId,
        QObject *parent);
    ~QHelpDBReader();

    bool init();

    QString errorMessage() const;

    QString databaseName() const;
    QString namespaceName() const;
    QString virtualFolder() const;
    QList<QStringList> filterAttributeSets() const;
    QStringList files(const QStringList &filterAttributes,
        const QString &extensionFilter = QString()) const;
    bool fileExists(const QString &virtualFolder, const QString &filePath,
        const QStringList &filterAttributes = QStringList()) const;
    QByteArray fileData(const QString &virtualFolder,
        const QString &filePath) const;

    QStringList customFilters() const;
    QStringList filterAttributes(const QString &filterName = QString()) const;
    QStringList indicesForFilter(const QStringList &filterAttributes) const;
    void linksForKeyword(const QString &keyword, const QStringList &filterAttributes,
        QMap<QString, QUrl> &linkMap) const;

    void linksForIdentifier(const QString &id, const QStringList &filterAttributes,
        QMap<QString, QUrl> &linkMap) const;

    QList<QByteArray> contentsForFilter(const QStringList &filterAttributes) const;
    QUrl urlOfPath(const QString &relativePath) const;

    QSet<int> indexIds(const QStringList &attributes) const;
    bool createAttributesCache(const QStringList &attributes,
        const QSet<int> &indexIds);
    QVariant metaData(const QString &name) const;

private:
    void initObject(const QString &dbName, const QString &uniqueId);
    QUrl buildQUrl(const QString &ns, const QString &folder,
        const QString &relFileName, const QString &anchor) const;
    QString mergeList(const QStringList &list) const;
    QString quote(const QString &string) const;

    bool m_initDone;
    QString m_dbName;
    QString m_uniqueId;
    QString m_error;
    QSqlQuery *m_query;
    mutable QString m_namespace;
    QSet<QString> m_viewAttributes;
    bool m_useAttributesCache;
    QSet<int> m_indicesCache;
};

QT_END_NAMESPACE

#endif
