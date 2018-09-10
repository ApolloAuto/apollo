/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2015 Klaralvdalens Datakonsult AB, a KDAB Group company, info@kdab.com, author David Faure <david.faure@kdab.com>
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

#ifndef QMIMEPROVIDER_P_H
#define QMIMEPROVIDER_P_H

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

#include "qmimedatabase_p.h"

#ifndef QT_NO_MIMETYPE

#include <QtCore/qdatetime.h>
#include <QtCore/qset.h>
#include <QtCore/qelapsedtimer.h>

QT_BEGIN_NAMESPACE

class QMimeMagicRuleMatcher;

class QMimeProviderBase
{
public:
    QMimeProviderBase(QMimeDatabasePrivate *db);
    virtual ~QMimeProviderBase() {}

    virtual bool isValid() = 0;
    virtual QMimeType mimeTypeForName(const QString &name) = 0;
    virtual QStringList findByFileName(const QString &fileName, QString *foundSuffix) = 0;
    virtual QStringList parents(const QString &mime) = 0;
    virtual QString resolveAlias(const QString &name) = 0;
    virtual QStringList listAliases(const QString &name) = 0;
    virtual QMimeType findByMagic(const QByteArray &data, int *accuracyPtr) = 0;
    virtual QList<QMimeType> allMimeTypes() = 0;
    virtual void loadMimeTypePrivate(QMimeTypePrivate &) {}
    virtual void loadIcon(QMimeTypePrivate &) {}
    virtual void loadGenericIcon(QMimeTypePrivate &) {}

    QMimeDatabasePrivate *m_db;
protected:
    bool shouldCheck();
    QElapsedTimer m_lastCheck;
};

/*
   Parses the files 'mime.cache' and 'types' on demand
 */
class QMimeBinaryProvider : public QMimeProviderBase
{
public:
    QMimeBinaryProvider(QMimeDatabasePrivate *db);
    virtual ~QMimeBinaryProvider();

    virtual bool isValid() Q_DECL_OVERRIDE;
    virtual QMimeType mimeTypeForName(const QString &name) Q_DECL_OVERRIDE;
    virtual QStringList findByFileName(const QString &fileName, QString *foundSuffix) Q_DECL_OVERRIDE;
    virtual QStringList parents(const QString &mime) Q_DECL_OVERRIDE;
    virtual QString resolveAlias(const QString &name) Q_DECL_OVERRIDE;
    virtual QStringList listAliases(const QString &name) Q_DECL_OVERRIDE;
    virtual QMimeType findByMagic(const QByteArray &data, int *accuracyPtr) Q_DECL_OVERRIDE;
    virtual QList<QMimeType> allMimeTypes() Q_DECL_OVERRIDE;
    virtual void loadMimeTypePrivate(QMimeTypePrivate &) Q_DECL_OVERRIDE;
    virtual void loadIcon(QMimeTypePrivate &) Q_DECL_OVERRIDE;
    virtual void loadGenericIcon(QMimeTypePrivate &) Q_DECL_OVERRIDE;

private:
    struct CacheFile;

    void matchGlobList(QMimeGlobMatchResult &result, CacheFile *cacheFile, int offset, const QString &fileName);
    bool matchSuffixTree(QMimeGlobMatchResult &result, CacheFile *cacheFile, int numEntries, int firstOffset, const QString &fileName, int charPos, bool caseSensitiveCheck);
    bool matchMagicRule(CacheFile *cacheFile, int numMatchlets, int firstOffset, const QByteArray &data);
    QString iconForMime(CacheFile *cacheFile, int posListOffset, const QByteArray &inputMime);
    void loadMimeTypeList();
    void checkCache();

    class CacheFileList : public QList<CacheFile *>
    {
    public:
        CacheFile *findCacheFile(const QString &fileName) const;
        bool checkCacheChanged();
    };
    CacheFileList m_cacheFiles;
    QStringList m_cacheFileNames;
    QSet<QString> m_mimetypeNames;
    bool m_mimetypeListLoaded;
};

/*
   Parses the raw XML files (slower)
 */
class QMimeXMLProvider : public QMimeProviderBase
{
public:
    QMimeXMLProvider(QMimeDatabasePrivate *db);

    virtual bool isValid() Q_DECL_OVERRIDE;
    virtual QMimeType mimeTypeForName(const QString &name) Q_DECL_OVERRIDE;
    virtual QStringList findByFileName(const QString &fileName, QString *foundSuffix) Q_DECL_OVERRIDE;
    virtual QStringList parents(const QString &mime) Q_DECL_OVERRIDE;
    virtual QString resolveAlias(const QString &name) Q_DECL_OVERRIDE;
    virtual QStringList listAliases(const QString &name) Q_DECL_OVERRIDE;
    virtual QMimeType findByMagic(const QByteArray &data, int *accuracyPtr) Q_DECL_OVERRIDE;
    virtual QList<QMimeType> allMimeTypes() Q_DECL_OVERRIDE;

    bool load(const QString &fileName, QString *errorMessage);

    // Called by the mimetype xml parser
    void addMimeType(const QMimeType &mt);
    void addGlobPattern(const QMimeGlobPattern &glob);
    void addParent(const QString &child, const QString &parent);
    void addAlias(const QString &alias, const QString &name);
    void addMagicMatcher(const QMimeMagicRuleMatcher &matcher);

private:
    void ensureLoaded();
    void load(const QString &fileName);

    bool m_loaded;

    typedef QHash<QString, QMimeType> NameMimeTypeMap;
    NameMimeTypeMap m_nameMimeTypeMap;

    typedef QHash<QString, QString> AliasHash;
    AliasHash m_aliases;

    typedef QHash<QString, QStringList> ParentsHash;
    ParentsHash m_parents;
    QMimeAllGlobPatterns m_mimeTypeGlobs;

    QList<QMimeMagicRuleMatcher> m_magicMatchers;
    QStringList m_allFiles;
};

QT_END_NAMESPACE

#endif // QT_NO_MIMETYPE
#endif // QMIMEPROVIDER_P_H
