/****************************************************************************
**
** Copyright (C) 2003-2006 Ben van Klinken and the CLucene Team.
** All rights reserved.
**
** Portion Copyright (C) 2015 The Qt Company Ltd.
** All rights reserved.
**
** This file may be used under the terms of the GNU Lesser General Public
** License version 2.1 as published by the Free Software Foundation and
** appearing in the file LICENSE.LGPL included in the packaging of this file.
** Please review the following information to ensure the GNU Lesser General
** Public License version 2.1 requirements will be met:
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
****************************************************************************/

#ifndef QINDEXWRITER_P_H
#define QINDEXWRITER_P_H

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

#include "qanalyzer_p.h"
#include "qdocument_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QString>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(index)
    class IndexWriter;
CL_NS_END
CL_NS_USE(index)

QT_BEGIN_NAMESPACE

class QCLuceneIndexReader;

class Q_CLUCENE_EXPORT QCLuceneIndexWriterPrivate : public QSharedData
{
public:
    QCLuceneIndexWriterPrivate();
    QCLuceneIndexWriterPrivate(const QCLuceneIndexWriterPrivate &other);

    ~QCLuceneIndexWriterPrivate();

    IndexWriter *writer;
    bool deleteCLuceneIndexWriter;

private:
    QCLuceneIndexWriterPrivate &operator=(const QCLuceneIndexWriterPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneIndexWriter
{
public:
    enum {
        DEFAULT_MERGE_FACTOR = 10,
        COMMIT_LOCK_TIMEOUT = 10000,
        DEFAULT_MAX_BUFFERED_DOCS = 10,
        DEFAULT_MAX_FIELD_LENGTH = 10000,
        DEFAULT_TERM_INDEX_INTERVAL = 128,
        DEFAULT_MAX_MERGE_DOCS = 0x7FFFFFFFL
    };

    QCLuceneIndexWriter(const QString &path, QCLuceneAnalyzer &analyzer,
    bool create, bool closeDir = true);
    virtual ~QCLuceneIndexWriter();

    void close();
    void optimize();
    qint32 docCount();
    QCLuceneAnalyzer getAnalyzer();

    void addIndexes(const QList<QCLuceneIndexReader*> &readers);
    void addDocument(QCLuceneDocument &doc, QCLuceneAnalyzer &analyzer);

    qint32 getMaxFieldLength() const;
    void setMaxFieldLength(qint32 value);

    qint32 getMaxBufferedDocs() const;
    void setMaxBufferedDocs(qint32 value);

    qint64 getWriteLockTimeout() const;
    void setWriteLockTimeout(qint64 writeLockTimeout);

    qint64 getCommitLockTimeout() const;
    void setCommitLockTimeout(qint64 commitLockTimeout);

    qint32 getMergeFactor() const;
    void setMergeFactor(qint32 value);

    qint32 getTermIndexInterval() const;
    void setTermIndexInterval(qint32 interval);

    qint32 getMinMergeDocs() const;
    void setMinMergeDocs(qint32 value);

    qint32 getMaxMergeDocs() const;
    void setMaxMergeDocs(qint32 value);

    bool getUseCompoundFile() const;
    void setUseCompoundFile(bool value);

protected:
    QSharedDataPointer<QCLuceneIndexWriterPrivate> d;

private:
    QCLuceneAnalyzer analyzer;
};

QT_END_NAMESPACE

#endif  // QINDEXWRITER_P_H
