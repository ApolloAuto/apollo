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

#ifndef QANALYZER_P_H
#define QANALYZER_P_H

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

#include "qreader_p.h"
#include "qtokenstream_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(analysis)
    class Analyzer;
CL_NS_END
CL_NS_USE(analysis)

QT_BEGIN_NAMESPACE

class QCLuceneIndexWriter;
class QCLuceneQueryParser;
class QCLuceneStopAnalyzer;
class QCLuceneSimpleAnalyzer;
class QCLuceneKeywordAnalyzer;
class QCLuceneStandardAnalyzer;
class QCLuceneWhitespaceAnalyzer;
class QCLucenePerFieldAnalyzerWrapper;

class Q_CLUCENE_EXPORT QCLuceneAnalyzerPrivate : public QSharedData
{
public:
    QCLuceneAnalyzerPrivate();
    QCLuceneAnalyzerPrivate(const QCLuceneAnalyzerPrivate &other);

    ~QCLuceneAnalyzerPrivate();

    Analyzer *analyzer;
    bool deleteCLuceneAnalyzer;

private:
    QCLuceneAnalyzerPrivate &operator=(const QCLuceneAnalyzerPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneAnalyzer
{
public:
    virtual ~QCLuceneAnalyzer();

    qint32 positionIncrementGap(const QString &fieldName) const;
    QCLuceneTokenStream tokenStream(const QString &fieldName,
                                    const QCLuceneReader &reader) const;

protected:
    friend class QCLuceneIndexWriter;
    friend class QCLuceneQueryParser;
    friend class QCLuceneStopAnalyzer;
    friend class QCLuceneSimpleAnalyzer;
    friend class QCLuceneKeywordAnalyzer;
    friend class QCLuceneStandardAnalyzer;
    friend class QCLuceneWhitespaceAnalyzer;
    friend class QCLucenePerFieldAnalyzerWrapper;
    QSharedDataPointer<QCLuceneAnalyzerPrivate> d;

private:
    QCLuceneAnalyzer();
};

class Q_CLUCENE_EXPORT QCLuceneStandardAnalyzer : public QCLuceneAnalyzer
{
public:
    QCLuceneStandardAnalyzer();
    QCLuceneStandardAnalyzer(const QStringList &stopWords);

    ~QCLuceneStandardAnalyzer();
};

class Q_CLUCENE_EXPORT QCLuceneWhitespaceAnalyzer : public QCLuceneAnalyzer
{
public:
    QCLuceneWhitespaceAnalyzer();
    ~QCLuceneWhitespaceAnalyzer();
};

class Q_CLUCENE_EXPORT QCLuceneSimpleAnalyzer : public QCLuceneAnalyzer
{
public:
    QCLuceneSimpleAnalyzer();
    ~QCLuceneSimpleAnalyzer();
};

class Q_CLUCENE_EXPORT QCLuceneStopAnalyzer : public QCLuceneAnalyzer
{
public:
    QCLuceneStopAnalyzer();
    QCLuceneStopAnalyzer(const QStringList &stopWords);

    ~QCLuceneStopAnalyzer();

    QStringList englishStopWords() const;
};

class Q_CLUCENE_EXPORT QCLuceneKeywordAnalyzer : public QCLuceneAnalyzer
{
public:
    QCLuceneKeywordAnalyzer();
    ~QCLuceneKeywordAnalyzer();
};

class Q_CLUCENE_EXPORT QCLucenePerFieldAnalyzerWrapper : public QCLuceneAnalyzer
{
public:
    QCLucenePerFieldAnalyzerWrapper(QCLuceneAnalyzer *defaultAnalyzer);
    ~QCLucenePerFieldAnalyzerWrapper();

    void addAnalyzer(const QString &fieldName, QCLuceneAnalyzer *analyzer);

private:
    QList<QCLuceneAnalyzer*> analyzers;
};

QT_END_NAMESPACE

#endif  // QANALYZER_P_H
