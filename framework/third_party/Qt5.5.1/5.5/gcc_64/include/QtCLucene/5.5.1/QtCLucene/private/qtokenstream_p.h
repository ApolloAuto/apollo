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

#ifndef QTOKENSTREAM_P_H
#define QTOKENSTREAM_P_H

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

#include "qtoken_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QString>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(analysis)
    class TokenStream;
CL_NS_END
CL_NS_USE(analysis)

QT_BEGIN_NAMESPACE

class QCLuceneAnalyzer;
class QCLuceneTokenizer;
class QCLuceneStopAnalyzer;
class QCLuceneSimpleAnalyzer;
class QCLuceneKeywordAnalyzer;
class QCLuceneStandardAnalyzer;
class QCLuceneWhitespaceAnalyzer;
class QCLucenePerFieldAnalyzerWrapper;

class Q_CLUCENE_EXPORT QCLuceneTokenStreamPrivate : public QSharedData
{
public:
    QCLuceneTokenStreamPrivate();
    QCLuceneTokenStreamPrivate(const QCLuceneTokenStreamPrivate &other);

    ~QCLuceneTokenStreamPrivate();

    TokenStream *tokenStream;
    bool deleteCLuceneTokenStream;

private:
    QCLuceneTokenStreamPrivate &operator=(const QCLuceneTokenStreamPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneTokenStream
{
public:
    virtual ~QCLuceneTokenStream();

    void close();
    bool next(QCLuceneToken &token);

protected:
    friend class QCLuceneAnalyzer;
    friend class QCLuceneTokenizer;
    friend class QCLuceneStopAnalyzer;
    friend class QCLuceneSimpleAnalyzer;
    friend class QCLuceneKeywordAnalyzer;
    friend class QCLuceneStandardAnalyzer;
    friend class QCLuceneWhitespaceAnalyzer;
    friend class QCLucenePerFieldAnalyzerWrapper;
    QSharedDataPointer<QCLuceneTokenStreamPrivate> d;

private:
    QCLuceneTokenStream();
};

QT_END_NAMESPACE

#endif  // QTOKENSTREAM_P_H
