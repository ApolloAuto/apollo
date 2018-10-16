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

#ifndef QHELPSEARCHINDEXREADERCLUCENE_H
#define QHELPSEARCHINDEXREADERCLUCENE_H

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

#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QStringList>

#include "private/qanalyzer_p.h"
#include "private/qquery_p.h"
#include "qhelpsearchindexreader_p.h"

QT_BEGIN_NAMESPACE

namespace fulltextsearch {
namespace clucene {

class QHelpSearchIndexReaderClucene : public QHelpSearchIndexReader
{
    Q_OBJECT

public:
    QHelpSearchIndexReaderClucene();
    ~QHelpSearchIndexReaderClucene();

private:
    void run();
    void boostSearchHits(const QHelpEngineCore &engine, QList<QHelpSearchEngine::SearchHit> &hitList,
        const QList<QHelpSearchQuery> &queryList);
    bool buildQuery(const QList<QHelpSearchQuery> &queries,
                    const QString &fieldName,
                    const QStringList &filterAttributes,
                    QCLuceneBooleanQuery &booleanQuery,
                    QCLuceneAnalyzer &analyzer);
    bool buildTryHarderQuery(const QList<QHelpSearchQuery> &queries,
                             const QString &fieldName,
                             const QStringList &filterAttributes,
                             QCLuceneBooleanQuery &booleanQuery,
                             QCLuceneAnalyzer &analyzer);
    bool addFuzzyQuery(const QHelpSearchQuery &query, const QString &fieldName,
                       QCLuceneBooleanQuery &booleanQuery, QCLuceneAnalyzer &analyzer);
    bool addWithoutQuery(const QHelpSearchQuery &query, const QString &fieldName,
                         QCLuceneBooleanQuery &booleanQuery);
    bool addPhraseQuery(const QHelpSearchQuery &query, const QString &fieldName,
                        QCLuceneBooleanQuery &booleanQuery);
    bool addAllQuery(const QHelpSearchQuery &query, const QString &fieldName,
                     QCLuceneBooleanQuery &booleanQuery);
    bool addDefaultQuery(const QHelpSearchQuery &query, const QString &fieldName,
                         bool allTermsRequired, QCLuceneBooleanQuery &booleanQuery,
                         QCLuceneAnalyzer &analyzer);
    bool addAtLeastQuery(const QHelpSearchQuery &query, const QString &fieldName,
                         QCLuceneBooleanQuery &booleanQuery, QCLuceneAnalyzer &analyzer);
    bool addAttributesQuery(const QStringList &filterAttributes,
               QCLuceneBooleanQuery &booleanQuery, QCLuceneAnalyzer &analyzer);
    bool isNegativeQuery(const QHelpSearchQuery &query) const;
};

}   // namespace clucene
}   // namespace fulltextsearch

QT_END_NAMESPACE

#endif  // QHELPSEARCHINDEXREADERCLUCENE_H
