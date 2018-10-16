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

#ifndef QHITS_P_H
#define QHITS_P_H

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

#include "qsort_p.h"
#include "qquery_p.h"
#include "qfilter_p.h"
#include "qdocument_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(search)
    class Hits;
CL_NS_END
CL_NS_USE(search)

QT_BEGIN_NAMESPACE

class QCLuceneSearcher;

class Q_CLUCENE_EXPORT QCLuceneHitsPrivate : public QSharedData
{
public:
    QCLuceneHitsPrivate();
    QCLuceneHitsPrivate(const QCLuceneHitsPrivate &other);

    ~QCLuceneHitsPrivate();

    Hits *hits;
    bool deleteCLuceneHits;

private:
    QCLuceneHitsPrivate &operator=(const QCLuceneHitsPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneHits
{
public:
    QCLuceneHits(const QCLuceneSearcher &searcher, const QCLuceneQuery &query,
        const QCLuceneFilter &filter);
    QCLuceneHits(const QCLuceneSearcher &searcher, const QCLuceneQuery &query,
        const QCLuceneFilter &filter, const QCLuceneSort &sort);
    virtual ~QCLuceneHits();

    QCLuceneDocument document(const qint32 index);
    qint32 length() const;
    qint32 id(const qint32 index);
    qreal score(const qint32 index);

protected:
    friend class QCLuceneSearcher;
    QSharedDataPointer<QCLuceneHitsPrivate> d;
};

QT_END_NAMESPACE

#endif  // QHITS_P_H
