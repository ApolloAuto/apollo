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

#ifndef QSORT_P_H
#define QSORT_P_H

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

#include "qclucene_global_p.h"

#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(search)
    class Sort;
CL_NS_END
CL_NS_USE(search)

QT_BEGIN_NAMESPACE

class QCLuceneHits;
class QCLuceneField;

class Q_CLUCENE_EXPORT QCLuceneSortPrivate : public QSharedData
{
public:
    QCLuceneSortPrivate();
    QCLuceneSortPrivate (const QCLuceneSortPrivate &other);

    ~QCLuceneSortPrivate();

    Sort *sort;
    bool deleteCLuceneSort;

private:
    QCLuceneSortPrivate &operator=(const QCLuceneSortPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneSort
{
public:
    QCLuceneSort();
    explicit QCLuceneSort(const QStringList &fieldNames);
    explicit QCLuceneSort(const QString &field, bool reverse = false);

    virtual ~QCLuceneSort();

    QString toString() const;
    void setSort(const QStringList &fieldNames);
    void setSort(const QString &field, bool reverse = false);

protected:
    friend class QCLuceneHits;
    QSharedDataPointer<QCLuceneSortPrivate> d;
};

QT_END_NAMESPACE

#endif  // QSORT_P_H
