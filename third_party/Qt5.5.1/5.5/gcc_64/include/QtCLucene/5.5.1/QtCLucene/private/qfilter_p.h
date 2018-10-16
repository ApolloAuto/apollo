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

#ifndef QFilter_P_H
#define QFilter_P_H

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

#include <QtCore/QSharedData>
#include <QtCore/QSharedDataPointer>

CL_NS_DEF(search)
    class Filter;
CL_NS_END
CL_NS_USE(search)

QT_BEGIN_NAMESPACE

class QCLuceneHits;
class QCLuceneSearcher;

class Q_CLUCENE_EXPORT QCLuceneFilterPrivate : public QSharedData
{
public:
    QCLuceneFilterPrivate();
    QCLuceneFilterPrivate(const QCLuceneFilterPrivate &other);

    ~QCLuceneFilterPrivate ();

    Filter *filter;
    bool deleteCLuceneFilter;

private:
    QCLuceneFilterPrivate &operator=(const QCLuceneFilterPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneFilter
{
    QCLuceneFilter();
    virtual ~QCLuceneFilter();

protected:
    friend class QCLuceneHits;
    friend class QCLuceneSearcher;
    QSharedDataPointer<QCLuceneFilterPrivate> d;
};

QT_END_NAMESPACE

#endif  // QFilter_P_H
