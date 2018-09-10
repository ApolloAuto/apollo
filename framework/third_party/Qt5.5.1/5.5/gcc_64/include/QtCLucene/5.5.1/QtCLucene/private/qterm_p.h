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

#ifndef QTERM_P_H
#define QTERM_P_H

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
#include <QtCore/QString>
#include <QtCore/QSharedDataPointer>

CL_NS_DEF(index)
    class Term;
CL_NS_END
CL_NS_USE(index)

QT_BEGIN_NAMESPACE

class QCLuceneTermQuery;
class QCLuceneRangeQuery;
class QCLucenePrefixQuery;
class QCLuceneIndexReader;
class QCLucenePhraseQuery;

class Q_CLUCENE_EXPORT QCLuceneTermPrivate : public QSharedData
{
public:
    QCLuceneTermPrivate();
    QCLuceneTermPrivate(const QCLuceneTermPrivate &other);

    ~QCLuceneTermPrivate();

    Term *term;
    bool deleteCLuceneTerm;

private:
    QCLuceneTermPrivate &operator=(const QCLuceneTermPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneTerm
{
public:
    QCLuceneTerm();
    QCLuceneTerm(const QString &field, const QString &text);
    QCLuceneTerm(const QCLuceneTerm &fieldTerm, const QString &text);

    virtual ~QCLuceneTerm();

    QString field() const;
    QString text() const;

    void set(const QString &field, const QString &text);
    void set(const QCLuceneTerm &fieldTerm, const QString &text);
    void set(const QString &field, const QString &text, bool internField);

    bool equals(const QCLuceneTerm &other) const;
    qint32 compareTo(const QCLuceneTerm &other) const;

    QString toString() const;
    quint32 hashCode() const;
    quint32 textLength() const;

protected:
    friend class QCLuceneTermQuery;
    friend class QCLuceneRangeQuery;
    friend class QCLucenePrefixQuery;
    friend class QCLuceneIndexReader;
    friend class QCLucenePhraseQuery;
    QSharedDataPointer<QCLuceneTermPrivate> d;
};

QT_END_NAMESPACE

#endif  // QTERM_P_H
