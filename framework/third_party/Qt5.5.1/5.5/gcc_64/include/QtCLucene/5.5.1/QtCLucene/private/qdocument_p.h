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

#ifndef QDOCUMENT_P_H
#define QDOCUMENT_P_H

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

#include "qfield_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(document)
    class Document;
CL_NS_END
CL_NS_USE(document)

QT_BEGIN_NAMESPACE

class QCLuceneHits;
class QCLuceneIndexReader;
class QCLuceneIndexWriter;
class QCLuceneIndexSearcher;
class QCLuceneMultiSearcher;

class Q_CLUCENE_EXPORT QCLuceneDocumentPrivate : public QSharedData
{
public:
    QCLuceneDocumentPrivate();
    QCLuceneDocumentPrivate(const QCLuceneDocumentPrivate &other);

    ~QCLuceneDocumentPrivate();

    Document *document;
    bool deleteCLuceneDocument;

private:
    QCLuceneDocumentPrivate &operator=(const QCLuceneDocumentPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneDocument
{
public:
    QCLuceneDocument();
    ~QCLuceneDocument();

    void add(QCLuceneField *field);
    QCLuceneField* getField(const QString &name) const;
    QString get(const QString &name) const;
    QString toString() const;
    void setBoost(qreal boost);
    qreal getBoost() const;
    void removeField(const QString &name);
    void removeFields(const QString &name);
    QStringList getValues(const QString &name) const;
    void clear();

protected:
    friend class QCLuceneHits;
    friend class QCLuceneIndexReader;
    friend class QCLuceneIndexWriter;
    friend class QCLuceneIndexSearcher;
    friend class QCLuceneMultiSearcher;
    QSharedDataPointer<QCLuceneDocumentPrivate> d;

private:
    mutable QList<QCLuceneField*> fieldList;
};

QT_END_NAMESPACE

#endif  // QDOCUMENT_P_H
