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

#ifndef QREADER_P_H
#define QREADER_P_H

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
#include <QtCore/QSharedDataPointer>
#include <QtCore/QSharedData>

CL_NS_DEF(util)
    class Reader;
CL_NS_END
CL_NS_USE(util)

QT_BEGIN_NAMESPACE

class QCLuceneField;
class QCLuceneAnalyzer;
class QCLuceneDocument;
class QCLuceneQueryParser;
class QCLuceneStandardTokenizer;

class Q_CLUCENE_EXPORT QCLuceneReaderPrivate : public QSharedData
{
public:
    QCLuceneReaderPrivate();
    QCLuceneReaderPrivate(const QCLuceneReaderPrivate &other);

    ~QCLuceneReaderPrivate();

    Reader* reader;
    bool deleteCLuceneReader;

private:
    QCLuceneReaderPrivate &operator=(const QCLuceneReaderPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneReader
{
public:
    QCLuceneReader();
    virtual ~QCLuceneReader();

protected:
    friend class QCLuceneField;
    friend class QCLuceneAnalyzer;
    friend class QCLuceneDocument;
    friend class QCLuceneQueryParser;
    friend class QCLuceneStandardTokenizer;
    QSharedDataPointer<QCLuceneReaderPrivate> d;
};

class QCLuceneStringReader : public QCLuceneReader
{
public:
    QCLuceneStringReader(const QString &value);
    QCLuceneStringReader(const QString &value, qint32 length);
    QCLuceneStringReader(const QString &value, qint32 length, bool copyData);

    ~QCLuceneStringReader();

private:
    TCHAR *string;
};

class Q_CLUCENE_EXPORT QCLuceneFileReader : public QCLuceneReader
{
public:
    QCLuceneFileReader(const QString &path, const QString &encoding,
                       qint32 cacheLength = 13, qint32 cacheBuffer = 14);
    ~QCLuceneFileReader();
};

QT_END_NAMESPACE

#endif  // QREADER_P_H
