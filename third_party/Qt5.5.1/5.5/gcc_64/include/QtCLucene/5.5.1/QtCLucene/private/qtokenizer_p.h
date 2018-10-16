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

#ifndef QTOKENIZER_P_H
#define QTOKENIZER_P_H

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
#include "qreader_p.h"
#include "qtokenstream_p.h"
#include "qclucene_global_p.h"

#include <QtCore/QChar>
#include <QtCore/QString>

QT_BEGIN_NAMESPACE

class Q_CLUCENE_EXPORT QCLuceneTokenizer : public QCLuceneTokenStream
{
public:
    QCLuceneTokenizer(const QCLuceneReader &reader);
    virtual ~QCLuceneTokenizer();

    void close();
    bool next(QCLuceneToken &token);

protected:
    friend class QCLuceneStandardTokenizer;

private:
    QCLuceneTokenizer();
    QCLuceneReader reader;
};

class Q_CLUCENE_EXPORT QCLuceneStandardTokenizer : public QCLuceneTokenizer
{
public:
    QCLuceneStandardTokenizer(const QCLuceneReader &reader);
    ~QCLuceneStandardTokenizer();

    bool readApostrophe(const QString &string, QCLuceneToken &token);
    bool readAt(const QString &string, QCLuceneToken &token);
    bool readCompany(const QString &string, QCLuceneToken &token);
};

class QCLuceneCharTokenizer : public QCLuceneTokenizer
{

};

class QCLuceneLetterTokenizer : public QCLuceneCharTokenizer
{

};

class QCLuceneLowerCaseTokenizer : public QCLuceneLetterTokenizer
{

};

class QCLuceneWhitespaceTokenizer : public QCLuceneCharTokenizer
{

};

class QCLuceneKeywordTokenizer : public QCLuceneTokenizer
{

};

QT_END_NAMESPACE

#endif  // QTOKENIZER_P_H
