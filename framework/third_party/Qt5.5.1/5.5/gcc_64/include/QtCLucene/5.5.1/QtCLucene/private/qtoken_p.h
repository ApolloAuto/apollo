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

#ifndef QTOKEN_P_H
#define QTOKEN_P_H

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

CL_NS_DEF(analysis)
    class Token;
CL_NS_END
CL_NS_USE(analysis)

QT_BEGIN_NAMESPACE

class QCLuceneTokenizer;
class QCLuceneTokenStream;
class QCLuceneStandardTokenizer;

class Q_CLUCENE_EXPORT QCLuceneTokenPrivate : public QSharedData
{
public:
    QCLuceneTokenPrivate();
    QCLuceneTokenPrivate(const QCLuceneTokenPrivate &other);

    ~QCLuceneTokenPrivate();

    Token *token;
    bool deleteCLuceneToken;

private:
    QCLuceneTokenPrivate &operator=(const QCLuceneTokenPrivate &other);
};

class Q_CLUCENE_EXPORT QCLuceneToken
{
public:
    QCLuceneToken();
    QCLuceneToken(const QString &text, qint32 startOffset,
                  qint32 endOffset, const QString &defaultTyp = QLatin1String("word"));

    virtual ~QCLuceneToken();

    void set(const QString &text, qint32 startOffset,
             qint32 endOffset, const QString &defaultTyp = QLatin1String("word"));

    quint32 bufferLength() const;
    void growBuffer(quint32 size);

    qint32 positionIncrement() const;
    void setPositionIncrement(qint32 positionIncrement);

    QString termText() const;
    void setTermText(const QString &text);

    quint32 termTextLength() const;
    void resetTermTextLength() const;

    qint32 startOffset() const;
    void setStartOffset(qint32 value);

    qint32 endOffset() const;
    void setEndOffset(qint32 value);

    QString type() const;
    void setType(const QString &type);

    QString toString() const;

protected:
    friend class QCLuceneTokenizer;
    friend class QCLuceneTokenStream;
    friend class QCLuceneStandardTokenizer;
    QSharedDataPointer<QCLuceneTokenPrivate> d;

private:
    TCHAR *tokenText;
    TCHAR *tokenType;
};

QT_END_NAMESPACE

#endif  // QTOKEN_P_H
