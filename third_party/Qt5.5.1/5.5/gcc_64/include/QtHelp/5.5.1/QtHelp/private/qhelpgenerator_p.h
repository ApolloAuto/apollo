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

#ifndef QHELPGENERATOR_H
#define QHELPGENERATOR_H

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

#include "qhelp_global.h"
#include "qhelpdatainterface_p.h"

#include <QtCore/QObject>

QT_BEGIN_NAMESPACE

class QHelpGeneratorPrivate;

class QHELP_EXPORT QHelpGenerator : public QObject
{
    Q_OBJECT

public:
    QHelpGenerator(QObject *parent = 0);
    ~QHelpGenerator();

    bool generate(QHelpDataInterface *helpData,
        const QString &outputFileName);
    bool checkLinks(const QHelpDataInterface &helpData);
    QString error() const;

Q_SIGNALS:
    void statusChanged(const QString &msg);
    void progressChanged(double progress);
    void warning(const QString &msg);

private:
    struct FileNameTableData
    {
        QString name;
        int fileId;
        QString title;
    };

    void writeTree(QDataStream &s, QHelpDataContentItem *item, int depth);
    bool createTables();
    bool insertFileNotFoundFile();
    bool registerCustomFilter(const QString &filterName,
        const QStringList &filterAttribs, bool forceUpdate = false);
    bool registerVirtualFolder(const QString &folderName, const QString &ns);
    bool insertFilterAttributes(const QStringList &attributes);
    bool insertKeywords(const QList<QHelpDataIndexItem> &keywords,
        const QStringList &filterAttributes);
    bool insertFiles(const QStringList &files, const QString &rootPath,
        const QStringList &filterAttributes);
    bool insertContents(const QByteArray &ba,
        const QStringList &filterAttributes);
    bool insertMetaData(const QMap<QString, QVariant> &metaData);
    void cleanupDB();
    void setupProgress(QHelpDataInterface *helpData);
    void addProgress(double step);

    QHelpGeneratorPrivate *d;
};

QT_END_NAMESPACE

#endif
