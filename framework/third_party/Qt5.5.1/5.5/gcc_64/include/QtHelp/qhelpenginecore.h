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

#ifndef QHELPENGINECORE_H
#define QHELPENGINECORE_H

#include <QtHelp/qhelp_global.h>

#include <QtCore/QUrl>
#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QVariant>

QT_BEGIN_NAMESPACE


class QHelpEngineCorePrivate;

class QHELP_EXPORT QHelpEngineCore : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool autoSaveFilter READ autoSaveFilter WRITE setAutoSaveFilter)
    Q_PROPERTY(QString collectionFile READ collectionFile WRITE setCollectionFile)
    Q_PROPERTY(QString currentFilter READ currentFilter WRITE setCurrentFilter)

public:
    explicit QHelpEngineCore(const QString &collectionFile, QObject *parent = 0);
    virtual ~QHelpEngineCore();

    bool setupData();

    QString collectionFile() const;
    void setCollectionFile(const QString &fileName);

    bool copyCollectionFile(const QString &fileName);

    static QString namespaceName(const QString &documentationFileName);
    bool registerDocumentation(const QString &documentationFileName);
    bool unregisterDocumentation(const QString &namespaceName);
    QString documentationFileName(const QString &namespaceName);

    QStringList customFilters() const;
    bool removeCustomFilter(const QString &filterName);
    bool addCustomFilter(const QString &filterName,
        const QStringList &attributes);

    QStringList filterAttributes() const;
    QStringList filterAttributes(const QString &filterName) const;

    QString currentFilter() const;
    void setCurrentFilter(const QString &filterName);

    QStringList registeredDocumentations() const;
    QList<QStringList> filterAttributeSets(const QString &namespaceName) const;
    QList<QUrl> files(const QString namespaceName,
        const QStringList &filterAttributes,
        const QString &extensionFilter = QString());
    QUrl findFile(const QUrl &url) const;
    QByteArray fileData(const QUrl &url) const;

    QMap<QString, QUrl> linksForIdentifier(const QString &id) const;

    bool removeCustomValue(const QString &key);
    QVariant customValue(const QString &key,
        const QVariant &defaultValue = QVariant()) const;
    bool setCustomValue(const QString &key, const QVariant &value);

    static QVariant metaData(const QString &documentationFileName,
        const QString &name);

    QString error() const;

    void setAutoSaveFilter(bool save);
    bool autoSaveFilter() const;

Q_SIGNALS:
    void setupStarted();
    void setupFinished();
    void currentFilterChanged(const QString &newFilter);
    void warning(const QString &msg);
    void readersAboutToBeInvalidated();

protected:
    QHelpEngineCore(QHelpEngineCorePrivate *helpEngineCorePrivate,
        QObject *parent);

private:
    QHelpEngineCorePrivate *d;
    friend class QHelpEngineCorePrivate;
};

QT_END_NAMESPACE

#endif // QHELPENGINECORE_H
