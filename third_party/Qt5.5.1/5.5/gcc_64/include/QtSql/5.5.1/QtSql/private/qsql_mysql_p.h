/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtSql module of the Qt Toolkit.
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

#ifndef QSQL_MYSQL_H
#define QSQL_MYSQL_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtSql/qsqldriver.h>
#include <QtSql/qsqlresult.h>

#if defined (Q_OS_WIN32)
#include <QtCore/qt_windows.h>
#endif

#include <mysql.h>

#ifdef QT_PLUGIN
#define Q_EXPORT_SQLDRIVER_MYSQL
#else
#define Q_EXPORT_SQLDRIVER_MYSQL Q_SQL_EXPORT
#endif

QT_BEGIN_NAMESPACE

class QMYSQLDriverPrivate;
class QMYSQLResultPrivate;
class QMYSQLDriver;
class QSqlRecordInfo;

class QMYSQLResult : public QSqlResult
{
    friend class QMYSQLDriver;
    friend class QMYSQLResultPrivate;
public:
    explicit QMYSQLResult(const QMYSQLDriver* db);
    ~QMYSQLResult();

    QVariant handle() const Q_DECL_OVERRIDE;
protected:
    void cleanup();
    bool fetch(int i) Q_DECL_OVERRIDE;
    bool fetchNext() Q_DECL_OVERRIDE;
    bool fetchLast() Q_DECL_OVERRIDE;
    bool fetchFirst() Q_DECL_OVERRIDE;
    QVariant data(int field) Q_DECL_OVERRIDE;
    bool isNull(int field) Q_DECL_OVERRIDE;
    bool reset (const QString& query) Q_DECL_OVERRIDE;
    int size() Q_DECL_OVERRIDE;
    int numRowsAffected() Q_DECL_OVERRIDE;
    QVariant lastInsertId() const Q_DECL_OVERRIDE;
    QSqlRecord record() const Q_DECL_OVERRIDE;
    void virtual_hook(int id, void *data) Q_DECL_OVERRIDE;
    bool nextResult() Q_DECL_OVERRIDE;

#if MYSQL_VERSION_ID >= 40108
    bool prepare(const QString& stmt) Q_DECL_OVERRIDE;
    bool exec() Q_DECL_OVERRIDE;
#endif
private:
    QMYSQLResultPrivate* d;
};

class Q_EXPORT_SQLDRIVER_MYSQL QMYSQLDriver : public QSqlDriver
{
    friend class QMYSQLResult;
    Q_DECLARE_PRIVATE(QMYSQLDriver)
    Q_OBJECT
public:
    explicit QMYSQLDriver(QObject *parent=0);
    explicit QMYSQLDriver(MYSQL *con, QObject * parent=0);
    ~QMYSQLDriver();
    bool hasFeature(DriverFeature f) const Q_DECL_OVERRIDE;
    bool open(const QString & db,
               const QString & user,
               const QString & password,
               const QString & host,
               int port,
               const QString& connOpts) Q_DECL_OVERRIDE;
    void close() Q_DECL_OVERRIDE;
    QSqlResult *createResult() const Q_DECL_OVERRIDE;
    QStringList tables(QSql::TableType) const Q_DECL_OVERRIDE;
    QSqlIndex primaryIndex(const QString& tablename) const Q_DECL_OVERRIDE;
    QSqlRecord record(const QString& tablename) const Q_DECL_OVERRIDE;
    QString formatValue(const QSqlField &field,
                                     bool trimStrings) const Q_DECL_OVERRIDE;
    QVariant handle() const Q_DECL_OVERRIDE;
    QString escapeIdentifier(const QString &identifier, IdentifierType type) const Q_DECL_OVERRIDE;

    bool isIdentifierEscaped(const QString &identifier, IdentifierType type) const Q_DECL_OVERRIDE;

protected:
    bool beginTransaction() Q_DECL_OVERRIDE;
    bool commitTransaction() Q_DECL_OVERRIDE;
    bool rollbackTransaction() Q_DECL_OVERRIDE;
private:
    void init();
};

QT_END_NAMESPACE

#endif // QSQL_MYSQL_H
