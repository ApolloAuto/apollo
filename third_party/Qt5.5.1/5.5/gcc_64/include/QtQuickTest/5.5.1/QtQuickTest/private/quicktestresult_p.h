/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the test suite of the Qt Toolkit.
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

#ifndef QUICKTESTRESULT_P_H
#define QUICKTESTRESULT_P_H

#include <QtQuickTest/quicktestglobal.h>
#include <QtCore/qobject.h>
#include <QtCore/qstring.h>
#include <QtCore/qstringlist.h>
#include <QtCore/qscopedpointer.h>
#include <QtQuick/qquickitem.h>
#include <QtQml/private/qv8engine_p.h>

QT_BEGIN_NAMESPACE

class QUrl;
class QuickTestResultPrivate;

class Q_QUICK_TEST_EXPORT QuickTestResult : public QObject
{
    Q_OBJECT
    Q_ENUMS(RunMode)
    Q_PROPERTY(QString testCaseName READ testCaseName WRITE setTestCaseName NOTIFY testCaseNameChanged)
    Q_PROPERTY(QString functionName READ functionName WRITE setFunctionName NOTIFY functionNameChanged)
    Q_PROPERTY(QString dataTag READ dataTag WRITE setDataTag NOTIFY dataTagChanged)
    Q_PROPERTY(bool failed READ isFailed)
    Q_PROPERTY(bool skipped READ isSkipped WRITE setSkipped NOTIFY skippedChanged)
    Q_PROPERTY(int passCount READ passCount)
    Q_PROPERTY(int failCount READ failCount)
    Q_PROPERTY(int skipCount READ skipCount)
    Q_PROPERTY(QStringList functionsToRun READ functionsToRun)
public:
    QuickTestResult(QObject *parent = 0);
    ~QuickTestResult();

    // Values must match QBenchmarkIterationController::RunMode.
    enum RunMode
    {
        RepeatUntilValidMeasurement,
        RunOnce
    };

    QString testCaseName() const;
    void setTestCaseName(const QString &name);

    QString functionName() const;
    void setFunctionName(const QString &name);

    QString dataTag() const;
    void setDataTag(const QString &tag);

    bool isFailed() const;

    bool isSkipped() const;
    void setSkipped(bool skip);

    int passCount() const;
    int failCount() const;
    int skipCount() const;

    QStringList functionsToRun() const;

public Q_SLOTS:
    void reset();

    void startLogging();
    void stopLogging();

    void initTestTable();
    void clearTestTable();

    void finishTestData();
    void finishTestDataCleanup();
    void finishTestFunction();

    void stringify(QQmlV4Function *args);

    void fail(const QString &message, const QUrl &location, int line);
    bool verify(bool success, const QString &message,
                const QUrl &location, int line);
    bool compare(bool success, const QString &message,
                 const QVariant &val1, const QVariant &val2,
                 const QUrl &location, int line);
    bool fuzzyCompare(const QVariant &actual, const QVariant &expected, qreal delta);
    void skip(const QString &message, const QUrl &location, int line);
    bool expectFail(const QString &tag, const QString &comment,
                    const QUrl &location, int line);
    bool expectFailContinue(const QString &tag, const QString &comment,
                            const QUrl &location, int line);
    void warn(const QString &message, const QUrl &location, int line);

    void ignoreWarning(const QString &message);

    void wait(int ms);
    void sleep(int ms);
    bool waitForRendering(QQuickItem *item, int timeout = 5000);

    void startMeasurement();
    void beginDataRun();
    void endDataRun();
    bool measurementAccepted();
    bool needsMoreMeasurements();

    void startBenchmark(RunMode runMode, const QString &tag);
    bool isBenchmarkDone() const;
    void nextBenchmark();
    void stopBenchmark();

    QObject *grabImage(QQuickItem *item);

    Q_REVISION(1) QObject *findChild(QObject *parent, const QString &objectName);

public:
    // Helper functions for the C++ main() shell.
    static void parseArgs(int argc, char *argv[]);
    static void setProgramName(const char *name);
    static void setCurrentAppname(const char *appname);
    static int exitCode();

Q_SIGNALS:
    void programNameChanged();
    void testCaseNameChanged();
    void functionNameChanged();
    void dataTagChanged();
    void skippedChanged();

private:
    QScopedPointer<QuickTestResultPrivate> d_ptr;

    Q_DECLARE_PRIVATE(QuickTestResult)
    Q_DISABLE_COPY(QuickTestResult)
};

QT_END_NAMESPACE

#endif
