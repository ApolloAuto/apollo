/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQml module of the Qt Toolkit.
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

#ifndef QQMLTYPELOADER_P_H
#define QQMLTYPELOADER_P_H

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

#include <QtCore/qobject.h>
#include <QtCore/qatomic.h>
#include <QtNetwork/qnetworkreply.h>
#include <QtQml/qqmlerror.h>
#include <QtQml/qqmlengine.h>
#include <QtQml/qqmlfile.h>
#include <QtQml/qqmlabstracturlinterceptor.h>

#include <private/qhashedstring_p.h>
#include <private/qqmlimport_p.h>
#include <private/qqmlcleanup_p.h>
#include <private/qqmldirparser_p.h>
#include <private/qflagpointer_p.h>
#include <private/qqmlirbuilder_p.h>

#include <private/qv4value_inl_p.h>
#include <private/qv4script_p.h>

QT_BEGIN_NAMESPACE

class QQmlScriptData;
class QQmlScriptBlob;
class QQmlQmldirData;
class QQmlTypeLoader;
class QQmlCompiledData;
class QQmlComponentPrivate;
class QQmlTypeData;
class QQmlTypeLoader;
class QQmlExtensionInterface;

namespace QmlIR {
struct Document;
}

class Q_QML_PRIVATE_EXPORT QQmlDataBlob : public QQmlRefCount
{
public:
    enum Status {
        Null,                    // Prior to QQmlTypeLoader::load()
        Loading,                 // Prior to data being received and dataReceived() being called
        WaitingForDependencies,  // While there are outstanding addDependency()s
        Complete,                // Finished
        Error                    // Error
    };

    enum Type { //Matched in QQmlAbstractUrlInterceptor
        QmlFile = QQmlAbstractUrlInterceptor::QmlFile,
        JavaScriptFile = QQmlAbstractUrlInterceptor::JavaScriptFile,
        QmldirFile = QQmlAbstractUrlInterceptor::QmldirFile
    };

    QQmlDataBlob(const QUrl &, Type, QQmlTypeLoader* manager);
    virtual ~QQmlDataBlob();

    void startLoading();

    QQmlTypeLoader *typeLoader() const { return m_typeLoader; }

    Type type() const;

    Status status() const;
    bool isNull() const;
    bool isLoading() const;
    bool isWaiting() const;
    bool isComplete() const;
    bool isError() const;
    bool isCompleteOrError() const;

    qreal progress() const;

    QUrl url() const;
    QUrl finalUrl() const;
    QString finalUrlString() const;

    QList<QQmlError> errors() const;

    class Data {
    public:
        inline const char *data() const;
        inline int size() const;

        inline QByteArray asByteArray() const;

        inline bool isFile() const;
        inline QQmlFile *asFile() const;

    private:
        friend class QQmlDataBlob;
        friend class QQmlTypeLoader;
        inline Data();
        Data(const Data &);
        Data &operator=(const Data &);
        QBiPointer<const QByteArray, QQmlFile> d;
    };

protected:
    // Can be called from within callbacks
    void setError(const QQmlError &);
    void setError(const QList<QQmlError> &errors);
    void addDependency(QQmlDataBlob *);

    // Callbacks made in load thread
    virtual void dataReceived(const Data &) = 0;
    virtual void initializeFromCachedUnit(const QQmlPrivate::CachedQmlUnit*) = 0;
    virtual void done();
    virtual void networkError(QNetworkReply::NetworkError);
    virtual void dependencyError(QQmlDataBlob *);
    virtual void dependencyComplete(QQmlDataBlob *);
    virtual void allDependenciesDone();

    // Callbacks made in main thread
    virtual void downloadProgressChanged(qreal);
    virtual void completed();

protected:
    // Manager that is currently fetching data for me
    QQmlTypeLoader *m_typeLoader;

private:
    friend class QQmlTypeLoader;
    friend class QQmlTypeLoaderThread;

    void tryDone();
    void cancelAllWaitingFor();
    void notifyAllWaitingOnMe();
    void notifyComplete(QQmlDataBlob *);

    struct ThreadData {
        inline ThreadData();
        inline QQmlDataBlob::Status status() const;
        inline void setStatus(QQmlDataBlob::Status);
        inline bool isAsync() const;
        inline void setIsAsync(bool);
        inline quint8 progress() const;
        inline void setProgress(quint8);

    private:
        QAtomicInt _p;
    };
    ThreadData m_data;

    // m_errors should *always* be written before the status is set to Error.
    // We use the status change as a memory fence around m_errors so that locking
    // isn't required.  Once the status is set to Error (or Complete), m_errors
    // cannot be changed.
    QList<QQmlError> m_errors;

    Type m_type;

    QUrl m_url;
    QUrl m_finalUrl;
    mutable QString m_finalUrlString;

    // List of QQmlDataBlob's that are waiting for me to complete.
    QList<QQmlDataBlob *> m_waitingOnMe;

    // List of QQmlDataBlob's that I am waiting for to complete.
    QList<QQmlDataBlob *> m_waitingFor;

    int m_redirectCount:30;
    bool m_inCallback:1;
    bool m_isDone:1;
};

class QQmlTypeLoaderThread;

class Q_AUTOTEST_EXPORT QQmlTypeLoader
{
    Q_DECLARE_TR_FUNCTIONS(QQmlTypeLoader)
public:
    enum Mode { PreferSynchronous, Asynchronous };

    class Q_QML_PRIVATE_EXPORT Blob : public QQmlDataBlob
    {
    public:
        Blob(const QUrl &url, QQmlDataBlob::Type type, QQmlTypeLoader *loader);
        ~Blob();

        const QQmlImports &imports() const { return m_importCache; }

    protected:
        bool addImport(const QV4::CompiledData::Import *import, QList<QQmlError> *errors);
        bool addPragma(const QmlIR::Pragma &pragma, QList<QQmlError> *errors);

        bool fetchQmldir(const QUrl &url, const QV4::CompiledData::Import *import, int priority, QList<QQmlError> *errors);
        bool updateQmldir(QQmlQmldirData *data, const QV4::CompiledData::Import *import, QList<QQmlError> *errors);

    private:
        virtual bool qmldirDataAvailable(QQmlQmldirData *, QList<QQmlError> *);

        virtual void scriptImported(QQmlScriptBlob *, const QV4::CompiledData::Location &, const QString &, const QString &) {}

        virtual void dependencyError(QQmlDataBlob *);
        virtual void dependencyComplete(QQmlDataBlob *);

    protected:
        virtual QString stringAt(int) const { return QString(); }

        QQmlImports m_importCache;
        bool m_isSingleton;
        QHash<const QV4::CompiledData::Import*, int> m_unresolvedImports;
        QList<QQmlQmldirData *> m_qmldirs;
    };

    class QmldirContent
    {
    private:
        friend class QQmlTypeLoader;
        QmldirContent();

        void setContent(const QString &location, const QString &content);
        void setError(const QQmlError &);

    public:
        bool hasError() const;
        QList<QQmlError> errors(const QString &uri) const;

        QString typeNamespace() const;

        QQmlDirComponents components() const;
        QQmlDirScripts scripts() const;
        QQmlDirPlugins plugins() const;

        QString pluginLocation() const;

        bool designerSupported() const;

    private:
        QQmlDirParser m_parser;
        QString m_location;
    };

    QQmlTypeLoader(QQmlEngine *);
    ~QQmlTypeLoader();

    QQmlImportDatabase *importDatabase();

    QQmlTypeData *getType(const QUrl &url, Mode mode = PreferSynchronous);
    QQmlTypeData *getType(const QByteArray &, const QUrl &url);

    QQmlScriptBlob *getScript(const QUrl &);
    QQmlQmldirData *getQmldir(const QUrl &);

    QString absoluteFilePath(const QString &path);
    bool directoryExists(const QString &path);

    const QmldirContent *qmldirContent(const QString &filePath);
    void setQmldirContent(const QString &filePath, const QString &content);

    void clearCache();
    void trimCache();

    bool isTypeLoaded(const QUrl &url) const;
    bool isScriptLoaded(const QUrl &url) const;

    void lock();
    void unlock();

    void load(QQmlDataBlob *, Mode = PreferSynchronous);
    void loadWithStaticData(QQmlDataBlob *, const QByteArray &, Mode = PreferSynchronous);
    void loadWithCachedUnit(QQmlDataBlob *blob, const QQmlPrivate::CachedQmlUnit *unit, Mode mode = PreferSynchronous);

    QQmlEngine *engine() const;
    void initializeEngine(QQmlExtensionInterface *, const char *);
    void invalidate();

private:
    friend class QQmlDataBlob;
    friend class QQmlTypeLoaderThread;
    friend class QQmlTypeLoaderNetworkReplyProxy;

    void shutdownThread();

    void loadThread(QQmlDataBlob *);
    void loadWithStaticDataThread(QQmlDataBlob *, const QByteArray &);
    void loadWithCachedUnitThread(QQmlDataBlob *blob, const QQmlPrivate::CachedQmlUnit *unit);
    void networkReplyFinished(QNetworkReply *);
    void networkReplyProgress(QNetworkReply *, qint64, qint64);

    typedef QHash<QNetworkReply *, QQmlDataBlob *> NetworkReplies;

    void setData(QQmlDataBlob *, const QByteArray &);
    void setData(QQmlDataBlob *, QQmlFile *);
    void setData(QQmlDataBlob *, const QQmlDataBlob::Data &);
    void setCachedUnit(QQmlDataBlob *blob, const QQmlPrivate::CachedQmlUnit *unit);

    template<typename T>
    struct TypedCallback
    {
        TypedCallback(T *object, void (T::*func)(QQmlTypeData *)) : o(object), mf(func) {}

        static void redirect(void *arg, QQmlTypeData *type)
        {
            TypedCallback<T> *self = reinterpret_cast<TypedCallback<T> *>(arg);
            ((self->o)->*(self->mf))(type);
        }

    private:
        T *o;
        void (T::*mf)(QQmlTypeData *);
    };

    typedef QHash<QUrl, QQmlTypeData *> TypeCache;
    typedef QHash<QUrl, QQmlScriptBlob *> ScriptCache;
    typedef QHash<QUrl, QQmlQmldirData *> QmldirCache;
    typedef QStringHash<bool> StringSet;
    typedef QStringHash<StringSet*> ImportDirCache;
    typedef QStringHash<QmldirContent *> ImportQmlDirCache;

    QQmlEngine *m_engine;
    QQmlTypeLoaderThread *m_thread;
    NetworkReplies m_networkReplies;
    TypeCache m_typeCache;
    ScriptCache m_scriptCache;
    QmldirCache m_qmldirCache;
    ImportDirCache m_importDirCache;
    ImportQmlDirCache m_importQmlDirCache;
};

class Q_AUTOTEST_EXPORT QQmlTypeData : public QQmlTypeLoader::Blob
{
public:
    struct TypeReference
    {
        TypeReference() : type(0), majorVersion(0), minorVersion(0), typeData(0), needsCreation(true) {}

        QV4::CompiledData::Location location;
        QQmlType *type;
        int majorVersion;
        int minorVersion;
        QQmlTypeData *typeData;
        QString prefix; // used by CompositeSingleton types
        bool needsCreation;
    };

    struct ScriptReference
    {
        ScriptReference() : script(0) {}

        QV4::CompiledData::Location location;
        QString qualifier;
        QQmlScriptBlob *script;
    };

private:
    friend class QQmlTypeLoader;

    QQmlTypeData(const QUrl &, QQmlTypeLoader *);

public:
    ~QQmlTypeData();

    const QHash<int, TypeReference> &resolvedTypeRefs() const { return m_resolvedTypes; }

    const QList<ScriptReference> &resolvedScripts() const;
    const QSet<QString> &namespaces() const;
    const QList<TypeReference> &compositeSingletons() const;

    QQmlCompiledData *compiledData() const;

    // Used by QQmlComponent to get notifications
    struct TypeDataCallback {
        virtual ~TypeDataCallback();
        virtual void typeDataProgress(QQmlTypeData *, qreal) {}
        virtual void typeDataReady(QQmlTypeData *) {}
    };
    void registerCallback(TypeDataCallback *);
    void unregisterCallback(TypeDataCallback *);

protected:
    virtual void done();
    virtual void completed();
    virtual void dataReceived(const Data &);
    virtual void initializeFromCachedUnit(const QQmlPrivate::CachedQmlUnit *unit);
    virtual void allDependenciesDone();
    virtual void downloadProgressChanged(qreal);

    virtual QString stringAt(int index) const;

private:
    void continueLoadFromIR();
    void resolveTypes();
    void compile();
    bool resolveType(const QString &typeName, int &majorVersion, int &minorVersion, TypeReference &ref);

    virtual void scriptImported(QQmlScriptBlob *blob, const QV4::CompiledData::Location &location, const QString &qualifier, const QString &nameSpace);

    QScopedPointer<QmlIR::Document> m_document;

    QList<ScriptReference> m_scripts;

    QSet<QString> m_namespaces;
    QList<TypeReference> m_compositeSingletons;

    // map from name index to resolved type
    QHash<int, TypeReference> m_resolvedTypes;
    bool m_typesResolved:1;

    QQmlCompiledData *m_compiledData;

    QList<TypeDataCallback *> m_callbacks;

    QV4::CompiledData::Import *m_implicitImport;
    bool m_implicitImportLoaded;
    bool loadImplicitImport();
};

// QQmlScriptData instances are created, uninitialized, by the loader in the
// load thread.  The first time they are used by the VME, they are initialized which
// creates their v8 objects and they are referenced and added to the  engine's cleanup
// list.  During QQmlCleanup::clear() all v8 resources are destroyed, and the
// reference that was created is released but final deletion only occurs once all the
// references as released.  This is all intended to ensure that the v8 resources are
// only created and destroyed in the main thread :)
class Q_AUTOTEST_EXPORT QQmlScriptData : public QQmlCleanup, public QQmlRefCount
{
private:
    friend class QQmlTypeLoader;

    QQmlScriptData();

public:
    ~QQmlScriptData();

    QUrl url;
    QString urlString;
    QQmlTypeNameCache *importCache;
    QList<QQmlScriptBlob *> scripts;

    QV4::PersistentValue scriptValueForContext(QQmlContextData *parentCtxt);

protected:
    virtual void clear(); // From QQmlCleanup

private:
    friend class QQmlScriptBlob;

    void initialize(QQmlEngine *);

    bool m_loaded;
    QQmlRefPointer<QV4::CompiledData::CompilationUnit> m_precompiledScript;
    QV4::Script *m_program;
    QV4::PersistentValue m_value;
};

class Q_AUTOTEST_EXPORT QQmlScriptBlob : public QQmlTypeLoader::Blob
{
private:
    friend class QQmlTypeLoader;

    QQmlScriptBlob(const QUrl &, QQmlTypeLoader *);

public:
    ~QQmlScriptBlob();

    struct ScriptReference
    {
        ScriptReference() : script(0) {}

        QV4::CompiledData::Location location;
        QString qualifier;
        QString nameSpace;
        QQmlScriptBlob *script;
    };

    QQmlScriptData *scriptData() const;

protected:
    virtual void dataReceived(const Data &);
    virtual void initializeFromCachedUnit(const QQmlPrivate::CachedQmlUnit *unit);
    virtual void done();

    virtual QString stringAt(int index) const;

private:
    virtual void scriptImported(QQmlScriptBlob *blob, const QV4::CompiledData::Location &location, const QString &qualifier, const QString &nameSpace);
    void initializeFromCompilationUnit(QV4::CompiledData::CompilationUnit *unit);

    QList<ScriptReference> m_scripts;
    QQmlScriptData *m_scriptData;
};

class Q_AUTOTEST_EXPORT QQmlQmldirData : public QQmlTypeLoader::Blob
{
private:
    friend class QQmlTypeLoader;

    QQmlQmldirData(const QUrl &, QQmlTypeLoader *);

public:
    const QString &content() const;

    const QV4::CompiledData::Import *import() const;
    void setImport(const QV4::CompiledData::Import *);

    int priority() const;
    void setPriority(int);

protected:
    virtual void dataReceived(const Data &);
    virtual void initializeFromCachedUnit(const QQmlPrivate::CachedQmlUnit*);

private:
    QString m_content;
    const QV4::CompiledData::Import *m_import;
    int m_priority;
};

QQmlDataBlob::Data::Data()
{
}

const char *QQmlDataBlob::Data::data() const
{
    Q_ASSERT(!d.isNull());

    if (d.isT1()) return d.asT1()->constData();
    else return d.asT2()->data();
}

int QQmlDataBlob::Data::size() const
{
    Q_ASSERT(!d.isNull());

    if (d.isT1()) return d.asT1()->size();
    else return d.asT2()->size();
}

bool QQmlDataBlob::Data::isFile() const
{
    return d.isT2();
}

QByteArray QQmlDataBlob::Data::asByteArray() const
{
    Q_ASSERT(!d.isNull());

    if (d.isT1()) return *d.asT1();
    else return d.asT2()->dataByteArray();
}

QQmlFile *QQmlDataBlob::Data::asFile() const
{
    if (d.isT2()) return d.asT2();
    else return 0;
}

QT_END_NAMESPACE

#endif // QQMLTYPELOADER_P_H
