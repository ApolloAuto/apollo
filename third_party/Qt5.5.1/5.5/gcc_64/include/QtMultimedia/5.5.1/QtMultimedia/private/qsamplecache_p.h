/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QSAMPLECACHE_P_H
#define QSAMPLECACHE_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists purely as an
// implementation detail. This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qobject.h>
#include <QtCore/qthread.h>
#include <QtCore/qurl.h>
#include <QtCore/qmutex.h>
#include <QtCore/qmap.h>
#include <QtCore/qset.h>
#include <qaudioformat.h>


QT_BEGIN_NAMESPACE

class QIODevice;
class QNetworkAccessManager;
class QSampleCache;
class QWaveDecoder;

// Lives in application thread
class Q_MULTIMEDIA_EXPORT QSample : public QObject
{
    Q_OBJECT
public:
    friend class QSampleCache;
    enum State
    {
        Creating,
        Loading,
        Error,
        Ready,
    };

    State state() const;
    // These are not (currently) locked because they are only meant to be called after these
    // variables are updated to their final states
    const QByteArray& data() const { Q_ASSERT(state() == Ready); return m_soundData; }
    const QAudioFormat& format() const { Q_ASSERT(state() == Ready); return m_audioFormat; }
    void release();

Q_SIGNALS:
    void error();
    void ready();

protected:
    QSample(const QUrl& url, QSampleCache *parent);

private Q_SLOTS:
    void load();
    void decoderError();
    void readSample();
    void decoderReady();

private:
    void onReady();
    void cleanup();
    void addRef();
    void loadIfNecessary();
    QSample();
    ~QSample();

    mutable QMutex m_mutex;
    QSampleCache *m_parent;
    QByteArray   m_soundData;
    QAudioFormat m_audioFormat;
    QIODevice    *m_stream;
    QWaveDecoder *m_waveDecoder;
    QUrl         m_url;
    qint64       m_sampleReadLength;
    State        m_state;
    int          m_ref;
};

class Q_MULTIMEDIA_EXPORT QSampleCache : public QObject
{
    Q_OBJECT
public:
    friend class QSample;

    QSampleCache(QObject *parent = 0);
    ~QSampleCache();

    QSample* requestSample(const QUrl& url);
    void setCapacity(qint64 capacity);

    bool isLoading() const;
    bool isCached(const QUrl& url) const;

Q_SIGNALS:
    void isLoadingChanged();

private:
    QMap<QUrl, QSample*> m_samples;
    QSet<QSample*> m_staleSamples;
    QNetworkAccessManager *m_networkAccessManager;
    mutable QMutex m_mutex;
    qint64 m_capacity;
    qint64 m_usage;
    QThread m_loadingThread;

    QNetworkAccessManager& networkAccessManager();
    void refresh(qint64 usageChange);
    bool notifyUnreferencedSample(QSample* sample);
    void removeUnreferencedSample(QSample* sample);
    void unloadSample(QSample* sample);

    void loadingRelease();
    int m_loadingRefCount;
    QMutex m_loadingMutex;
};

QT_END_NAMESPACE

#endif // QSAMPLECACHE_P_H
