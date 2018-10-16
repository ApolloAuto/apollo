/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKSPRITEENGINE_P_H
#define QQUICKSPRITEENGINE_P_H

#include <QObject>
#include <QVector>
#include <QTimer>
#include <QTime>
#include <QList>
#include <QQmlListProperty>
#include <QImage>
#include <QPair>
#include <private/qquickpixmapcache_p.h>
#include <private/qtquickglobal_p.h>

QT_BEGIN_NAMESPACE

class QQuickSprite;
class Q_QUICK_PRIVATE_EXPORT QQuickStochasticState : public QObject //Currently for internal use only - Sprite and ParticleGroup
{
    Q_OBJECT
    Q_PROPERTY(int duration READ duration WRITE setDuration NOTIFY durationChanged)
    Q_PROPERTY(int durationVariation READ durationVariation WRITE setDurationVariation NOTIFY durationVariationChanged)
    //Note that manually advanced sprites need to query this variable and implement own behaviour for it
    Q_PROPERTY(bool randomStart READ randomStart WRITE setRandomStart NOTIFY randomStartChanged)
    Q_PROPERTY(QVariantMap to READ to WRITE setTo NOTIFY toChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)

public:
    QQuickStochasticState(QObject* parent = 0)
        : QObject(parent)
        , m_duration(-1)
        , m_durationVariation(0)
        , m_randomStart(false)
    {
    }

    int duration() const
    {
        return m_duration;
    }

    QString name() const
    {
        return m_name;
    }

    QVariantMap to() const
    {
        return m_to;
    }

    int durationVariation() const
    {
        return m_durationVariation;
    }


    virtual int variedDuration() const
    {
        return qMax(qreal(0.0) , m_duration
                + (m_durationVariation * ((qreal)qrand()/RAND_MAX) * 2)
                - m_durationVariation);
    }

    bool randomStart() const
    {
        return m_randomStart;
    }

Q_SIGNALS:
    void durationChanged(int arg);

    void nameChanged(const QString &arg);

    void toChanged(const QVariantMap &arg);

    void durationVariationChanged(int arg);

    void entered();//### Just playing around - don't expect full state API

    void randomStartChanged(bool arg);

public Q_SLOTS:
    void setDuration(int arg)
    {
        if (m_duration != arg) {
            m_duration = arg;
            Q_EMIT durationChanged(arg);
        }
    }

    void setName(const QString &arg)
    {
        if (m_name != arg) {
            m_name = arg;
            Q_EMIT nameChanged(arg);
        }
    }

    void setTo(const QVariantMap &arg)
    {
        if (m_to != arg) {
            m_to = arg;
            Q_EMIT toChanged(arg);
        }
    }

    void setDurationVariation(int arg)
    {
        if (m_durationVariation != arg) {
            m_durationVariation = arg;
            Q_EMIT durationVariationChanged(arg);
        }
    }

    void setRandomStart(bool arg)
    {
        if (m_randomStart != arg) {
            m_randomStart = arg;
            Q_EMIT randomStartChanged(arg);
        }
    }

private:
    QString m_name;
    QVariantMap m_to;
    int m_duration;
    int m_durationVariation;

    friend class QQuickStochasticEngine;
    bool m_randomStart;
};

class Q_QUICK_PRIVATE_EXPORT QQuickStochasticEngine : public QObject
{
    Q_OBJECT
    //TODO: Optimize single state case?
    Q_PROPERTY(QString globalGoal READ globalGoal WRITE setGlobalGoal NOTIFY globalGoalChanged)
    Q_PROPERTY(QQmlListProperty<QQuickStochasticState> states READ states)
public:
    explicit QQuickStochasticEngine(QObject *parent = 0);
    QQuickStochasticEngine(QList<QQuickStochasticState*> states, QObject *parent=0);
    ~QQuickStochasticEngine();

    QQmlListProperty<QQuickStochasticState> states()
    {
        return QQmlListProperty<QQuickStochasticState>(this, m_states);
    }

    QString globalGoal() const
    {
        return m_globalGoal;
    }

    int count() const {return m_things.count();}
    void setCount(int c);

    void setGoal(int state, int sprite=0, bool jump=false);
    void start(int index=0, int state=0);
    virtual void restart(int index=0);
    virtual void advance(int index=0);//Sends state to the next chosen state, unlike goal.
    void stop(int index=0);
    int curState(int index=0) {return m_things[index];}

    QQuickStochasticState* state(int idx){return m_states[idx];}
    int stateIndex(QQuickStochasticState* s){return m_states.indexOf(s);}
    int stateIndex(const QString& s) {
        for (int i=0; i<m_states.count(); i++)
            if (m_states[i]->name() == s)
                return i;
        return -1;
    }

    int stateCount() {return m_states.count();}
private:
Q_SIGNALS:

    void globalGoalChanged(const QString &arg);
    void stateChanged(int idx);

public Q_SLOTS:
    void setGlobalGoal(const QString &arg)
    {
        if (m_globalGoal != arg) {
            m_globalGoal = arg;
            Q_EMIT globalGoalChanged(arg);
        }
    }

    uint updateSprites(uint time);

protected:
    friend class QQuickParticleSystem;
    void addToUpdateList(uint t, int idx);
    int nextState(int curState, int idx=0);
    int goalSeek(int curState, int idx, int dist=-1);
    QList<QQuickStochasticState*> m_states;
    //### Consider struct or class for the four data variables?
    QVector<int> m_things;//int is the index in m_states of the current state
    QVector<int> m_goals;
    QVector<int> m_duration;
    QVector<int> m_startTimes;
    QList<QPair<uint, QList<int> > > m_stateUpdates;//### This could be done faster - priority queue?

    QTime m_advanceTime;
    uint m_timeOffset;
    QString m_globalGoal;
    int m_maxFrames;
    int m_imageStateCount;
    bool m_addAdvance;
};

class Q_QUICK_PRIVATE_EXPORT QQuickSpriteEngine : public QQuickStochasticEngine
{
    Q_OBJECT
    Q_PROPERTY(QQmlListProperty<QQuickSprite> sprites READ sprites)
public:
    explicit QQuickSpriteEngine(QObject *parent = 0);
    QQuickSpriteEngine(QList<QQuickSprite*> sprites, QObject *parent=0);
    ~QQuickSpriteEngine();
    QQmlListProperty<QQuickSprite> sprites()
    {
        return QQmlListProperty<QQuickSprite>(this, m_sprites);
    }

    QQuickSprite* sprite(int sprite=0);
    int spriteState(int sprite=0);
    int spriteStart(int sprite=0);
    int spriteFrames(int sprite=0);
    int spriteDuration(int sprite=0);
    int spriteX(int sprite=0);
    int spriteY(int sprite=0);
    int spriteWidth(int sprite=0);
    int spriteHeight(int sprite=0);
    int spriteCount();//Like state count
    int maxFrames();

    void restart(int index=0) Q_DECL_OVERRIDE;
    void advance(int index=0) Q_DECL_OVERRIDE;

    //Similar API to QQuickPixmap for async loading convenience
    bool isNull() { return status() == QQuickPixmap::Null; }
    bool isReady() { return status() == QQuickPixmap::Ready; }
    bool isLoading() { return status() == QQuickPixmap::Loading; }
    bool isError() { return status() == QQuickPixmap::Error; }
    QQuickPixmap::Status status();//Composed status of all Sprites
    void startAssemblingImage();
    QImage assembledImage();

private:
    int pseudospriteProgress(int,int,int*rd=0);
    QList<QQuickSprite*> m_sprites;
    bool m_startedImageAssembly;
    bool m_loaded;
    bool m_errorsPrinted;
};

//Common use is to have your own list property which is transparently an engine
inline void spriteAppend(QQmlListProperty<QQuickSprite> *p, QQuickSprite* s)
{
    reinterpret_cast<QList<QQuickSprite *> *>(p->data)->append(s);
    p->object->metaObject()->invokeMethod(p->object, "createEngine");
}

inline QQuickSprite* spriteAt(QQmlListProperty<QQuickSprite> *p, int idx)
{
    return reinterpret_cast<QList<QQuickSprite *> *>(p->data)->at(idx);
}

inline void spriteClear(QQmlListProperty<QQuickSprite> *p)
{
    reinterpret_cast<QList<QQuickSprite *> *>(p->data)->clear();
    p->object->metaObject()->invokeMethod(p->object, "createEngine");
}

inline int spriteCount(QQmlListProperty<QQuickSprite> *p)
{
    return reinterpret_cast<QList<QQuickSprite *> *>(p->data)->count();
}

QT_END_NAMESPACE

#endif // QQUICKSPRITEENGINE_P_H
