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

#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <QtQuick/QQuickItem>
#include <QElapsedTimer>
#include <QVector>
#include <QHash>
#include <QPointer>
#include <QSignalMapper>
#include <private/qquicksprite_p.h>
#include <QAbstractAnimation>
#include <QtQml/qqml.h>
#include <private/qv8engine_p.h> //For QQmlV4Handle

QT_BEGIN_NAMESPACE

class QQuickParticleSystem;
class QQuickParticleAffector;
class QQuickParticleEmitter;
class QQuickParticlePainter;
class QQuickParticleData;
class QQuickParticleSystemAnimation;
class QQuickStochasticEngine;
class QQuickSprite;
class QQuickV4ParticleData;
class QQuickParticleGroup;
class QQuickImageParticle;

struct QQuickParticleDataHeapNode{
    int time;//in ms
    QSet<QQuickParticleData*> data;//Set ptrs instead?
};

class QQuickParticleDataHeap {
    //Idea is to do a binary heap, but which also stores a set of int,Node* so that if the int already exists, you can
    //add it to the data* list. Pops return the whole list at once.
public:
    QQuickParticleDataHeap();
    void insert(QQuickParticleData* data);
    void insertTimed(QQuickParticleData* data, int time);

    int top();

    QSet<QQuickParticleData*> pop();

    void clear();

    bool contains(QQuickParticleData*);//O(n), for debugging purposes only
private:
    void grow();
    void swap(int, int);
    void bubbleUp(int);
    void bubbleDown(int);
    int m_size;
    int m_end;
    QQuickParticleDataHeapNode m_tmp;
    QVector<QQuickParticleDataHeapNode> m_data;
    QHash<int,int> m_lookups;
};

class Q_AUTOTEST_EXPORT QQuickParticleGroupData {
public:
    QQuickParticleGroupData(int id, QQuickParticleSystem* sys);
    ~QQuickParticleGroupData();

    int size();
    QString name();

    void setSize(int newSize);

    int index;
    QSet<QQuickParticlePainter*> painters;//TODO: What if they are dynamically removed?

    //TODO: Refactor particle data list out into a separate class
    QVector<QQuickParticleData*> data;
    QQuickParticleDataHeap dataHeap;
    QSet<int> reusableIndexes;
    bool recycle(); //Force recycling round, returns true if all indexes are now reusable

    void initList();
    void kill(QQuickParticleData* d);

    //After calling this, initialize, then call prepareRecycler(d)
    QQuickParticleData* newDatum(bool respectsLimits);

    //TODO: Find and clean up those that don't get added to the recycler (currently they get lost)
    void prepareRecycler(QQuickParticleData* d);

private:
    int m_size;
    QQuickParticleSystem* m_system;
};

struct Color4ub {
    uchar r;
    uchar g;
    uchar b;
    uchar a;
};

class Q_AUTOTEST_EXPORT QQuickParticleData {
public:
    //TODO: QObject like memory management (without the cost, just attached to system)
    QQuickParticleData(QQuickParticleSystem* sys);
    ~QQuickParticleData();

    QQuickParticleData(const QQuickParticleData &other);
    QQuickParticleData &operator=(const QQuickParticleData &other);

    //Convenience functions for working backwards, because parameters are from the start of particle life
    //If setting multiple parameters at once, doing the conversion yourself will be faster.

    //sets the x accleration without affecting the instantaneous x velocity or position
    void setInstantaneousAX(qreal ax);
    //sets the x velocity without affecting the instantaneous x postion
    void setInstantaneousVX(qreal vx);
    //sets the instantaneous x postion
    void setInstantaneousX(qreal x);
    //sets the y accleration without affecting the instantaneous y velocity or position
    void setInstantaneousAY(qreal ay);
    //sets the y velocity without affecting the instantaneous y postion
    void setInstantaneousVY(qreal vy);
    //sets the instantaneous Y postion
    void setInstantaneousY(qreal y);

    //TODO: Slight caching?
    qreal curX() const;
    qreal curVX() const;
    qreal curAX() const { return ax; }
    qreal curY() const;
    qreal curVY() const;
    qreal curAY() const { return ay; }

    QQuickParticleEmitter* e;//### Needed?
    QQuickParticleSystem* system;
    int index;
    int systemIndex;

    //General Position Stuff
    float x;
    float y;
    float t;
    float lifeSpan;
    float size;
    float endSize;
    float vx;
    float vy;
    float ax;
    float ay;

    //Painter-specific stuff, now universally shared
    //Used by ImageParticle color mode
    Color4ub color;
    //Used by ImageParticle deform mode
    float xx;
    float xy;
    float yx;
    float yy;
    float rotation;
    float rotationVelocity;
    float autoRotate;//Assume that GPUs prefer floats to bools
    //Used by ImageParticle Sprite mode
    float animIdx;
    float frameDuration;
    float frameAt;//Used for duration -1
    float frameCount;
    float animT;
    float animX;
    float animY;
    float animWidth;
    float animHeight;

    int group;

    //Used by ImageParticle data shadowing
    QQuickImageParticle* colorOwner;
    QQuickImageParticle* rotationOwner;
    QQuickImageParticle* deformationOwner;
    QQuickImageParticle* animationOwner;

    //Used by ItemParticle
    QQuickItem* delegate;
    int modelIndex;
    //Used by custom affectors
    float update;
    //Used by CustomParticle
    float r;

    // 4 bytes wasted


    void debugDump();
    bool stillAlive();//Only checks end, because usually that's all you need and it's a little faster.
    bool alive();
    float lifeLeft();
    float curSize();
    void clone(const QQuickParticleData& other);//Not =, leaves meta-data like index
    QQmlV4Handle v4Value();
    void extendLife(float time);
private:
    QQuickV4ParticleData* v8Datum;
};

class Q_AUTOTEST_EXPORT QQuickParticleSystem : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(bool running READ isRunning WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool paused READ isPaused WRITE setPaused NOTIFY pausedChanged)
    Q_PROPERTY(bool empty READ isEmpty NOTIFY emptyChanged)

public:
    explicit QQuickParticleSystem(QQuickItem *parent = 0);
    ~QQuickParticleSystem();

    bool isRunning() const
    {
        return m_running;
    }

    int count(){ return particleCount; }

    static const int maxLife = 600000;

Q_SIGNALS:

    void systemInitialized();
    void runningChanged(bool arg);
    void pausedChanged(bool arg);
    void emptyChanged(bool arg);

public Q_SLOTS:
    void start(){setRunning(true);}
    void stop(){setRunning(false);}
    void restart(){setRunning(false);setRunning(true);}
    void pause(){setPaused(true);}
    void resume(){setPaused(false);}

    void reset();
    void setRunning(bool arg);
    void setPaused(bool arg);

    virtual int duration() const { return -1; }


protected:
    //This one only once per frame (effectively)
    void componentComplete();

private Q_SLOTS:
    void emittersChanged();
    void loadPainter(QObject* p);
    void createEngine(); //Not invoked by sprite engine, unlike Sprite uses
    void particleStateChange(int idx);

public:
    //These can be called multiple times per frame, performance critical
    void emitParticle(QQuickParticleData* p);
    QQuickParticleData* newDatum(int groupId, bool respectLimits = true, int sysIdx = -1);
    void finishNewDatum(QQuickParticleData*);
    void moveGroups(QQuickParticleData *d, int newGIdx);
    int nextSystemIndex();

    //This one only once per painter per frame
    int systemSync(QQuickParticlePainter* p);

    //Data members here for ease of related class and auto-test usage. Not "public" API. TODO: d_ptrize
    QSet<QQuickParticleData*> needsReset;
    QVector<QQuickParticleData*> bySysIdx; //Another reference to the data (data owned by group), but by sysIdx
    QHash<QString, int> groupIds;
    QHash<int, QQuickParticleGroupData*> groupData;
    QQuickStochasticEngine* stateEngine;

    //Also only here for auto-test usage
    void updateCurrentTime( int currentTime );
    QQuickParticleSystemAnimation* m_animation;
    bool m_running;
    bool m_debugMode;

    int timeInt;
    bool initialized;
    int particleCount;

    void registerParticlePainter(QQuickParticlePainter* p);
    void registerParticleEmitter(QQuickParticleEmitter* e);
    void registerParticleAffector(QQuickParticleAffector* a);
    void registerParticleGroup(QQuickParticleGroup* g);

    static void statePropertyRedirect(QQmlListProperty<QObject> *prop, QObject *value);
    static void stateRedirect(QQuickParticleGroup* group, QQuickParticleSystem* sys, QObject *value);
    bool isPaused() const
    {
        return m_paused;
    }

    bool isEmpty() const
    {
        return m_empty;
    }

private:
    void initializeSystem();
    void initGroups();
    QList<QPointer<QQuickParticleEmitter> > m_emitters;
    QList<QPointer<QQuickParticleAffector> > m_affectors;
    QList<QPointer<QQuickParticlePainter> > m_painters;
    QList<QPointer<QQuickParticlePainter> > m_syncList;
    QList<QQuickParticleGroup*> m_groups;
    int m_nextGroupId;
    int m_nextIndex;
    QSet<int> m_reusableIndexes;
    bool m_componentComplete;

    QSignalMapper m_painterMapper;
    QSignalMapper m_emitterMapper;
    bool m_paused;
    bool m_allDead;
    bool m_empty;
};

// Internally, this animation drives all the timing. Painters sync up in their updatePaintNode
class QQuickParticleSystemAnimation : public QAbstractAnimation
{
    Q_OBJECT
public:
    QQuickParticleSystemAnimation(QQuickParticleSystem* system)
        : QAbstractAnimation(static_cast<QObject*>(system)), m_system(system)
    { }
protected:
    virtual void updateCurrentTime( int t )
    {
        m_system->updateCurrentTime(t);
    }

    virtual int duration() const
    {
        return -1;
    }

private:
    QQuickParticleSystem* m_system;
};


QT_END_NAMESPACE

#endif // PARTICLESYSTEM_H


