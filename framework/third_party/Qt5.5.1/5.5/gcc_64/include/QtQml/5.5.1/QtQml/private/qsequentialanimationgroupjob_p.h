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

#ifndef QSEQUENTIALANIMATIONGROUPJOB_P_H
#define QSEQUENTIALANIMATIONGROUPJOB_P_H

#include <private/qanimationgroupjob_p.h>

QT_BEGIN_NAMESPACE

class QPauseAnimationJob;
class Q_QML_PRIVATE_EXPORT QSequentialAnimationGroupJob : public QAnimationGroupJob
{
    Q_DISABLE_COPY(QSequentialAnimationGroupJob)
public:
    QSequentialAnimationGroupJob();
    ~QSequentialAnimationGroupJob();

    int duration() const;

    QAbstractAnimationJob *currentAnimation() const { return m_currentAnimation; }

protected:
    void updateCurrentTime(int);
    void updateState(QAbstractAnimationJob::State newState, QAbstractAnimationJob::State oldState);
    void updateDirection(QAbstractAnimationJob::Direction direction);
    void uncontrolledAnimationFinished(QAbstractAnimationJob *animation);
    void debugAnimation(QDebug d) const;

private:
    struct AnimationIndex
    {
        AnimationIndex() : afterCurrent(false), timeOffset(0), animation(0) {}
        // AnimationIndex points to the animation at timeOffset, skipping 0 duration animations.
        // Note that the index semantic is slightly different depending on the direction.
        bool afterCurrent;  //whether animation is before or after m_currentAnimation   //TODO: make enum Before/After/Same
        int timeOffset; // time offset when the animation at index starts.
        QAbstractAnimationJob *animation; //points to the animation at timeOffset
    };

    int animationActualTotalDuration(QAbstractAnimationJob *anim) const;
    AnimationIndex indexForCurrentTime() const;

    void setCurrentAnimation(QAbstractAnimationJob *anim, bool intermediate = false);
    void activateCurrentAnimation(bool intermediate = false);

    void animationInserted(QAbstractAnimationJob *anim);
    void animationRemoved(QAbstractAnimationJob *anim,QAbstractAnimationJob*,QAbstractAnimationJob*);

    bool atEnd() const;

    void restart();

    // handle time changes
    void rewindForwards(const AnimationIndex &newAnimationIndex);
    void advanceForwards(const AnimationIndex &newAnimationIndex);

    //state
    QAbstractAnimationJob *m_currentAnimation;
    int m_previousLoop;
};

QT_END_NAMESPACE

#endif //QSEQUENTIALANIMATIONGROUPJOB_P_H
