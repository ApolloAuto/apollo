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

#ifndef QQUICKSPRITESEQUENCE_P_H
#define QQUICKSPRITESEQUENCE_P_H

#include <QtQuick/QQuickItem>
#include <QTime>

QT_BEGIN_NAMESPACE

class QSGContext;
class QQuickSprite;
class QQuickSpriteEngine;
class QSGGeometryNode;
class QQuickSpriteSequenceMaterial;
class Q_AUTOTEST_EXPORT QQuickSpriteSequence : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(bool running READ running WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool interpolate READ interpolate WRITE setInterpolate NOTIFY interpolateChanged)
    Q_PROPERTY(QString goalSprite READ goalSprite WRITE setGoalSprite NOTIFY goalSpriteChanged)
    Q_PROPERTY(QString currentSprite READ currentSprite NOTIFY currentSpriteChanged)
    //###try to share similar spriteEngines for less overhead?
    Q_PROPERTY(QQmlListProperty<QQuickSprite> sprites READ sprites)
    Q_CLASSINFO("DefaultProperty", "sprites")

public:
    explicit QQuickSpriteSequence(QQuickItem *parent = 0);

    QQmlListProperty<QQuickSprite> sprites();

    bool running() const
    {
        return m_running;
    }

    bool interpolate() const
    {
        return m_interpolate;
    }

    QString goalSprite() const
    {
        return m_goalState;
    }

    QString currentSprite() const
    {
        return m_curState;
    }

Q_SIGNALS:

    void runningChanged(bool arg);
    void interpolateChanged(bool arg);
    void goalSpriteChanged(const QString &arg);
    void currentSpriteChanged(const QString &arg);

public Q_SLOTS:

    void jumpTo(const QString &sprite);
    void setGoalSprite(const QString &sprite);

    void setRunning(bool arg)
    {
        if (m_running != arg) {
            m_running = arg;
            Q_EMIT runningChanged(arg);
        }
    }

    void setInterpolate(bool arg)
    {
        if (m_interpolate != arg) {
            m_interpolate = arg;
            Q_EMIT interpolateChanged(arg);
        }
    }

private Q_SLOTS:
    void createEngine();
    void sizeVertices();

protected:
    void reset();
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;
private:
    void prepareNextFrame();
    QSGGeometryNode* buildNode();
    QSGGeometryNode *m_node;
    QQuickSpriteSequenceMaterial *m_material;
    QList<QQuickSprite*> m_sprites;
    QQuickSpriteEngine* m_spriteEngine;
    QTime m_timestamp;
    int m_curFrame;
    bool m_pleaseReset;
    bool m_running;
    bool m_interpolate;
    QString m_goalState;
    QString m_curState;
    int m_curStateIdx;
    QSizeF m_sheetSize;
};

QT_END_NAMESPACE

#endif // QQUICKSPRITESEQUENCE_P_H
