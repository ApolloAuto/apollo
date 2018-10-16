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

#ifndef QQUICKPATHINTERPOLATOR_P_H
#define QQUICKPATHINTERPOLATOR_P_H

#include <qqml.h>
#include <QObject>

QT_BEGIN_NAMESPACE

class QQuickPath;
class Q_AUTOTEST_EXPORT QQuickPathInterpolator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QQuickPath *path READ path WRITE setPath NOTIFY pathChanged)
    Q_PROPERTY(qreal progress READ progress WRITE setProgress NOTIFY progressChanged)
    Q_PROPERTY(qreal x READ x NOTIFY xChanged)
    Q_PROPERTY(qreal y READ y NOTIFY yChanged)
    Q_PROPERTY(qreal angle READ angle NOTIFY angleChanged)
public:
    explicit QQuickPathInterpolator(QObject *parent = 0);

    QQuickPath *path() const;
    void setPath(QQuickPath *path);

    qreal progress() const;
    void setProgress(qreal progress);

    qreal x() const;
    qreal y() const;
    qreal angle() const;

Q_SIGNALS:
    void pathChanged();
    void progressChanged();
    void xChanged();
    void yChanged();
    void angleChanged();

private Q_SLOTS:
    void _q_pathUpdated();

private:
    QQuickPath *_path;
    qreal _x;
    qreal _y;
    qreal _angle;
    qreal _progress;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickPathInterpolator)

#endif // QQUICKPATHINTERPOLATOR_P_H
