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

#ifndef QQUICKANIMATIONCONTROLLER_H
#define QQUICKANIMATIONCONTROLLER_H

#include <qqml.h>
#include "qquickanimation_p.h"

QT_BEGIN_NAMESPACE

class QQuickAnimationControllerPrivate;
class Q_AUTOTEST_EXPORT QQuickAnimationController : public QObject, public QQmlParserStatus
{
    Q_OBJECT
    Q_INTERFACES(QQmlParserStatus)

    Q_DECLARE_PRIVATE(QQuickAnimationController)
    Q_CLASSINFO("DefaultProperty", "animation")

    Q_PROPERTY(qreal progress READ progress WRITE setProgress NOTIFY progressChanged)
    Q_PROPERTY(QQuickAbstractAnimation *animation READ animation WRITE setAnimation NOTIFY animationChanged)

public:
    QQuickAnimationController(QObject *parent=0);
    ~QQuickAnimationController();

    qreal progress() const;
    void setProgress(qreal progress);

    QQuickAbstractAnimation *animation() const;
    void setAnimation(QQuickAbstractAnimation *animation);

    void classBegin();
    void componentComplete() {}
Q_SIGNALS:
    void progressChanged();
    void animationChanged();
public Q_SLOTS:
    void reload();
    void completeToBeginning();
    void completeToEnd();
private Q_SLOTS:
    void componentFinalized();
    void updateProgress();
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickAnimationController)

#endif // QQUICKANIMATIONCONTROLLER_H
