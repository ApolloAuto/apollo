/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QSTYLEHINTS_H
#define QSTYLEHINTS_H

#include <QtCore/qobject.h>

QT_BEGIN_NAMESPACE


class QPlatformIntegration;
class QStyleHintsPrivate;

class Q_GUI_EXPORT QStyleHints : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QStyleHints)
    Q_PROPERTY(int cursorFlashTime READ cursorFlashTime NOTIFY cursorFlashTimeChanged FINAL)
    Q_PROPERTY(qreal fontSmoothingGamma READ fontSmoothingGamma STORED false CONSTANT FINAL)
    Q_PROPERTY(int keyboardAutoRepeatRate READ keyboardAutoRepeatRate STORED false CONSTANT FINAL)
    Q_PROPERTY(int keyboardInputInterval READ keyboardInputInterval NOTIFY keyboardInputIntervalChanged FINAL)
    Q_PROPERTY(int mouseDoubleClickInterval READ mouseDoubleClickInterval NOTIFY mouseDoubleClickIntervalChanged FINAL)
    Q_PROPERTY(int mousePressAndHoldInterval READ mousePressAndHoldInterval STORED false CONSTANT FINAL)
    Q_PROPERTY(QChar passwordMaskCharacter READ passwordMaskCharacter STORED false CONSTANT FINAL)
    Q_PROPERTY(int passwordMaskDelay READ passwordMaskDelay STORED false CONSTANT FINAL)
    Q_PROPERTY(bool setFocusOnTouchRelease READ setFocusOnTouchRelease STORED false CONSTANT FINAL)
    Q_PROPERTY(bool showIsFullScreen READ showIsFullScreen STORED false CONSTANT FINAL)
    Q_PROPERTY(int startDragDistance READ startDragDistance NOTIFY startDragDistanceChanged FINAL)
    Q_PROPERTY(int startDragTime READ startDragTime NOTIFY startDragTimeChanged FINAL)
    Q_PROPERTY(int startDragVelocity READ startDragVelocity STORED false CONSTANT FINAL)
    Q_PROPERTY(bool useRtlExtensions READ useRtlExtensions STORED false CONSTANT FINAL)
    Q_PROPERTY(Qt::TabFocusBehavior tabFocusBehavior READ tabFocusBehavior STORED false CONSTANT FINAL)
    Q_PROPERTY(bool singleClickActivation READ singleClickActivation STORED false CONSTANT FINAL)

public:
    void setMouseDoubleClickInterval(int mouseDoubleClickInterval);
    int mouseDoubleClickInterval() const;
    int mousePressAndHoldInterval() const;
    void setStartDragDistance(int startDragDistance);
    int startDragDistance() const;
    void setStartDragTime(int startDragTime);
    int startDragTime() const;
    int startDragVelocity() const;
    void setKeyboardInputInterval(int keyboardInputInterval);
    int keyboardInputInterval() const;
    int keyboardAutoRepeatRate() const;
    void setCursorFlashTime(int cursorFlashTime);
    int cursorFlashTime() const;
    bool showIsFullScreen() const;
    int passwordMaskDelay() const;
    QChar passwordMaskCharacter() const;
    qreal fontSmoothingGamma() const;
    bool useRtlExtensions() const;
    bool setFocusOnTouchRelease() const;
    Qt::TabFocusBehavior tabFocusBehavior() const;
    bool singleClickActivation() const;

Q_SIGNALS:
    void cursorFlashTimeChanged(int cursorFlashTime);
    void keyboardInputIntervalChanged(int keyboardInputInterval);
    void mouseDoubleClickIntervalChanged(int mouseDoubleClickInterval);
    void startDragDistanceChanged(int startDragDistance);
    void startDragTimeChanged(int startDragTime);

private:
    friend class QGuiApplication;
    QStyleHints();
};

QT_END_NAMESPACE

#endif
