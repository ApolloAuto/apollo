/****************************************************************************
**
** Copyright (C) 2013 Research In Motion.
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

#ifndef QQUICKFLICKABLEBEHAVIOR_H
#define QQUICKFLICKABLEBEHAVIOR_H

/* ### Platform specific flickable mechanics are defined either here, or in
       mkspec files. Long-term (QtQuick 3) Flickable needs to allow such
       mechanic details to be controlled via QML so that platforms can easily
       load custom behavior at QML compile time.
*/

// The maximum number of pixels a flick can overshoot
#ifndef QML_FLICK_OVERSHOOT
#define QML_FLICK_OVERSHOOT 150
#endif

// The number of samples to use in calculating the velocity of a flick
#ifndef QML_FLICK_SAMPLEBUFFER
#define QML_FLICK_SAMPLEBUFFER 3
#endif

// The number of samples to discard when calculating the flick velocity.
// Touch panels often produce inaccurate results as the finger is lifted.
#ifndef QML_FLICK_DISCARDSAMPLES
#define QML_FLICK_DISCARDSAMPLES 0
#endif

// The default maximum velocity of a flick.
#ifndef QML_FLICK_DEFAULTMAXVELOCITY
#ifdef Q_OS_BLACKBERRY
#define QML_FLICK_DEFAULTMAXVELOCITY 10000
#else
#define QML_FLICK_DEFAULTMAXVELOCITY 2500
#endif
#endif

// The default deceleration of a flick.
#ifndef QML_FLICK_DEFAULTDECELERATION
#ifdef Q_OS_BLACKBERRY
#define QML_FLICK_DEFAULTDECELERATION 5000
#else
#define QML_FLICK_DEFAULTDECELERATION 1500
#endif
#endif

// How much faster to decelerate when overshooting
#ifndef QML_FLICK_OVERSHOOTFRICTION
#define QML_FLICK_OVERSHOOTFRICTION 8
#endif

// Multiflick acceleration minimum flick velocity threshold
#ifndef QML_FLICK_MULTIFLICK_THRESHOLD
#define QML_FLICK_MULTIFLICK_THRESHOLD 1250
#endif

// Multiflick acceleration minimum contentSize/viewSize ratio
#ifndef QML_FLICK_MULTIFLICK_RATIO
#define QML_FLICK_MULTIFLICK_RATIO 10
#endif

// Multiflick acceleration maximum velocity multiplier
#ifndef QML_FLICK_MULTIFLICK_MAXBOOST
#define QML_FLICK_MULTIFLICK_MAXBOOST 3.0
#endif

#endif //QQUICKFLICKABLEBEHAVIOR_H
