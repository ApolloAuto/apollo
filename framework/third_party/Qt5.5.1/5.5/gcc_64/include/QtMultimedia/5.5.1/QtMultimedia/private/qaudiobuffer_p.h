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

#ifndef QAUDIOBUFFER_P_H
#define QAUDIOBUFFER_P_H

#include <qtmultimediadefs.h>
#include <qmultimedia.h>

#include "qaudioformat.h"

QT_BEGIN_NAMESPACE

// Required for QDoc workaround
class QString;

class Q_MULTIMEDIA_EXPORT QAbstractAudioBuffer {
public:
    virtual ~QAbstractAudioBuffer() {}

    // Lifetime management
    virtual void release() = 0;

    // Format related
    virtual QAudioFormat format() const = 0;
    virtual qint64 startTime() const = 0;
    virtual int frameCount() const = 0;

    // R/O Data
    virtual void *constData() const = 0;

    // For writable data we do this:
    // If we only have one reference to the provider,
    // call writableData().  If that does not return 0,
    // then we're finished.  If it does return 0, then we call
    // writableClone() to get a new buffer and then release
    // the old clone if that succeeds.  If it fails, we create
    // a memory clone from the constData and release the old buffer.
    // If writableClone() succeeds, we then call writableData() on it
    // and that should be good.

    virtual void *writableData() = 0;
    virtual QAbstractAudioBuffer *clone() const = 0;
};


QT_END_NAMESPACE

#endif // QAUDIOBUFFER_P_H
