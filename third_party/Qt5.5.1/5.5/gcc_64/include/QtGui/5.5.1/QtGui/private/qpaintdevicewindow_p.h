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

#ifndef QPAINTDEVICEWINDOW_P_H
#define QPAINTDEVICEWINDOW_P_H

#include <QtGui/QPaintDeviceWindow>
#include <QtCore/QCoreApplication>
#include <QtGui/private/qwindow_p.h>
#include <QtGui/QPaintEvent>

QT_BEGIN_NAMESPACE

class Q_GUI_EXPORT QPaintDeviceWindowPrivate : public QWindowPrivate
{
    Q_DECLARE_PUBLIC(QPaintDeviceWindow)

public:
    virtual void beginPaint(const QRegion &region)
    {
        Q_UNUSED(region);
    }

    virtual void endPaint()
    {
    }

    virtual void flush(const QRegion &region)
    {
        Q_UNUSED(region);
    }

    bool paint(const QRegion &region)
    {
        Q_Q(QPaintDeviceWindow);
        QRegion toPaint = region & dirtyRegion;
        if (toPaint.isEmpty())
            return false;

        // Clear the region now. The overridden functions may call update().
        dirtyRegion -= toPaint;

        beginPaint(toPaint);

        QPaintEvent paintEvent(toPaint);
        q->paintEvent(&paintEvent);

        endPaint();

        return true;
    }

    void doFlush(const QRegion &region)
    {
        QRegion toFlush = region;
        if (paint(toFlush))
            flush(toFlush);
    }

    void handleUpdateEvent()
    {
        if (dirtyRegion.isEmpty())
            return;
        doFlush(dirtyRegion);
    }

    void markWindowAsDirty()
    {
        Q_Q(QPaintDeviceWindow);
        dirtyRegion += QRect(QPoint(0, 0), q->size());
    }

private:
    QRegion dirtyRegion;
};


QT_END_NAMESPACE

#endif //QPAINTDEVICEWINDOW_P_H
