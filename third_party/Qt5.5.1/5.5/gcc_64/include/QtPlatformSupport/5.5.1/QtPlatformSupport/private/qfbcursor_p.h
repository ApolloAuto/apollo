/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the plugins of the Qt Toolkit.
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

#ifndef QFBCURSOR_P_H
#define QFBCURSOR_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <qpa/qplatformcursor.h>
#include <QtGui/private/qinputdevicemanager_p.h>

QT_BEGIN_NAMESPACE

class QFbScreen;
class QFbCursor;

class QFbCursorDeviceListener : public QObject
{
    Q_OBJECT

public:
    QFbCursorDeviceListener(QFbCursor *cursor) : m_cursor(cursor) { }
    bool hasMouse() const;

public slots:
    void onDeviceListChanged(QInputDeviceManager::DeviceType type);

private:
    QFbCursor *m_cursor;
};

class QFbCursor : public QPlatformCursor
{
    Q_OBJECT

public:
    QFbCursor(QFbScreen *screen);
    ~QFbCursor();

    // output methods
    QRect dirtyRect();
    virtual QRect drawCursor(QPainter &painter);

    // input methods
    void pointerEvent(const QMouseEvent &event) Q_DECL_OVERRIDE;
    QPoint pos() const Q_DECL_OVERRIDE;
    void setPos(const QPoint &pos) Q_DECL_OVERRIDE;
#ifndef QT_NO_CURSOR
    void changeCursor(QCursor *widgetCursor, QWindow *window) Q_DECL_OVERRIDE;
#endif

    virtual void setDirty();
    virtual bool isDirty() const { return mDirty; }
    virtual bool isOnScreen() const { return mOnScreen; }
    virtual QRect lastPainted() const { return mPrevRect; }

    void updateMouseStatus();

private:
    void setCursor(const uchar *data, const uchar *mask, int width, int height, int hotX, int hotY);
    void setCursor(Qt::CursorShape shape);
    void setCursor(const QImage &image, int hotx, int hoty);
    QRect getCurrentRect();

    bool mVisible;
    QFbScreen *mScreen;
    QRect mCurrentRect;      // next place to draw the cursor
    QRect mPrevRect;         // last place the cursor was drawn
    bool mDirty;
    bool mOnScreen;
    QPlatformCursorImage *mGraphic;
    QFbCursorDeviceListener *mDeviceListener;
    QPoint m_pos;
};

QT_END_NAMESPACE

#endif // QFBCURSOR_P_H

