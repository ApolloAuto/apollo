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

#ifndef QWINDOW_P_H
#define QWINDOW_P_H

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

#include <QtGui/qscreen.h>
#include <QtGui/qwindow.h>
#include <qpa/qplatformwindow.h>

#include <QtCore/private/qobject_p.h>
#include <QtGui/QIcon>

QT_BEGIN_NAMESPACE

#define QWINDOWSIZE_MAX ((1<<24)-1)

class Q_GUI_EXPORT QWindowPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QWindow)

public:
    enum PositionPolicy
    {
        WindowFrameInclusive,
        WindowFrameExclusive
    };

    QWindowPrivate()
        : QObjectPrivate()
        , surfaceType(QWindow::RasterSurface)
        , windowFlags(Qt::Window)
        , parentWindow(0)
        , platformWindow(0)
        , visible(false)
        , visibilityOnDestroy(false)
        , exposed(false)
        , windowState(Qt::WindowNoState)
        , visibility(QWindow::Hidden)
        , resizeEventPending(true)
        , receivedExpose(false)
        , positionPolicy(WindowFrameExclusive)
        , positionAutomatic(true)
        , contentOrientation(Qt::PrimaryOrientation)
        , opacity(qreal(1.0))
        , minimumSize(0, 0)
        , maximumSize(QWINDOWSIZE_MAX, QWINDOWSIZE_MAX)
        , modality(Qt::NonModal)
        , blockedByModalWindow(false)
        , updateRequestPending(false)
        , updateTimer(0)
        , transientParent(0)
        , topLevelScreen(0)
#ifndef QT_NO_CURSOR
        , cursor(Qt::ArrowCursor)
        , hasCursor(false)
#endif
        , compositing(false)
    {
        isWindow = true;
    }

    ~QWindowPrivate()
    {
    }

    void init();

    void maybeQuitOnLastWindowClosed();
#ifndef QT_NO_CURSOR
    void setCursor(const QCursor *c = 0);
    void applyCursor();
#endif

    void deliverUpdateRequest();

    QPoint globalPosition() const {
        Q_Q(const QWindow);
        QPoint offset = q->position();
        for (const QWindow *p = q->parent(); p; p = p->parent())
            offset += p->position();
        return offset;
    }

    QWindow *topLevelWindow() const;

    virtual QWindow *eventReceiver() { Q_Q(QWindow); return q; }

    void updateVisibility();
    void _q_clearAlert();

    bool windowRecreationRequired(QScreen *newScreen) const;
    void create(bool recursive);
    void setTopLevelScreen(QScreen *newScreen, bool recreate);
    void connectToScreen(QScreen *topLevelScreen);
    void disconnectFromScreen();
    void emitScreenChangedRecursion(QScreen *newScreen);

    virtual void clearFocusObject();
    virtual QRectF closestAcceptableGeometry(const QRectF &rect) const;

    bool isPopup() const { return (windowFlags & Qt::WindowType_Mask) == Qt::Popup; }

    static QWindowPrivate *get(QWindow *window) { return window->d_func(); }

    QWindow::SurfaceType surfaceType;
    Qt::WindowFlags windowFlags;
    QWindow *parentWindow;
    QPlatformWindow *platformWindow;
    bool visible;
    bool visibilityOnDestroy;
    bool exposed;
    QSurfaceFormat requestedFormat;
    QString windowTitle;
    QString windowFilePath;
    QIcon windowIcon;
    QRect geometry;
    Qt::WindowState windowState;
    QWindow::Visibility visibility;
    bool resizeEventPending;
    bool receivedExpose;
    PositionPolicy positionPolicy;
    bool positionAutomatic;
    Qt::ScreenOrientation contentOrientation;
    qreal opacity;
    QRegion mask;

    QSize minimumSize;
    QSize maximumSize;
    QSize baseSize;
    QSize sizeIncrement;

    Qt::WindowModality modality;
    bool blockedByModalWindow;

    bool updateRequestPending;
    int updateTimer;

    QPointer<QWindow> transientParent;
    QPointer<QScreen> topLevelScreen;

#ifndef QT_NO_CURSOR
    QCursor cursor;
    bool hasCursor;
#endif

    bool compositing;
};


QT_END_NAMESPACE

#endif // QWINDOW_P_H
