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

#ifndef QGSTREAMERVIDEOWIDGET_H
#define QGSTREAMERVIDEOWIDGET_H

#include <qvideowidgetcontrol.h>

#include "qgstreamervideorendererinterface_p.h"
#include <private/qgstreamerbushelper_p.h>
#include <private/qgstreamervideooverlay_p.h>

QT_BEGIN_NAMESPACE

class QGstreamerVideoWidget;

class QGstreamerVideoWidgetControl
        : public QVideoWidgetControl
        , public QGstreamerVideoRendererInterface
        , public QGstreamerSyncMessageFilter
        , public QGstreamerBusMessageFilter
{
    Q_OBJECT
    Q_INTERFACES(QGstreamerVideoRendererInterface QGstreamerSyncMessageFilter QGstreamerBusMessageFilter)
public:
    explicit QGstreamerVideoWidgetControl(QObject *parent = 0, const QByteArray &elementName = QByteArray());
    virtual ~QGstreamerVideoWidgetControl();

    GstElement *videoSink();

    QWidget *videoWidget();

    void stopRenderer();

    Qt::AspectRatioMode aspectRatioMode() const;
    void setAspectRatioMode(Qt::AspectRatioMode mode);

    bool isFullScreen() const;
    void setFullScreen(bool fullScreen);

    int brightness() const;
    void setBrightness(int brightness);

    int contrast() const;
    void setContrast(int contrast);

    int hue() const;
    void setHue(int hue);

    int saturation() const;
    void setSaturation(int saturation);

    bool eventFilter(QObject *object, QEvent *event);

signals:
    void sinkChanged();
    void readyChanged(bool);

private Q_SLOTS:
    void onOverlayActiveChanged();
    void onNativeVideoSizeChanged();

private:
    void createVideoWidget();
    void updateWidgetAttributes();

    bool processSyncMessage(const QGstreamerMessage &message);
    bool processBusMessage(const QGstreamerMessage &message);

    QGstreamerVideoOverlay m_videoOverlay;
    QGstreamerVideoWidget *m_widget;
    bool m_stopped;
    WId m_windowId;
    bool m_fullScreen;
};

QT_END_NAMESPACE

#endif // QGSTREAMERVIDEOWIDGET_H
