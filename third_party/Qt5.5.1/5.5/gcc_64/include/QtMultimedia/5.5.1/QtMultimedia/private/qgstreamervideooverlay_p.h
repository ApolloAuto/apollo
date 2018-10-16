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

#ifndef QGSTREAMERVIDEOOVERLAY_P_H
#define QGSTREAMERVIDEOOVERLAY_P_H

#include <private/qgstreamerbushelper_p.h>
#include <private/qgstreamerbufferprobe_p.h>
#include <QtGui/qwindowdefs.h>
#include <QtCore/qsize.h>

QT_BEGIN_NAMESPACE

class QGstreamerVideoOverlay
        : public QObject
        , public QGstreamerSyncMessageFilter
        , public QGstreamerBusMessageFilter
        , private QGstreamerBufferProbe
{
    Q_OBJECT
    Q_INTERFACES(QGstreamerSyncMessageFilter QGstreamerBusMessageFilter)
public:
    explicit QGstreamerVideoOverlay(QObject *parent = 0, const QByteArray &elementName = QByteArray());
    virtual ~QGstreamerVideoOverlay();

    GstElement *videoSink() const;
    QSize nativeVideoSize() const;

    void setWindowHandle(WId id);
    void expose();
    void setRenderRectangle(const QRect &rect);

    bool isActive() const;

    Qt::AspectRatioMode aspectRatioMode() const;
    void setAspectRatioMode(Qt::AspectRatioMode mode);

    int brightness() const;
    void setBrightness(int brightness);

    int contrast() const;
    void setContrast(int contrast);

    int hue() const;
    void setHue(int hue);

    int saturation() const;
    void setSaturation(int saturation);

    bool processSyncMessage(const QGstreamerMessage &message);
    bool processBusMessage(const QGstreamerMessage &message);

Q_SIGNALS:
    void nativeVideoSizeChanged();
    void activeChanged();
    void brightnessChanged(int brightness);
    void contrastChanged(int contrast);
    void hueChanged(int hue);
    void saturationChanged(int saturation);

private:
    GstElement *findBestVideoSink() const;
    void setWindowHandle_helper(WId id);
    void updateIsActive();
    void probeCaps(GstCaps *caps);
    static void showPrerollFrameChanged(GObject *, GParamSpec *, QGstreamerVideoOverlay *);

    GstElement *m_videoSink;
    QSize m_nativeVideoSize;
    bool m_isActive;

    bool m_hasForceAspectRatio;
    bool m_hasBrightness;
    bool m_hasContrast;
    bool m_hasHue;
    bool m_hasSaturation;
    bool m_hasShowPrerollFrame;

    WId m_windowId;
    Qt::AspectRatioMode m_aspectRatioMode;
    int m_brightness;
    int m_contrast;
    int m_hue;
    int m_saturation;
};

QT_END_NAMESPACE

#endif // QGSTREAMERVIDEOOVERLAY_P_H

