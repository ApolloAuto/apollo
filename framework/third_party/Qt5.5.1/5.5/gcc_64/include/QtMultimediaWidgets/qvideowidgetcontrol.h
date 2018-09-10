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

#ifndef QVIDEOWIDGETCONTROL_H
#define QVIDEOWIDGETCONTROL_H

#include <QtMultimediaWidgets/qvideowidget.h>
#include <QtMultimedia/qmediacontrol.h>

#include <QtWidgets/qwidget.h>

QT_BEGIN_NAMESPACE


class QVideoWidgetControlPrivate;

class Q_MULTIMEDIAWIDGETS_EXPORT QVideoWidgetControl : public QMediaControl
{
    Q_OBJECT

public:
    virtual ~QVideoWidgetControl();

    virtual QWidget *videoWidget() = 0;

    virtual Qt::AspectRatioMode aspectRatioMode() const = 0;
    virtual void setAspectRatioMode(Qt::AspectRatioMode mode) = 0;

    virtual bool isFullScreen() const = 0;
    virtual void setFullScreen(bool fullScreen) = 0;

    virtual int brightness() const = 0;
    virtual void setBrightness(int brightness) = 0;

    virtual int contrast() const = 0;
    virtual void setContrast(int contrast) = 0;

    virtual int hue() const = 0;
    virtual void setHue(int hue) = 0;

    virtual int saturation() const = 0;
    virtual void setSaturation(int saturation) = 0;

Q_SIGNALS:
    void fullScreenChanged(bool fullScreen);
    void brightnessChanged(int brightness);
    void contrastChanged(int contrast);
    void hueChanged(int hue);
    void saturationChanged(int saturation);

protected:
    QVideoWidgetControl(QObject *parent = 0);
};

#define QVideoWidgetControl_iid "org.qt-project.qt.videowidgetcontrol/5.0"
Q_MEDIA_DECLARE_CONTROL(QVideoWidgetControl, QVideoWidgetControl_iid)

QT_END_NAMESPACE


#endif
