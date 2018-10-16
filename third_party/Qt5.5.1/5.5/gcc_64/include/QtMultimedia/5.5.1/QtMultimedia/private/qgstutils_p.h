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

#ifndef QGSTUTILS_P_H
#define QGSTUTILS_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API. It exists purely as an
// implementation detail. This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qmap.h>
#include <QtCore/qset.h>
#include <QtCore/qvector.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <qaudioformat.h>
#include <qcamera.h>
#include <qabstractvideobuffer.h>
#include <qvideoframe.h>
#include <QDebug>

#if GST_CHECK_VERSION(1,0,0)
# define QT_GSTREAMER_PLAYBIN_ELEMENT_NAME "playbin"
# define QT_GSTREAMER_CAMERABIN_ELEMENT_NAME "camerabin"
# define QT_GSTREAMER_COLORCONVERSION_ELEMENT_NAME "videoconvert"
# define QT_GSTREAMER_RAW_AUDIO_MIME "audio/x-raw"
# define QT_GSTREAMER_VIDEOOVERLAY_INTERFACE_NAME "GstVideoOverlay"
#else
# define QT_GSTREAMER_PLAYBIN_ELEMENT_NAME "playbin2"
# define QT_GSTREAMER_CAMERABIN_ELEMENT_NAME "camerabin2"
# define QT_GSTREAMER_COLORCONVERSION_ELEMENT_NAME "ffmpegcolorspace"
# define QT_GSTREAMER_RAW_AUDIO_MIME "audio/x-raw-int"
# define QT_GSTREAMER_VIDEOOVERLAY_INTERFACE_NAME "GstXOverlay"
#endif

QT_BEGIN_NAMESPACE

class QSize;
class QVariant;
class QByteArray;
class QImage;
class QVideoSurfaceFormat;

namespace QGstUtils {
    struct CameraInfo
    {
        QString name;
        QString description;
        int orientation;
        QCamera::Position position;
        QByteArray driver;
    };

    QMap<QByteArray, QVariant> gstTagListToMap(const GstTagList *list);

    QSize capsResolution(const GstCaps *caps);
    QSize capsCorrectedResolution(const GstCaps *caps);
    QAudioFormat audioFormatForCaps(const GstCaps *caps);
#if GST_CHECK_VERSION(1,0,0)
    QAudioFormat audioFormatForSample(GstSample *sample);
#else
    QAudioFormat audioFormatForBuffer(GstBuffer *buffer);
#endif
    GstCaps *capsForAudioFormat(const QAudioFormat &format);
    void initializeGst();
    QMultimedia::SupportEstimate hasSupport(const QString &mimeType,
                                             const QStringList &codecs,
                                             const QSet<QString> &supportedMimeTypeSet);

    QVector<CameraInfo> enumerateCameras(GstElementFactory *factory = 0);
    QList<QByteArray> cameraDevices(GstElementFactory * factory = 0);
    QString cameraDescription(const QString &device, GstElementFactory * factory = 0);
    QCamera::Position cameraPosition(const QString &device, GstElementFactory * factory = 0);
    int cameraOrientation(const QString &device, GstElementFactory * factory = 0);
    QByteArray cameraDriver(const QString &device, GstElementFactory * factory = 0);

    QSet<QString> supportedMimeTypes(bool (*isValidFactory)(GstElementFactory *factory));

#if GST_CHECK_VERSION(1,0,0)
    QImage bufferToImage(GstBuffer *buffer, const GstVideoInfo &info);
    QVideoSurfaceFormat formatForCaps(
            GstCaps *caps,
            GstVideoInfo *info = 0,
            QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle);
#else
    QImage bufferToImage(GstBuffer *buffer);
    QVideoSurfaceFormat formatForCaps(
            GstCaps *caps,
            int *bytesPerLine = 0,
            QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle);
#endif

    GstCaps *capsForFormats(const QList<QVideoFrame::PixelFormat> &formats);
    void setFrameTimeStamps(QVideoFrame *frame, GstBuffer *buffer);

    void setMetaData(GstElement *element, const QMap<QByteArray, QVariant> &data);
    void setMetaData(GstBin *bin, const QMap<QByteArray, QVariant> &data);

    GstCaps *videoFilterCaps();

    QSize structureResolution(const GstStructure *s);
    QVideoFrame::PixelFormat structurePixelFormat(const GstStructure *s, int *bpp = 0);
    QSize structurePixelAspectRatio(const GstStructure *s);
    QPair<qreal, qreal> structureFrameRateRange(const GstStructure *s);

}

void qt_gst_object_ref_sink(gpointer object);
GstCaps *qt_gst_pad_get_current_caps(GstPad *pad);
GstCaps *qt_gst_pad_get_caps(GstPad *pad);
GstStructure *qt_gst_structure_new_empty(const char *name);
gboolean qt_gst_element_query_position(GstElement *element, GstFormat format, gint64 *cur);
gboolean qt_gst_element_query_duration(GstElement *element, GstFormat format, gint64 *cur);
GstCaps *qt_gst_caps_normalize(GstCaps *caps);
const gchar *qt_gst_element_get_factory_name(GstElement *element);

QDebug operator <<(QDebug debug, GstCaps *caps);

QT_END_NAMESPACE

#endif
