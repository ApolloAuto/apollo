/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2012 Research In Motion
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

#ifndef QDECLARATIVEVIDEOOUTPUT_P_H
#define QDECLARATIVEVIDEOOUTPUT_P_H

#include <QtCore/qrect.h>
#include <QtCore/qsharedpointer.h>
#include <QtQuick/qquickitem.h>
#include <QtCore/qpointer.h>
#include <QtMultimedia/qcamerainfo.h>
#include <QtMultimedia/qabstractvideofilter.h>

#include <private/qtmultimediaquickdefs_p.h>

QT_BEGIN_NAMESPACE

class QMediaObject;
class QMediaService;
class QDeclarativeVideoBackend;
class QVideoOutputOrientationHandler;

class Q_MULTIMEDIAQUICK_EXPORT QDeclarativeVideoOutput : public QQuickItem
{
    Q_OBJECT
    Q_DISABLE_COPY(QDeclarativeVideoOutput)
    Q_PROPERTY(QObject* source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(FillMode fillMode READ fillMode WRITE setFillMode NOTIFY fillModeChanged)
    Q_PROPERTY(int orientation READ orientation WRITE setOrientation NOTIFY orientationChanged)
    Q_PROPERTY(bool autoOrientation READ autoOrientation WRITE setAutoOrientation NOTIFY autoOrientationChanged REVISION 2)
    Q_PROPERTY(QRectF sourceRect READ sourceRect NOTIFY sourceRectChanged)
    Q_PROPERTY(QRectF contentRect READ contentRect NOTIFY contentRectChanged)
    Q_PROPERTY(QQmlListProperty<QAbstractVideoFilter> filters READ filters);
    Q_ENUMS(FillMode)

public:
    enum FillMode
    {
        Stretch            = Qt::IgnoreAspectRatio,
        PreserveAspectFit  = Qt::KeepAspectRatio,
        PreserveAspectCrop = Qt::KeepAspectRatioByExpanding
    };

    QDeclarativeVideoOutput(QQuickItem *parent = 0);
    ~QDeclarativeVideoOutput();

    QObject *source() const { return m_source.data(); }
    void setSource(QObject *source);

    FillMode fillMode() const;
    void setFillMode(FillMode mode);

    int orientation() const;
    void setOrientation(int);

    bool autoOrientation() const;
    void setAutoOrientation(bool);

    QRectF sourceRect() const;
    QRectF contentRect() const;

    Q_INVOKABLE QPointF mapPointToItem(const QPointF &point) const;
    Q_INVOKABLE QRectF mapRectToItem(const QRectF &rectangle) const;
    Q_INVOKABLE QPointF mapNormalizedPointToItem(const QPointF &point) const;
    Q_INVOKABLE QRectF mapNormalizedRectToItem(const QRectF &rectangle) const;
    Q_INVOKABLE QPointF mapPointToSource(const QPointF &point) const;
    Q_INVOKABLE QRectF mapRectToSource(const QRectF &rectangle) const;
    Q_INVOKABLE QPointF mapPointToSourceNormalized(const QPointF &point) const;
    Q_INVOKABLE QRectF mapRectToSourceNormalized(const QRectF &rectangle) const;

    enum SourceType {
        NoSource,
        MediaObjectSource,
        VideoSurfaceSource
    };
    SourceType sourceType() const;

    QQmlListProperty<QAbstractVideoFilter> filters();

Q_SIGNALS:
    void sourceChanged();
    void fillModeChanged(QDeclarativeVideoOutput::FillMode);
    void orientationChanged();
    void autoOrientationChanged();
    void sourceRectChanged();
    void contentRectChanged();

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);
    void itemChange(ItemChange change, const ItemChangeData &changeData);
    void geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry);
    void releaseResources();

private Q_SLOTS:
    void _q_updateMediaObject();
    void _q_updateCameraInfo();
    void _q_updateNativeSize();
    void _q_updateGeometry();
    void _q_screenOrientationChanged(int);
    void _q_invalidateSceneGraph();

private:
    bool createBackend(QMediaService *service);

    static void filter_append(QQmlListProperty<QAbstractVideoFilter> *property, QAbstractVideoFilter *value);
    static int filter_count(QQmlListProperty<QAbstractVideoFilter> *property);
    static QAbstractVideoFilter *filter_at(QQmlListProperty<QAbstractVideoFilter> *property, int index);
    static void filter_clear(QQmlListProperty<QAbstractVideoFilter> *property);

    SourceType m_sourceType;

    QPointer<QObject> m_source;
    QPointer<QMediaObject> m_mediaObject;
    QPointer<QMediaService> m_service;
    QCameraInfo m_cameraInfo;

    FillMode m_fillMode;
    QSize m_nativeSize;

    bool m_geometryDirty;
    QRectF m_lastRect;      // Cache of last rect to avoid recalculating geometry
    QRectF m_contentRect;   // Destination pixel coordinates, unclipped
    int m_orientation;
    bool m_autoOrientation;
    QVideoOutputOrientationHandler *m_screenOrientationHandler;

    QScopedPointer<QDeclarativeVideoBackend> m_backend;

    QList<QAbstractVideoFilter *> m_filters;
};

QT_END_NAMESPACE

#endif // QDECLARATIVEVIDEOOUTPUT_H
