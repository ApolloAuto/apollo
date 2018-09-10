/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtQuick module of the Qt Toolkit.
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

#ifndef QQUICKCONTEXT2D_P_H
#define QQUICKCONTEXT2D_P_H

#include <QtQuick/qtquickglobal.h>
#include <QtQml/qqml.h>
#include <QtQml/qqmlcomponent.h>
#include <private/qquickcanvascontext_p.h>
#include <private/qquickcanvasitem_p.h>
#include <QtGui/qpainter.h>
#include <QtGui/qpainterpath.h>
#include <QtGui/qoffscreensurface.h>
#include <QtCore/qstring.h>
#include <QtCore/qstack.h>
#include <QtCore/qqueue.h>
#include <private/qv8engine_p.h>
#include <QtCore/QWaitCondition>

#include <private/qv4value_inl_p.h>

//#define QQUICKCONTEXT2D_DEBUG //enable this for just DEBUG purpose!

#ifdef QQUICKCONTEXT2D_DEBUG
#include <QElapsedTimer>
#endif

QT_BEGIN_NAMESPACE

class QQuickContext2DCommandBuffer;
class QQuickContext2DTexture;
class QQuickPixmap;
class QSGTexture;
class QSurface;
class QOpenGLContext;

class QQuickContext2D : public QQuickCanvasContext
{
    Q_OBJECT

public:
    Q_DISABLE_COPY(QQuickContext2D)

    enum TextBaseLineType { Alphabetic=0, Top, Middle, Bottom, Hanging};
    enum TextAlignType { Start=0, End, Left, Right, Center};
    enum PaintCommand {
        Invalid = 0,
        UpdateMatrix,
        ClearRect,
        FillRect,
        StrokeRect,
        Fill,
        Stroke,
        Clip,
        UpdateBrush,
        GlobalAlpha,
        GlobalCompositeOperation,
        StrokeStyle,
        FillStyle,
        LineWidth,
        LineCap,
        LineJoin,
        MiterLimit,
        ShadowOffsetX,
        ShadowOffsetY,
        ShadowBlur,
        ShadowColor,
        Font,
        TextBaseline,
        TextAlign,
        FillText,
        StrokeText,
        DrawImage,
        DrawPixmap,
        GetImageData
    };

    struct State {
        State()
            : strokeStyle(QColor("#000000"))
            , fillStyle(QColor("#000000"))
            , fillPatternRepeatX(false)
            , fillPatternRepeatY(false)
            , strokePatternRepeatX(false)
            , strokePatternRepeatY(false)
            , invertibleCTM(true)
            , clip(false)
            , fillRule(Qt::WindingFill)
            , globalAlpha(1.0)
            , lineWidth(1)
            , lineCap(Qt::FlatCap)
            , lineJoin(Qt::MiterJoin)
            , miterLimit(10)
            , shadowOffsetX(0)
            , shadowOffsetY(0)
            , shadowBlur(0)
            , shadowColor(qRgba(0, 0, 0, 0))
            , globalCompositeOperation(QPainter::CompositionMode_SourceOver)
            , font(QFont(QLatin1String("sans-serif")))
            , textAlign(QQuickContext2D::Start)
            , textBaseline(QQuickContext2D::Alphabetic)
        {
            font.setPixelSize(10);
        }

        QTransform matrix;
        QPainterPath clipPath;
        QBrush strokeStyle;
        QBrush fillStyle;
        bool fillPatternRepeatX:1;
        bool fillPatternRepeatY:1;
        bool strokePatternRepeatX:1;
        bool strokePatternRepeatY:1;
        bool invertibleCTM:1;
        bool clip:1;
        Qt::FillRule fillRule;
        qreal globalAlpha;
        qreal lineWidth;
        Qt::PenCapStyle lineCap;
        Qt::PenJoinStyle lineJoin;
        qreal miterLimit;
        qreal shadowOffsetX;
        qreal shadowOffsetY;
        qreal shadowBlur;
        QColor shadowColor;
        QPainter::CompositionMode globalCompositeOperation;
        QFont font;
        QQuickContext2D::TextAlignType textAlign;
        QQuickContext2D::TextBaseLineType textBaseline;
    };

    QQuickContext2D(QObject *parent = 0);
    ~QQuickContext2D();

    QStringList contextNames() const;
    void init(QQuickCanvasItem *canvasItem, const QVariantMap &args);
    void prepare(const QSize& canvasSize, const QSize& tileSize, const QRect& canvasWindow, const QRect& dirtyRect, bool smooth, bool antialiasing);
    void flush();
    void sync();
    QThread *thread() const { return m_thread; }
    QQuickContext2DTexture *texture() const;
    QImage toImage(const QRectF& bounds);

    QV4::ReturnedValue v4value() const;
    void setV4Engine(QV4::ExecutionEngine *eng);

    QQuickCanvasItem* canvas() const { return m_canvas; }
    QQuickContext2DCommandBuffer* buffer() const { return m_buffer; }

    bool bufferValid() const { return m_buffer != 0; }
    void popState();
    void pushState();
    void reset();

    void fill();
    void clip();
    void stroke();
    void fillRect(qreal x, qreal y, qreal w, qreal h);
    void strokeRect(qreal x, qreal y, qreal w, qreal h);
    void clearRect(qreal x, qreal y, qreal w, qreal h);
    void drawText(const QString& text, qreal x, qreal y, bool fill);

    //Transform APIs
    void scale(qreal x,  qreal y);
    void rotate(qreal angle);
    void shear(qreal h, qreal v);
    void translate(qreal x, qreal y);
    void transform(qreal a, qreal b, qreal c, qreal d, qreal e, qreal f);
    void setTransform(qreal a, qreal b, qreal c, qreal d, qreal e, qreal f);

    // Path APIs
    void beginPath();
    void closePath();
    void moveTo(qreal x, qreal y);
    void lineTo(qreal x, qreal y);
    void quadraticCurveTo(qreal cpx, qreal cpy, qreal x, qreal y);
    void bezierCurveTo(qreal cp1x, qreal cp1y,
                       qreal cp2x, qreal cp2y, qreal x, qreal y);
    void arcTo(qreal x1, qreal y1, qreal x2, qreal y2, qreal radius);
    void rect(qreal x, qreal y, qreal w, qreal h);
    void roundedRect(qreal x, qreal y,qreal w, qreal h, qreal xr, qreal yr);
    void ellipse(qreal x, qreal y,qreal w, qreal h);
    void text(const QString& str, qreal x, qreal y);
    void arc(qreal x, qreal y, qreal radius,
             qreal startAngle, qreal endAngle,
             bool anticlockwise);
    void addArcTo(const QPointF& p1, const QPointF& p2, float radius);

    bool isPointInPath(qreal x, qreal y) const;

    QPainterPath createTextGlyphs(qreal x, qreal y, const QString& text);
    QQmlRefPointer<QQuickCanvasPixmap> createPixmap(const QUrl& url);

    QOpenGLContext *glContext() { return m_glContext; }
    QSurface *surface() { return m_surface.data(); }
    void setGrabbedImage(const QImage& grab);

    State state;
    QStack<QQuickContext2D::State> m_stateStack;
    QQuickCanvasItem* m_canvas;
    QQuickContext2DCommandBuffer* m_buffer;
    QPainterPath m_path;
    QV4::PersistentValue m_fillStyle;
    QV4::PersistentValue m_strokeStyle;
    QV4::PersistentValue m_v4path;
    QV4::ExecutionEngine *m_v4engine;
    QScopedPointer<QOffscreenSurface> m_surface;
    QOpenGLContext *m_glContext;
    QV4::PersistentValue m_v4value;
    QQuickContext2DTexture *m_texture;
    QQuickCanvasItem::RenderTarget m_renderTarget;
    QQuickCanvasItem::RenderStrategy m_renderStrategy;
    QQueue<QQuickContext2DCommandBuffer*> m_bufferQueue;
    QThread *m_thread;
    QImage m_grabbedImage;
    bool m_grabbed:1;

    static QMutex mutex;
};


QT_END_NAMESPACE
QML_DECLARE_TYPE(QQuickContext2D)

#endif // QQUICKCONTEXT2D_P_H
