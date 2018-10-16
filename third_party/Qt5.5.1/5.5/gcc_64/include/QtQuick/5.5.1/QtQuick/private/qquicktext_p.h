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

#ifndef QQUICKTEXT_P_H
#define QQUICKTEXT_P_H

#include "qquickimplicitsizeitem_p.h"
#include <private/qtquickglobal_p.h>
#include <QtGui/qtextoption.h>

QT_BEGIN_NAMESPACE

class QQuickTextPrivate;
class QQuickTextLine;
class Q_QUICK_PRIVATE_EXPORT QQuickText : public QQuickImplicitSizeItem
{
    Q_OBJECT
    Q_ENUMS(HAlignment)
    Q_ENUMS(VAlignment)
    Q_ENUMS(TextStyle)
    Q_ENUMS(TextFormat)
    Q_ENUMS(TextElideMode)
    Q_ENUMS(WrapMode)
    Q_ENUMS(LineHeightMode)
    Q_ENUMS(FontSizeMode)
    Q_ENUMS(RenderType)

    Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)
    Q_PROPERTY(QFont font READ font WRITE setFont NOTIFY fontChanged)
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)
    Q_PROPERTY(QColor linkColor READ linkColor WRITE setLinkColor NOTIFY linkColorChanged)
    Q_PROPERTY(TextStyle style READ style WRITE setStyle NOTIFY styleChanged)
    Q_PROPERTY(QColor styleColor READ styleColor WRITE setStyleColor NOTIFY styleColorChanged)
    Q_PROPERTY(HAlignment horizontalAlignment READ hAlign WRITE setHAlign RESET resetHAlign NOTIFY horizontalAlignmentChanged)
    Q_PROPERTY(HAlignment effectiveHorizontalAlignment READ effectiveHAlign NOTIFY effectiveHorizontalAlignmentChanged)
    Q_PROPERTY(VAlignment verticalAlignment READ vAlign WRITE setVAlign NOTIFY verticalAlignmentChanged)
    Q_PROPERTY(WrapMode wrapMode READ wrapMode WRITE setWrapMode NOTIFY wrapModeChanged)
    Q_PROPERTY(int lineCount READ lineCount NOTIFY lineCountChanged)
    Q_PROPERTY(bool truncated READ truncated NOTIFY truncatedChanged)
    Q_PROPERTY(int maximumLineCount READ maximumLineCount WRITE setMaximumLineCount NOTIFY maximumLineCountChanged RESET resetMaximumLineCount)

    Q_PROPERTY(TextFormat textFormat READ textFormat WRITE setTextFormat NOTIFY textFormatChanged)
    Q_PROPERTY(TextElideMode elide READ elideMode WRITE setElideMode NOTIFY elideModeChanged) //### elideMode?
    Q_PROPERTY(qreal contentWidth READ contentWidth NOTIFY contentSizeChanged)
    Q_PROPERTY(qreal contentHeight READ contentHeight NOTIFY contentSizeChanged)
    Q_PROPERTY(qreal paintedWidth READ contentWidth NOTIFY contentSizeChanged)  // Compatibility
    Q_PROPERTY(qreal paintedHeight READ contentHeight NOTIFY contentSizeChanged)
    Q_PROPERTY(qreal lineHeight READ lineHeight WRITE setLineHeight NOTIFY lineHeightChanged)
    Q_PROPERTY(LineHeightMode lineHeightMode READ lineHeightMode WRITE setLineHeightMode NOTIFY lineHeightModeChanged)
    Q_PROPERTY(QUrl baseUrl READ baseUrl WRITE setBaseUrl RESET resetBaseUrl NOTIFY baseUrlChanged)
    Q_PROPERTY(int minimumPixelSize READ minimumPixelSize WRITE setMinimumPixelSize NOTIFY minimumPixelSizeChanged)
    Q_PROPERTY(int minimumPointSize READ minimumPointSize WRITE setMinimumPointSize NOTIFY minimumPointSizeChanged)
    Q_PROPERTY(FontSizeMode fontSizeMode READ fontSizeMode WRITE setFontSizeMode NOTIFY fontSizeModeChanged)
    Q_PROPERTY(RenderType renderType READ renderType WRITE setRenderType NOTIFY renderTypeChanged)
    Q_PROPERTY(QString hoveredLink READ hoveredLink NOTIFY linkHovered REVISION 2)

public:
    QQuickText(QQuickItem *parent=0);
    ~QQuickText();

    enum HAlignment { AlignLeft = Qt::AlignLeft,
                       AlignRight = Qt::AlignRight,
                       AlignHCenter = Qt::AlignHCenter,
                       AlignJustify = Qt::AlignJustify };
    enum VAlignment { AlignTop = Qt::AlignTop,
                       AlignBottom = Qt::AlignBottom,
                       AlignVCenter = Qt::AlignVCenter };
    enum TextStyle { Normal,
                      Outline,
                      Raised,
                      Sunken };
    enum TextFormat { PlainText = Qt::PlainText,
                       RichText = Qt::RichText,
                       AutoText = Qt::AutoText,
                       StyledText = 4 };
    enum TextElideMode { ElideLeft = Qt::ElideLeft,
                          ElideRight = Qt::ElideRight,
                          ElideMiddle = Qt::ElideMiddle,
                          ElideNone = Qt::ElideNone };

    enum WrapMode { NoWrap = QTextOption::NoWrap,
                    WordWrap = QTextOption::WordWrap,
                    WrapAnywhere = QTextOption::WrapAnywhere,
                    WrapAtWordBoundaryOrAnywhere = QTextOption::WrapAtWordBoundaryOrAnywhere, // COMPAT
                    Wrap = QTextOption::WrapAtWordBoundaryOrAnywhere
                  };

    enum RenderType { QtRendering,
                      NativeRendering
                    };

    enum LineHeightMode { ProportionalHeight, FixedHeight };

    enum FontSizeMode { FixedSize = 0x0, HorizontalFit = 0x01, VerticalFit = 0x02,
                        Fit = HorizontalFit | VerticalFit };

    QString text() const;
    void setText(const QString &);

    QFont font() const;
    void setFont(const QFont &font);

    QColor color() const;
    void setColor(const QColor &c);

    QColor linkColor() const;
    void setLinkColor(const QColor &color);

    TextStyle style() const;
    void setStyle(TextStyle style);

    QColor styleColor() const;
    void setStyleColor(const QColor &c);

    HAlignment hAlign() const;
    void setHAlign(HAlignment align);
    void resetHAlign();
    HAlignment effectiveHAlign() const;

    VAlignment vAlign() const;
    void setVAlign(VAlignment align);

    WrapMode wrapMode() const;
    void setWrapMode(WrapMode w);

    int lineCount() const;
    bool truncated() const;

    int maximumLineCount() const;
    void setMaximumLineCount(int lines);
    void resetMaximumLineCount();

    TextFormat textFormat() const;
    void setTextFormat(TextFormat format);

    TextElideMode elideMode() const;
    void setElideMode(TextElideMode);

    qreal lineHeight() const;
    void setLineHeight(qreal lineHeight);

    LineHeightMode lineHeightMode() const;
    void setLineHeightMode(LineHeightMode);


    QUrl baseUrl() const;
    void setBaseUrl(const QUrl &url);
    void resetBaseUrl();

    int minimumPixelSize() const;
    void setMinimumPixelSize(int size);

    int minimumPointSize() const;
    void setMinimumPointSize(int size);

    FontSizeMode fontSizeMode() const;
    void setFontSizeMode(FontSizeMode mode);

    void componentComplete() Q_DECL_OVERRIDE;

    int resourcesLoading() const; // mainly for testing

    qreal contentWidth() const;
    qreal contentHeight() const;

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    QRectF clipRect() const Q_DECL_OVERRIDE;
    Q_INVOKABLE void doLayout();

    RenderType renderType() const;
    void setRenderType(RenderType renderType);

    QString hoveredLink() const;

    Q_REVISION(3) Q_INVOKABLE QString linkAt(qreal x, qreal y) const;

Q_SIGNALS:
    void textChanged(const QString &text);
    void linkActivated(const QString &link);
    Q_REVISION(2) void linkHovered(const QString &link);
    void fontChanged(const QFont &font);
    void colorChanged();
    void linkColorChanged();
    void styleChanged(TextStyle style);
    void styleColorChanged();
    void horizontalAlignmentChanged(HAlignment alignment);
    void verticalAlignmentChanged(VAlignment alignment);
    void wrapModeChanged();
    void lineCountChanged();
    void truncatedChanged();
    void maximumLineCountChanged();
    void textFormatChanged(TextFormat textFormat);
    void elideModeChanged(TextElideMode mode);
    void contentSizeChanged();
    void lineHeightChanged(qreal lineHeight);
    void lineHeightModeChanged(LineHeightMode mode);
    void fontSizeModeChanged();
    void minimumPixelSizeChanged();
    void minimumPointSizeChanged();
    void effectiveHorizontalAlignmentChanged();
    void lineLaidOut(QQuickTextLine *line);
    void baseUrlChanged();
    void renderTypeChanged();

protected:
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void itemChange(ItemChange change, const ItemChangeData &value) Q_DECL_OVERRIDE;
    void geometryChanged(const QRectF &newGeometry,
                                 const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *) Q_DECL_OVERRIDE;

    void updatePolish() Q_DECL_OVERRIDE;

    void hoverEnterEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void hoverMoveEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void hoverLeaveEvent(QHoverEvent *event) Q_DECL_OVERRIDE;
    void invalidateFontCaches();

private Q_SLOTS:
    void q_imagesLoaded();
    void triggerPreprocess();
    void imageDownloadFinished();

private:
    Q_DISABLE_COPY(QQuickText)
    Q_DECLARE_PRIVATE(QQuickText)
};

class QTextLine;
class QQuickTextLine : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int number READ number)
    Q_PROPERTY(qreal width READ width WRITE setWidth)
    Q_PROPERTY(qreal height READ height WRITE setHeight)
    Q_PROPERTY(qreal x READ x WRITE setX)
    Q_PROPERTY(qreal y READ y WRITE setY)

public:
    QQuickTextLine();

    void setLine(QTextLine* line);
    void setLineOffset(int offset);
    int number() const;

    qreal width() const;
    void setWidth(qreal width);

    qreal height() const;
    void setHeight(qreal height);

    qreal x() const;
    void setX(qreal x);

    qreal y() const;
    void setY(qreal y);

private:
    QTextLine *m_line;
    qreal m_height;
    int m_lineOffset;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickText)
QML_DECLARE_TYPE(QQuickTextLine)

#endif // QQUICKTEXT_P_H
