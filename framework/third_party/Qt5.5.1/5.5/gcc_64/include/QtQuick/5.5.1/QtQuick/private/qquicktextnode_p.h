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

#ifndef QQUICKTEXTNODE_P_H
#define QQUICKTEXTNODE_P_H

#include <QtQuick/qsgnode.h>
#include "qquicktext_p.h"
#include <qglyphrun.h>

#include <QtGui/qcolor.h>
#include <QtGui/qtextlayout.h>
#include <QtCore/qvarlengtharray.h>
#include <QtCore/qscopedpointer.h>

QT_BEGIN_NAMESPACE

class QSGGlyphNode;
class QTextBlock;
class QColor;
class QTextDocument;
class QSGContext;
class QRawFont;
class QSGRectangleNode;
class QSGClipNode;
class QSGTexture;

class QQuickTextNodeEngine;

class QQuickTextNode : public QSGTransformNode
{
public:
    enum Decoration {
        NoDecoration = 0x0,
        Underline    = 0x1,
        Overline     = 0x2,
        StrikeOut    = 0x4,
        Background   = 0x8
    };
    Q_DECLARE_FLAGS(Decorations, Decoration)

    QQuickTextNode(QQuickItem *ownerElement);
    ~QQuickTextNode();

    static bool isComplexRichText(QTextDocument *);

    void deleteContent();
    void addTextLayout(const QPointF &position, QTextLayout *textLayout, const QColor &color = QColor(),
                       QQuickText::TextStyle style = QQuickText::Normal, const QColor &styleColor = QColor(),
                       const QColor &anchorColor = QColor(),
                       const QColor &selectionColor = QColor(), const QColor &selectedTextColor = QColor(),
                       int selectionStart = -1, int selectionEnd = -1,
                       int lineStart = 0, int lineCount = -1);
    void addTextDocument(const QPointF &position, QTextDocument *textDocument, const QColor &color = QColor(),
                         QQuickText::TextStyle style = QQuickText::Normal, const QColor &styleColor = QColor(),
                         const QColor &anchorColor = QColor(),
                         const QColor &selectionColor = QColor(), const QColor &selectedTextColor = QColor(),
                         int selectionStart = -1, int selectionEnd = -1);

    void setCursor(const QRectF &rect, const QColor &color);
    void clearCursor();
    QSGRectangleNode *cursorNode() const { return m_cursorNode; }

    QSGGlyphNode *addGlyphs(const QPointF &position, const QGlyphRun &glyphs, const QColor &color,
                            QQuickText::TextStyle style = QQuickText::Normal, const QColor &styleColor = QColor(),
                            QSGNode *parentNode = 0);
    void addImage(const QRectF &rect, const QImage &image);
    void addRectangleNode(const QRectF &rect, const QColor &color);

    bool useNativeRenderer() const { return m_useNativeRenderer; }
    void setUseNativeRenderer(bool on) { m_useNativeRenderer = on; }

private:
    void initEngine(const QColor &textColor, const QColor &selectedTextColor, const QColor &selectionColor, const QColor& anchorColor = QColor()
            , const QPointF &position = QPointF());

    QSGRectangleNode *m_cursorNode;
    QList<QSGTexture *> m_textures;
    QQuickItem *m_ownerElement;
    bool m_useNativeRenderer;
    QScopedPointer<QQuickTextNodeEngine> m_engine;

    friend class QQuickTextEdit;
    friend class QQuickTextEditPrivate;
};

QT_END_NAMESPACE

#endif // QQUICKTEXTNODE_P_H
