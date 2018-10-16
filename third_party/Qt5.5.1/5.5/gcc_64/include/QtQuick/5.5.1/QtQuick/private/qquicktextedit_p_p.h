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

#ifndef QQUICKTEXTEDIT_P_P_H
#define QQUICKTEXTEDIT_P_P_H

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

#include "qquicktextedit_p.h"
#include "qquickimplicitsizeitem_p_p.h"
#include "qquicktextcontrol_p.h"

#include <QtQml/qqml.h>
#include <QtCore/qlist.h>

QT_BEGIN_NAMESPACE
class QTextLayout;
class QQuickTextDocumentWithImageResources;
class QQuickTextControl;
class QQuickTextNode;

class QQuickTextEditPrivate : public QQuickImplicitSizeItemPrivate
{
public:
    Q_DECLARE_PUBLIC(QQuickTextEdit)

    typedef QQuickTextEdit Public;

    struct Node {
        explicit Node(int startPos, QQuickTextNode* node)
            : m_startPos(startPos), m_node(node), m_dirty(false) { }
        QQuickTextNode* textNode() const { return m_node; }
        void moveStartPos(int delta) { Q_ASSERT(m_startPos + delta > 0); m_startPos += delta; }
        int startPos() const { return m_startPos; }
        void setDirty() { m_dirty = true; }
        bool dirty() const { return m_dirty; }

    private:
        int m_startPos;
        QQuickTextNode* m_node;
        bool m_dirty;
    };
    typedef QList<Node*>::iterator TextNodeIterator;


    QQuickTextEditPrivate()
        : color(QRgb(0xFF000000)), selectionColor(QRgb(0xFF000080)), selectedTextColor(QRgb(0xFFFFFFFF))
        , textMargin(0.0), xoff(0), yoff(0), font(sourceFont), cursorComponent(0), cursorItem(0), document(0), control(0)
        , quickDocument(0), lastSelectionStart(0), lastSelectionEnd(0), lineCount(0)
        , hAlign(QQuickTextEdit::AlignLeft), vAlign(QQuickTextEdit::AlignTop)
        , format(QQuickTextEdit::PlainText), wrapMode(QQuickTextEdit::NoWrap)
        , renderType(QQuickTextEdit::QtRendering)
        , contentDirection(Qt::LayoutDirectionAuto)
        , mouseSelectionMode(QQuickTextEdit::SelectCharacters)
#ifndef QT_NO_IM
        , inputMethodHints(Qt::ImhNone)
#endif
        , updateType(UpdatePaintNode)
        , dirty(false), richText(false), cursorVisible(false), cursorPending(false)
        , focusOnPress(true), persistentSelection(false), requireImplicitWidth(false)
        , selectByMouse(false), canPaste(false), canPasteValid(false), hAlignImplicit(true)
        , textCached(true), inLayout(false), selectByKeyboard(false), selectByKeyboardSet(false)
        , hadSelection(false)
    {
    }

    ~QQuickTextEditPrivate()
    {
        qDeleteAll(textNodeMap);
    }

    static QQuickTextEditPrivate *get(QQuickTextEdit *item) {
        return static_cast<QQuickTextEditPrivate *>(QObjectPrivate::get(item)); }

    void init();

    void updateDefaultTextOption();
    void relayoutDocument();
    bool determineHorizontalAlignment();
    bool setHAlign(QQuickTextEdit::HAlignment, bool forceAlign = false);
    void mirrorChange() Q_DECL_OVERRIDE;
    qreal getImplicitWidth() const Q_DECL_OVERRIDE;
    Qt::LayoutDirection textDirection(const QString &text) const;
    bool isLinkHoveredConnected();

    void setNativeCursorEnabled(bool) {}
    void handleFocusEvent(QFocusEvent *event);
    void addCurrentTextNodeToRoot(QSGTransformNode *, QQuickTextNode*, TextNodeIterator&, int startPos);
    QQuickTextNode* createTextNode();

#ifndef QT_NO_IM
    Qt::InputMethodHints effectiveInputMethodHints() const;
#endif

    QColor color;
    QColor selectionColor;
    QColor selectedTextColor;

    QSizeF contentSize;

    qreal textMargin;
    qreal xoff;
    qreal yoff;

    QString text;
    QUrl baseUrl;
    QFont sourceFont;
    QFont font;

    QQmlComponent* cursorComponent;
    QQuickItem* cursorItem;
    QQuickTextDocumentWithImageResources *document;
    QQuickTextControl *control;
    QQuickTextDocument *quickDocument;
    QList<Node*> textNodeMap;

    int lastSelectionStart;
    int lastSelectionEnd;
    int lineCount;

    enum UpdateType {
        UpdateNone,
        UpdateOnlyPreprocess,
        UpdatePaintNode
    };

    QQuickTextEdit::HAlignment hAlign;
    QQuickTextEdit::VAlignment vAlign;
    QQuickTextEdit::TextFormat format;
    QQuickTextEdit::WrapMode wrapMode;
    QQuickTextEdit::RenderType renderType;
    Qt::LayoutDirection contentDirection;
    QQuickTextEdit::SelectionMode mouseSelectionMode;
#ifndef QT_NO_IM
    Qt::InputMethodHints inputMethodHints;
#endif
    UpdateType updateType;

    bool dirty : 1;
    bool richText : 1;
    bool cursorVisible : 1;
    bool cursorPending : 1;
    bool focusOnPress : 1;
    bool persistentSelection : 1;
    bool requireImplicitWidth:1;
    bool selectByMouse:1;
    bool canPaste:1;
    bool canPasteValid:1;
    bool hAlignImplicit:1;
    bool textCached:1;
    bool inLayout:1;
    bool selectByKeyboard:1;
    bool selectByKeyboardSet:1;
    bool hadSelection : 1;
};

QT_END_NAMESPACE

#endif // QQUICKTEXTEDIT_P_P_H
