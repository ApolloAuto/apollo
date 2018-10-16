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

#ifndef QQUICKTEXTINPUT_P_P_H
#define QQUICKTEXTINPUT_P_P_H

#include "qquicktextinput_p.h"
#include "qquicktext_p.h"
#include "qquickimplicitsizeitem_p_p.h"

#include <QtQml/qqml.h>
#include <QtCore/qelapsedtimer.h>
#include <QtCore/qpointer.h>
#include <QtCore/qbasictimer.h>
#include <QtGui/qclipboard.h>
#include <QtGui/qguiapplication.h>
#include <QtGui/qpalette.h>
#include <QtGui/qtextlayout.h>
#include <QtGui/qstylehints.h>

#include "qplatformdefs.h"

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

QT_BEGIN_NAMESPACE

class QQuickTextNode;

class Q_AUTOTEST_EXPORT QQuickTextInputPrivate : public QQuickImplicitSizeItemPrivate
{
public:
    Q_DECLARE_PUBLIC(QQuickTextInput)

    typedef QQuickTextInput Public;

    QQuickTextInputPrivate()
        : hscroll(0)
        , vscroll(0)
        , cursorItem(0)
        , textNode(0)
        , m_maskData(0)
        , color(QRgb(0xFF000000))
        , selectionColor(QRgb(0xFF000080))
        , selectedTextColor(QRgb(0xFFFFFFFF))
        , m_cursor(0)
#ifndef QT_NO_IM
        , m_preeditCursor(0)
#endif
        , m_blinkPeriod(0)
        , m_blinkTimer(0)
        , m_maxLength(32767)
        , m_lastCursorPos(-1)
        , m_undoState(0)
        , m_selstart(0)
        , m_selend(0)
#ifndef QT_NO_IM
        , inputMethodHints(Qt::ImhNone)
#endif
        , hAlign(QQuickTextInput::AlignLeft)
        , vAlign(QQuickTextInput::AlignTop)
        , wrapMode(QQuickTextInput::NoWrap)
        , m_echoMode(QQuickTextInput::Normal)
        , renderType(QQuickTextInput::QtRendering)
        , updateType(UpdatePaintNode)
        , mouseSelectionMode(QQuickTextInput::SelectCharacters)
        , m_layoutDirection(Qt::LayoutDirectionAuto)
        , m_passwordCharacter(QGuiApplication::styleHints()->passwordMaskCharacter())
        , m_passwordMaskDelay(QGuiApplication::styleHints()->passwordMaskDelay())
        , focusOnPress(true)
        , cursorVisible(false)
        , cursorPending(false)
        , autoScroll(true)
        , selectByMouse(false)
        , canPaste(false)
        , canPasteValid(false)
        , canUndo(false)
        , canRedo(false)
        , hAlignImplicit(true)
        , selectPressed(false)
        , textLayoutDirty(true)
        , persistentSelection(false)
        , hasImState(false)
        , m_separator(0)
        , m_readOnly(0)
        , m_textDirty(0)
#ifndef QT_NO_IM
        , m_preeditDirty(0)
#endif
        , m_selDirty(0)
        , m_validInput(1)
        , m_acceptableInput(1)
        , m_blinkStatus(0)
        , m_passwordEchoEditing(false)
        , inLayout(false)
        , requireImplicitWidth(false)
    {
    }

    ~QQuickTextInputPrivate()
    {
    }

    void init();
    void startCreatingCursor();
    void ensureVisible(int position, int preeditCursor = 0, int preeditLength = 0);
    void updateHorizontalScroll();
    void updateVerticalScroll();
    bool determineHorizontalAlignment();
    bool setHAlign(QQuickTextInput::HAlignment, bool forceAlign = false);
    void mirrorChange() Q_DECL_OVERRIDE;
    bool sendMouseEventToInputContext(QMouseEvent *event);
#ifndef QT_NO_IM
    Qt::InputMethodHints effectiveInputMethodHints() const;
#endif
    void handleFocusEvent(QFocusEvent *event);

    struct MaskInputData {
        enum Casemode { NoCaseMode, Upper, Lower };
        QChar maskChar; // either the separator char or the inputmask
        bool separator;
        Casemode caseMode;
    };

    // undo/redo handling
    enum CommandType { Separator, Insert, Remove, Delete, RemoveSelection, DeleteSelection, SetSelection };
    struct Command {
        inline Command() {}
        inline Command(CommandType t, int p, QChar c, int ss, int se) : type(t),uc(c),pos(p),selStart(ss),selEnd(se) {}
        uint type : 4;
        QChar uc;
        int pos, selStart, selEnd;
    };

    enum DrawFlags {
        DrawText = 0x01,
        DrawSelections = 0x02,
        DrawCursor = 0x04,
        DrawAll = DrawText | DrawSelections | DrawCursor
    };

    QElapsedTimer tripleClickTimer;
    QSizeF contentSize;
    QPointF pressPos;
    QPointF tripleClickStartPoint;

    QPointer<QQmlComponent> cursorComponent;
#ifndef QT_NO_VALIDATOR
    QPointer<QValidator> m_validator;
#endif

    qreal hscroll;
    qreal vscroll;

    QTextLayout m_textLayout;
    QString m_text;
    QString m_inputMask;
    QString m_cancelText;
    QFont font;
    QFont sourceFont;

    QQuickItem *cursorItem;
    QQuickTextNode *textNode;
    MaskInputData *m_maskData;

    QList<int> m_transactions;
    QVector<Command> m_history;

    QColor color;
    QColor selectionColor;
    QColor selectedTextColor;

    QBasicTimer m_passwordEchoTimer;
    int lastSelectionStart;
    int lastSelectionEnd;
    int m_cursor;
#ifndef QT_NO_IM
    int m_preeditCursor;
#endif
    int m_blinkPeriod; // 0 for non-blinking cursor
    int m_blinkTimer;
    int m_maxLength;
    int m_lastCursorPos;
    int m_undoState;
    int m_selstart;
    int m_selend;

    enum UpdateType {
        UpdateNone,
        UpdateOnlyPreprocess,
        UpdatePaintNode
    };

#ifndef QT_NO_IM
    Qt::InputMethodHints inputMethodHints;
#endif
    QQuickTextInput::HAlignment hAlign;
    QQuickTextInput::VAlignment vAlign;
    QQuickTextInput::WrapMode wrapMode;
    QQuickTextInput::EchoMode m_echoMode;
    QQuickTextInput::RenderType renderType;
    UpdateType updateType;
    QQuickTextInput::SelectionMode mouseSelectionMode;
    Qt::LayoutDirection m_layoutDirection;

    QChar m_blank;
    QChar m_passwordCharacter;
    int m_passwordMaskDelay;

    bool focusOnPress:1;
    bool cursorVisible:1;
    bool cursorPending:1;
    bool autoScroll:1;
    bool selectByMouse:1;
    bool canPaste:1;
    bool canPasteValid:1;
    bool canUndo:1;
    bool canRedo:1;
    bool hAlignImplicit:1;
    bool selectPressed:1;
    bool textLayoutDirty:1;
    bool persistentSelection:1;
    bool hasImState : 1;
    bool m_separator : 1;
    bool m_readOnly : 1;
    bool m_textDirty : 1;
#ifndef QT_NO_IM
    bool m_preeditDirty : 1;
#endif
    bool m_selDirty : 1;
    bool m_validInput : 1;
    bool m_acceptableInput : 1;
    bool m_blinkStatus : 1;
    bool m_passwordEchoEditing : 1;
    bool inLayout:1;
    bool requireImplicitWidth:1;

    static inline QQuickTextInputPrivate *get(QQuickTextInput *t) {
        return t->d_func();
    }
    bool hasPendingTripleClick() const {
        return !tripleClickTimer.hasExpired(QGuiApplication::styleHints()->mouseDoubleClickInterval());
    }

    void setNativeCursorEnabled(bool enabled) {
        setCursorBlinkPeriod(enabled && cursorVisible ? QGuiApplication::styleHints()->cursorFlashTime() : 0); }

    int nextMaskBlank(int pos)
    {
        int c = findInMask(pos, true, false);
        m_separator |= (c != pos);
        return (c != -1 ?  c : m_maxLength);
    }

    int prevMaskBlank(int pos)
    {
        int c = findInMask(pos, false, false);
        m_separator |= (c != pos);
        return (c != -1 ? c : 0);
    }

    bool isUndoAvailable() const { return !m_readOnly && m_undoState; }
    bool isRedoAvailable() const { return !m_readOnly && m_undoState < (int)m_history.size(); }
    void clearUndo() { m_history.clear(); m_undoState = 0; }

    bool allSelected() const { return !m_text.isEmpty() && m_selstart == 0 && m_selend == (int)m_text.length(); }
    bool hasSelectedText() const { return !m_text.isEmpty() && m_selend > m_selstart; }

    void setSelection(int start, int length);

    inline QString selectedText() const { return hasSelectedText() ? m_text.mid(m_selstart, m_selend - m_selstart) : QString(); }
    QString textBeforeSelection() const { return hasSelectedText() ? m_text.left(m_selstart) : QString(); }
    QString textAfterSelection() const { return hasSelectedText() ? m_text.mid(m_selend) : QString(); }

    int selectionStart() const { return hasSelectedText() ? m_selstart : -1; }
    int selectionEnd() const { return hasSelectedText() ? m_selend : -1; }

    int positionAt(qreal x, qreal y, QTextLine::CursorPosition position) const;
    int positionAt(const QPointF &point, QTextLine::CursorPosition position = QTextLine::CursorBetweenCharacters) const {
        return positionAt(point.x(), point.y(), position);
    }

    void removeSelection()
    {
        int priorState = m_undoState;
        removeSelectedText();
        finishChange(priorState);
    }

    int start() const { return 0; }
    int end() const { return m_text.length(); }

    QString realText() const;

#ifndef QT_NO_CLIPBOARD
    void copy(QClipboard::Mode mode = QClipboard::Clipboard) const;
    void paste(QClipboard::Mode mode = QClipboard::Clipboard);
#endif

#ifndef QT_NO_IM
    void commitPreedit();
    void cancelPreedit();
#endif

    Qt::CursorMoveStyle cursorMoveStyle() const { return m_textLayout.cursorMoveStyle(); }
    void setCursorMoveStyle(Qt::CursorMoveStyle style) { m_textLayout.setCursorMoveStyle(style); }

    void moveCursor(int pos, bool mark = false);
    void cursorForward(bool mark, int steps)
    {
        int c = m_cursor;
        if (steps > 0) {
            while (steps--)
                c = cursorMoveStyle() == Qt::VisualMoveStyle ? m_textLayout.rightCursorPosition(c)
                                                             : m_textLayout.nextCursorPosition(c);
        } else if (steps < 0) {
            while (steps++)
                c = cursorMoveStyle() == Qt::VisualMoveStyle ? m_textLayout.leftCursorPosition(c)
                                                             : m_textLayout.previousCursorPosition(c);
        }
        moveCursor(c, mark);
    }

    void cursorWordForward(bool mark) { moveCursor(m_textLayout.nextCursorPosition(m_cursor, QTextLayout::SkipWords), mark); }
    void cursorWordBackward(bool mark) { moveCursor(m_textLayout.previousCursorPosition(m_cursor, QTextLayout::SkipWords), mark); }

    void home(bool mark) { moveCursor(0, mark); }
    void end(bool mark) { moveCursor(q_func()->text().length(), mark); }

    void backspace();
    void del();
    void deselect() { internalDeselect(); finishChange(); }
    void selectAll() { m_selstart = m_selend = m_cursor = 0; moveCursor(m_text.length(), true); }

    void insert(const QString &);
    void clear();
    void selectWordAtPos(int);

    void setCursorPosition(int pos) { if (pos <= m_text.length()) moveCursor(qMax(0, pos)); }

    bool fixup();

    QString inputMask() const { return m_maskData ? m_inputMask + QLatin1Char(';') + m_blank : QString(); }
    void setInputMask(const QString &mask)
    {
        parseInputMask(mask);
        if (m_maskData)
            moveCursor(nextMaskBlank(0));
    }

    // input methods
#ifndef QT_NO_IM
    bool composeMode() const { return !m_textLayout.preeditAreaText().isEmpty(); }

    QString preeditAreaText() const { return m_textLayout.preeditAreaText(); }
#endif

    void updatePasswordEchoEditing(bool editing);

    void cancelPasswordEchoTimer() {
        m_passwordEchoTimer.stop();
    }

    Qt::LayoutDirection textDirection() const;
    Qt::LayoutDirection layoutDirection() const;
    void setLayoutDirection(Qt::LayoutDirection direction)
    {
        if (direction != m_layoutDirection) {
            m_layoutDirection = direction;
            updateDisplayText();
        }
    }

#ifndef QT_NO_IM
    void processInputMethodEvent(QInputMethodEvent *event);
#endif
    void processKeyEvent(QKeyEvent* ev);

    void setCursorBlinkPeriod(int msec);

    void updateLayout();
    void updateBaselineOffset();

    qreal getImplicitWidth() const Q_DECL_OVERRIDE;

private:
    void removeSelectedText();
    void internalSetText(const QString &txt, int pos = -1, bool edited = true);
    void updateDisplayText(bool forceUpdate = false);

    void internalInsert(const QString &s);
    void internalDelete(bool wasBackspace = false);
    void internalRemove(int pos);

    inline void internalDeselect()
    {
        m_selDirty |= (m_selend > m_selstart);
        m_selstart = m_selend = 0;
    }

    void internalUndo(int until = -1);
    void internalRedo();
    void emitUndoRedoChanged();

    bool emitCursorPositionChanged();

    bool finishChange(int validateFromState = -1, bool update = false, bool edited = true);

    void addCommand(const Command& cmd);

    inline void separate() { m_separator = true; }

    bool separateSelection();
    void deleteStartOfWord();
    void deleteEndOfWord();
    void deleteEndOfLine();

    enum ValidatorState {
#ifndef QT_NO_VALIDATOR
        InvalidInput        = QValidator::Invalid,
        IntermediateInput   = QValidator::Intermediate,
        AcceptableInput     = QValidator::Acceptable
#else
        InvalidInput,
        IntermediateInput,
        AcceptableInput
#endif
    };

    // masking
    void parseInputMask(const QString &maskFields);
    bool isValidInput(QChar key, QChar mask) const;
    ValidatorState hasAcceptableInput(const QString &text) const;
    void checkIsValid();
    QString maskString(uint pos, const QString &str, bool clear = false) const;
    QString clearString(uint pos, uint len) const;
    QString stripString(const QString &str) const;
    int findInMask(int pos, bool forward, bool findSeparator, QChar searchChar = QChar()) const;
};

QT_END_NAMESPACE

#endif // QQUICKTEXTINPUT_P_P_H
