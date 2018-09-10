/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtWidgets module of the Qt Toolkit.
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

#ifndef QPLAINTEXTEDIT_H
#define QPLAINTEXTEDIT_H

#include <QtWidgets/qtextedit.h>

#include <QtWidgets/qabstractscrollarea.h>
#include <QtGui/qtextdocument.h>
#include <QtGui/qtextoption.h>
#include <QtGui/qtextcursor.h>
#include <QtGui/qtextformat.h>
#include <QtGui/qabstracttextdocumentlayout.h>

#ifndef QT_NO_TEXTEDIT

QT_BEGIN_NAMESPACE


class QStyleSheet;
class QTextDocument;
class QMenu;
class QPlainTextEditPrivate;
class QMimeData;
class QPagedPaintDevice;

class Q_WIDGETS_EXPORT QPlainTextEdit : public QAbstractScrollArea
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QPlainTextEdit)
    Q_PROPERTY(bool tabChangesFocus READ tabChangesFocus WRITE setTabChangesFocus)
    Q_PROPERTY(QString documentTitle READ documentTitle WRITE setDocumentTitle)
    Q_PROPERTY(bool undoRedoEnabled READ isUndoRedoEnabled WRITE setUndoRedoEnabled)
    Q_PROPERTY(LineWrapMode lineWrapMode READ lineWrapMode WRITE setLineWrapMode)
    QDOC_PROPERTY(QTextOption::WrapMode wordWrapMode READ wordWrapMode WRITE setWordWrapMode)
    Q_PROPERTY(bool readOnly READ isReadOnly WRITE setReadOnly)
    Q_PROPERTY(QString plainText READ toPlainText WRITE setPlainText NOTIFY textChanged USER true)
    Q_PROPERTY(bool overwriteMode READ overwriteMode WRITE setOverwriteMode)
    Q_PROPERTY(int tabStopWidth READ tabStopWidth WRITE setTabStopWidth)
    Q_PROPERTY(int cursorWidth READ cursorWidth WRITE setCursorWidth)
    Q_PROPERTY(Qt::TextInteractionFlags textInteractionFlags READ textInteractionFlags WRITE setTextInteractionFlags)
    Q_PROPERTY(int blockCount READ blockCount)
    Q_PROPERTY(int maximumBlockCount READ maximumBlockCount WRITE setMaximumBlockCount)
    Q_PROPERTY(bool backgroundVisible READ backgroundVisible WRITE setBackgroundVisible)
    Q_PROPERTY(bool centerOnScroll READ centerOnScroll WRITE setCenterOnScroll)
    Q_PROPERTY(QString placeholderText READ placeholderText WRITE setPlaceholderText)
public:
    enum LineWrapMode {
        NoWrap,
        WidgetWidth
    };
    Q_ENUM(LineWrapMode)

    explicit QPlainTextEdit(QWidget *parent = 0);
    explicit QPlainTextEdit(const QString &text, QWidget *parent = 0);
    virtual ~QPlainTextEdit();

    void setDocument(QTextDocument *document);
    QTextDocument *document() const;

    void setPlaceholderText(const QString &placeholderText);
    QString placeholderText() const;

    void setTextCursor(const QTextCursor &cursor);
    QTextCursor textCursor() const;

    bool isReadOnly() const;
    void setReadOnly(bool ro);

    void setTextInteractionFlags(Qt::TextInteractionFlags flags);
    Qt::TextInteractionFlags textInteractionFlags() const;

    void mergeCurrentCharFormat(const QTextCharFormat &modifier);
    void setCurrentCharFormat(const QTextCharFormat &format);
    QTextCharFormat currentCharFormat() const;

    bool tabChangesFocus() const;
    void setTabChangesFocus(bool b);

    inline void setDocumentTitle(const QString &title)
    { document()->setMetaInformation(QTextDocument::DocumentTitle, title); }
    inline QString documentTitle() const
    { return document()->metaInformation(QTextDocument::DocumentTitle); }

    inline bool isUndoRedoEnabled() const
    { return document()->isUndoRedoEnabled(); }
    inline void setUndoRedoEnabled(bool enable)
    { document()->setUndoRedoEnabled(enable); }

    inline void setMaximumBlockCount(int maximum)
    { document()->setMaximumBlockCount(maximum); }
    inline int maximumBlockCount() const
    { return document()->maximumBlockCount(); }


    LineWrapMode lineWrapMode() const;
    void setLineWrapMode(LineWrapMode mode);

    QTextOption::WrapMode wordWrapMode() const;
    void setWordWrapMode(QTextOption::WrapMode policy);

    void setBackgroundVisible(bool visible);
    bool backgroundVisible() const;

    void setCenterOnScroll(bool enabled);
    bool centerOnScroll() const;

    bool find(const QString &exp, QTextDocument::FindFlags options = 0);
#ifndef QT_NO_REGEXP
    bool find(const QRegExp &exp, QTextDocument::FindFlags options = 0);
#endif

    inline QString toPlainText() const
    { return document()->toPlainText(); }

    void ensureCursorVisible();

    virtual QVariant loadResource(int type, const QUrl &name);
#ifndef QT_NO_CONTEXTMENU
    QMenu *createStandardContextMenu();
    QMenu *createStandardContextMenu(const QPoint &position);
#endif

    QTextCursor cursorForPosition(const QPoint &pos) const;
    QRect cursorRect(const QTextCursor &cursor) const;
    QRect cursorRect() const;

    QString anchorAt(const QPoint &pos) const;

    bool overwriteMode() const;
    void setOverwriteMode(bool overwrite);

    int tabStopWidth() const;
    void setTabStopWidth(int width);

    int cursorWidth() const;
    void setCursorWidth(int width);

    void setExtraSelections(const QList<QTextEdit::ExtraSelection> &selections);
    QList<QTextEdit::ExtraSelection> extraSelections() const;

    void moveCursor(QTextCursor::MoveOperation operation, QTextCursor::MoveMode mode = QTextCursor::MoveAnchor);

    bool canPaste() const;

    void print(QPagedPaintDevice *printer) const;

    int blockCount() const;
    QVariant inputMethodQuery(Qt::InputMethodQuery property) const Q_DECL_OVERRIDE;
    Q_INVOKABLE QVariant inputMethodQuery(Qt::InputMethodQuery query, QVariant argument) const;

public Q_SLOTS:

    void setPlainText(const QString &text);

#ifndef QT_NO_CLIPBOARD
    void cut();
    void copy();
    void paste();
#endif

    void undo();
    void redo();

    void clear();
    void selectAll();

    void insertPlainText(const QString &text);

    void appendPlainText(const QString &text);
    void appendHtml(const QString &html);

    void centerCursor();

    void zoomIn(int range = 1);
    void zoomOut(int range = 1);

Q_SIGNALS:
    void textChanged();
    void undoAvailable(bool b);
    void redoAvailable(bool b);
    void copyAvailable(bool b);
    void selectionChanged();
    void cursorPositionChanged();

    void updateRequest(const QRect &rect, int dy);
    void blockCountChanged(int newBlockCount);
    void modificationChanged(bool);

protected:
    virtual bool event(QEvent *e) Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;
    virtual void keyPressEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    virtual void keyReleaseEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    virtual void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;
    virtual void paintEvent(QPaintEvent *e) Q_DECL_OVERRIDE;
    virtual void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    virtual void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    virtual void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    virtual void mouseDoubleClickEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    virtual bool focusNextPrevChild(bool next) Q_DECL_OVERRIDE;
#ifndef QT_NO_CONTEXTMENU
    virtual void contextMenuEvent(QContextMenuEvent *e) Q_DECL_OVERRIDE;
#endif
#ifndef QT_NO_DRAGANDDROP
    virtual void dragEnterEvent(QDragEnterEvent *e) Q_DECL_OVERRIDE;
    virtual void dragLeaveEvent(QDragLeaveEvent *e) Q_DECL_OVERRIDE;
    virtual void dragMoveEvent(QDragMoveEvent *e) Q_DECL_OVERRIDE;
    virtual void dropEvent(QDropEvent *e) Q_DECL_OVERRIDE;
#endif
    virtual void focusInEvent(QFocusEvent *e) Q_DECL_OVERRIDE;
    virtual void focusOutEvent(QFocusEvent *e) Q_DECL_OVERRIDE;
    virtual void showEvent(QShowEvent *) Q_DECL_OVERRIDE;
    virtual void changeEvent(QEvent *e) Q_DECL_OVERRIDE;
#ifndef QT_NO_WHEELEVENT
    virtual void wheelEvent(QWheelEvent *e) Q_DECL_OVERRIDE;
#endif

    virtual QMimeData *createMimeDataFromSelection() const;
    virtual bool canInsertFromMimeData(const QMimeData *source) const;
    virtual void insertFromMimeData(const QMimeData *source);

    virtual void inputMethodEvent(QInputMethodEvent *) Q_DECL_OVERRIDE;

    QPlainTextEdit(QPlainTextEditPrivate &dd, QWidget *parent);

    virtual void scrollContentsBy(int dx, int dy) Q_DECL_OVERRIDE;
    virtual void doSetTextCursor(const QTextCursor &cursor);

    QTextBlock firstVisibleBlock() const;
    QPointF contentOffset() const;
    QRectF blockBoundingRect(const QTextBlock &block) const;
    QRectF blockBoundingGeometry(const QTextBlock &block) const;
    QAbstractTextDocumentLayout::PaintContext getPaintContext() const;

    void zoomInF(float range);

private:
    Q_DISABLE_COPY(QPlainTextEdit)
    Q_PRIVATE_SLOT(d_func(), void _q_repaintContents(const QRectF &r))
    Q_PRIVATE_SLOT(d_func(), void _q_adjustScrollbars())
    Q_PRIVATE_SLOT(d_func(), void _q_verticalScrollbarActionTriggered(int))
    Q_PRIVATE_SLOT(d_func(), void _q_cursorPositionChanged())

    friend class QPlainTextEditControl;
};


class QPlainTextDocumentLayoutPrivate;
class Q_WIDGETS_EXPORT QPlainTextDocumentLayout : public QAbstractTextDocumentLayout
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QPlainTextDocumentLayout)
    Q_PROPERTY(int cursorWidth READ cursorWidth WRITE setCursorWidth)

public:
    QPlainTextDocumentLayout(QTextDocument *document);
    ~QPlainTextDocumentLayout();

    void draw(QPainter *, const PaintContext &) Q_DECL_OVERRIDE;
    int hitTest(const QPointF &, Qt::HitTestAccuracy ) const Q_DECL_OVERRIDE;

    int pageCount() const Q_DECL_OVERRIDE;
    QSizeF documentSize() const Q_DECL_OVERRIDE;

    QRectF frameBoundingRect(QTextFrame *) const Q_DECL_OVERRIDE;
    QRectF blockBoundingRect(const QTextBlock &block) const Q_DECL_OVERRIDE;

    void ensureBlockLayout(const QTextBlock &block) const;

    void setCursorWidth(int width);
    int cursorWidth() const;

    void requestUpdate();

protected:
    void documentChanged(int from, int /*charsRemoved*/, int charsAdded) Q_DECL_OVERRIDE;


private:
    void setTextWidth(qreal newWidth);
    qreal textWidth() const;
    void layoutBlock(const QTextBlock &block);
    qreal blockWidth(const QTextBlock &block);

    QPlainTextDocumentLayoutPrivate *priv() const;

    friend class QPlainTextEdit;
    friend class QPlainTextEditPrivate;
};

QT_END_NAMESPACE


#endif // QT_NO_TEXTEDIT

#endif // QPLAINTEXTEDIT_H
