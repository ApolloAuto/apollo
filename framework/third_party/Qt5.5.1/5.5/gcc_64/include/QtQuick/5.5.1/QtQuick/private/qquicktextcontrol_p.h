/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QQUICKTEXTCONTROL_P_H
#define QQUICKTEXTCONTROL_P_H

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

#include <QtGui/qtextdocument.h>
#include <QtGui/qtextoption.h>
#include <QtGui/qtextcursor.h>
#include <QtGui/qtextformat.h>
#include <QtCore/qrect.h>
#include <QtGui/qabstracttextdocumentlayout.h>
#include <QtGui/qtextdocumentfragment.h>
#include <QtGui/qclipboard.h>
#include <QtCore/qmimedata.h>

QT_BEGIN_NAMESPACE


class QStyleSheet;
class QTextDocument;
class QQuickTextControlPrivate;
class QAbstractScrollArea;
class QEvent;
class QTimerEvent;

class Q_AUTOTEST_EXPORT QQuickTextControl : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickTextControl)
public:
    explicit QQuickTextControl(QTextDocument *doc, QObject *parent = 0);
    virtual ~QQuickTextControl();

    QTextDocument *document() const;

    void setTextCursor(const QTextCursor &cursor);
    QTextCursor textCursor() const;

    void setTextInteractionFlags(Qt::TextInteractionFlags flags);
    Qt::TextInteractionFlags textInteractionFlags() const;

    QString toPlainText() const;

#ifndef QT_NO_TEXTHTMLPARSER
    QString toHtml() const;
#endif

    bool hasImState() const;
    bool cursorVisible() const;
    void setCursorVisible(bool visible);
    QRectF cursorRect(const QTextCursor &cursor) const;
    QRectF cursorRect() const;
    QRectF selectionRect(const QTextCursor &cursor) const;
    QRectF selectionRect() const;

    QString hoveredLink() const;
    QString anchorAt(const QPointF &pos) const;

    void setCursorWidth(int width);

    void setAcceptRichText(bool accept);

    void moveCursor(QTextCursor::MoveOperation op, QTextCursor::MoveMode mode = QTextCursor::MoveAnchor);

    bool canPaste() const;

    void setCursorIsFocusIndicator(bool b);
    void setWordSelectionEnabled(bool enabled);

    void updateCursorRectangle(bool force);

    virtual int hitTest(const QPointF &point, Qt::HitTestAccuracy accuracy) const;
    virtual QRectF blockBoundingRect(const QTextBlock &block) const;

public Q_SLOTS:
    void setPlainText(const QString &text);
    void setHtml(const QString &text);

#ifndef QT_NO_CLIPBOARD
    void cut();
    void copy();
    void paste(QClipboard::Mode mode = QClipboard::Clipboard);
#endif

    void undo();
    void redo();

    void selectAll();

Q_SIGNALS:
    void textChanged();
    void contentsChange(int from, int charsRemoved, int charsAdded);
    void undoAvailable(bool b);
    void redoAvailable(bool b);
    void currentCharFormatChanged(const QTextCharFormat &format);
    void copyAvailable(bool b);
    void selectionChanged();
    void cursorPositionChanged();

    // control signals
    void updateCursorRequest();
    void updateRequest();
    void cursorRectangleChanged();
    void linkActivated(const QString &link);
    void linkHovered(const QString &link);

public:
    virtual void processEvent(QEvent *e, const QMatrix &matrix);
    void processEvent(QEvent *e, const QPointF &coordinateOffset = QPointF());

#ifndef QT_NO_IM
    virtual QVariant inputMethodQuery(Qt::InputMethodQuery property) const;
    Q_INVOKABLE QVariant inputMethodQuery(Qt::InputMethodQuery query, QVariant argument) const;
#endif

    virtual QMimeData *createMimeDataFromSelection() const;
    virtual bool canInsertFromMimeData(const QMimeData *source) const;
    virtual void insertFromMimeData(const QMimeData *source);

    bool cursorOn() const;

protected:
    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;

    bool event(QEvent *e) Q_DECL_OVERRIDE;

private:
    Q_DISABLE_COPY(QQuickTextControl)
    Q_PRIVATE_SLOT(d_func(), void _q_updateCurrentCharFormatAndSelection())
    Q_PRIVATE_SLOT(d_func(), void _q_updateCursorPosChanged(const QTextCursor &))
};


// also used by QLabel
class QQuickTextEditMimeData : public QMimeData
{
public:
    inline QQuickTextEditMimeData(const QTextDocumentFragment &aFragment) : fragment(aFragment) {}

    virtual QStringList formats() const;
protected:
    virtual QVariant retrieveData(const QString &mimeType, QVariant::Type type) const;
private:
    void setup() const;

    mutable QTextDocumentFragment fragment;
};

QT_END_NAMESPACE

#endif // QQuickTextControl_H
