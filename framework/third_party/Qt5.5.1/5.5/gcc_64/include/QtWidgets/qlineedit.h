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

#ifndef QLINEEDIT_H
#define QLINEEDIT_H

#include <QtWidgets/qframe.h>
#include <QtGui/qtextcursor.h>
#include <QtCore/qstring.h>
#include <QtCore/qmargins.h>

QT_BEGIN_NAMESPACE


#ifndef QT_NO_LINEEDIT

class QValidator;
class QMenu;
class QLineEditPrivate;
class QCompleter;
class QStyleOptionFrame;
class QAbstractSpinBox;
class QDateTimeEdit;
class QIcon;
class QToolButton;

class Q_WIDGETS_EXPORT QLineEdit : public QWidget
{
    Q_OBJECT

    Q_PROPERTY(QString inputMask READ inputMask WRITE setInputMask)
    Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged USER true)
    Q_PROPERTY(int maxLength READ maxLength WRITE setMaxLength)
    Q_PROPERTY(bool frame READ hasFrame WRITE setFrame)
    Q_PROPERTY(EchoMode echoMode READ echoMode WRITE setEchoMode)
    Q_PROPERTY(QString displayText READ displayText)
    Q_PROPERTY(int cursorPosition READ cursorPosition WRITE setCursorPosition)
    Q_PROPERTY(Qt::Alignment alignment READ alignment WRITE setAlignment)
    Q_PROPERTY(bool modified READ isModified WRITE setModified DESIGNABLE false)
    Q_PROPERTY(bool hasSelectedText READ hasSelectedText)
    Q_PROPERTY(QString selectedText READ selectedText)
    Q_PROPERTY(bool dragEnabled READ dragEnabled WRITE setDragEnabled)
    Q_PROPERTY(bool readOnly READ isReadOnly WRITE setReadOnly)
    Q_PROPERTY(bool undoAvailable READ isUndoAvailable)
    Q_PROPERTY(bool redoAvailable READ isRedoAvailable)
    Q_PROPERTY(bool acceptableInput READ hasAcceptableInput)
    Q_PROPERTY(QString placeholderText READ placeholderText WRITE setPlaceholderText)
    Q_PROPERTY(Qt::CursorMoveStyle cursorMoveStyle READ cursorMoveStyle WRITE setCursorMoveStyle)
    Q_PROPERTY(bool clearButtonEnabled READ isClearButtonEnabled WRITE setClearButtonEnabled)
public:
    enum ActionPosition {
        LeadingPosition,
        TrailingPosition
    };
    Q_ENUM(ActionPosition)

    explicit QLineEdit(QWidget* parent=0);
    explicit QLineEdit(const QString &, QWidget* parent=0);
    ~QLineEdit();

    QString text() const;

    QString displayText() const;

    QString placeholderText() const;
    void setPlaceholderText(const QString &);

    int maxLength() const;
    void setMaxLength(int);

    void setFrame(bool);
    bool hasFrame() const;

    void setClearButtonEnabled(bool enable);
    bool isClearButtonEnabled() const;

    enum EchoMode { Normal, NoEcho, Password, PasswordEchoOnEdit };
    Q_ENUM(EchoMode)
    EchoMode echoMode() const;
    void setEchoMode(EchoMode);

    bool isReadOnly() const;
    void setReadOnly(bool);

#ifndef QT_NO_VALIDATOR
    void setValidator(const QValidator *);
    const QValidator * validator() const;
#endif

#ifndef QT_NO_COMPLETER
    void setCompleter(QCompleter *completer);
    QCompleter *completer() const;
#endif

    QSize sizeHint() const Q_DECL_OVERRIDE;
    QSize minimumSizeHint() const Q_DECL_OVERRIDE;

    int cursorPosition() const;
    void setCursorPosition(int);
    int cursorPositionAt(const QPoint &pos);

    void setAlignment(Qt::Alignment flag);
    Qt::Alignment alignment() const;

    void cursorForward(bool mark, int steps = 1);
    void cursorBackward(bool mark, int steps = 1);
    void cursorWordForward(bool mark);
    void cursorWordBackward(bool mark);
    void backspace();
    void del();
    void home(bool mark);
    void end(bool mark);

    bool isModified() const;
    void setModified(bool);

    void setSelection(int, int);
    bool hasSelectedText() const;
    QString selectedText() const;
    int selectionStart() const;

    bool isUndoAvailable() const;
    bool isRedoAvailable() const;

    void setDragEnabled(bool b);
    bool dragEnabled() const;

    void setCursorMoveStyle(Qt::CursorMoveStyle style);
    Qt::CursorMoveStyle cursorMoveStyle() const;

    QString inputMask() const;
    void setInputMask(const QString &inputMask);
    bool hasAcceptableInput() const;

    void setTextMargins(int left, int top, int right, int bottom);
    void setTextMargins(const QMargins &margins);
    void getTextMargins(int *left, int *top, int *right, int *bottom) const;
    QMargins textMargins() const;

    using QWidget::addAction;
    void addAction(QAction *action, ActionPosition position);
    QAction *addAction(const QIcon &icon, ActionPosition position);

public Q_SLOTS:
    void setText(const QString &);
    void clear();
    void selectAll();
    void undo();
    void redo();
#ifndef QT_NO_CLIPBOARD
    void cut();
    void copy() const;
    void paste();
#endif

public:
    void deselect();
    void insert(const QString &);
#ifndef QT_NO_CONTEXTMENU
    QMenu *createStandardContextMenu();
#endif

Q_SIGNALS:
    void textChanged(const QString &);
    void textEdited(const QString &);
    void cursorPositionChanged(int, int);
    void returnPressed();
    void editingFinished();
    void selectionChanged();

protected:
    void mousePressEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *) Q_DECL_OVERRIDE;
    void focusInEvent(QFocusEvent *) Q_DECL_OVERRIDE;
    void focusOutEvent(QFocusEvent *) Q_DECL_OVERRIDE;
    void paintEvent(QPaintEvent *) Q_DECL_OVERRIDE;
#ifndef QT_NO_DRAGANDDROP
    void dragEnterEvent(QDragEnterEvent *) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *e) Q_DECL_OVERRIDE;
    void dragLeaveEvent(QDragLeaveEvent *e) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *) Q_DECL_OVERRIDE;
#endif
    void changeEvent(QEvent *) Q_DECL_OVERRIDE;
#ifndef QT_NO_CONTEXTMENU
    void contextMenuEvent(QContextMenuEvent *) Q_DECL_OVERRIDE;
#endif

    void inputMethodEvent(QInputMethodEvent *) Q_DECL_OVERRIDE;
    void initStyleOption(QStyleOptionFrame *option) const;
public:
    QVariant inputMethodQuery(Qt::InputMethodQuery) const Q_DECL_OVERRIDE;
    bool event(QEvent *) Q_DECL_OVERRIDE;
protected:
    QRect cursorRect() const;

public:

private:
    friend class QAbstractSpinBox;
    friend class QAccessibleLineEdit;
#ifdef QT_KEYPAD_NAVIGATION
    friend class QDateTimeEdit;
#endif
    Q_DISABLE_COPY(QLineEdit)
    Q_DECLARE_PRIVATE(QLineEdit)
    Q_PRIVATE_SLOT(d_func(), void _q_handleWindowActivate())
    Q_PRIVATE_SLOT(d_func(), void _q_textEdited(const QString &))
    Q_PRIVATE_SLOT(d_func(), void _q_cursorPositionChanged(int, int))
#ifndef QT_NO_COMPLETER
    Q_PRIVATE_SLOT(d_func(), void _q_completionHighlighted(const QString &))
#endif
#ifdef QT_KEYPAD_NAVIGATION
    Q_PRIVATE_SLOT(d_func(), void _q_editFocusChange(bool))
#endif
    Q_PRIVATE_SLOT(d_func(), void _q_selectionChanged())
    Q_PRIVATE_SLOT(d_func(), void _q_updateNeeded(const QRect &))
    Q_PRIVATE_SLOT(d_func(), void _q_textChanged(const QString &))
    Q_PRIVATE_SLOT(d_func(), void _q_clearButtonClicked())
};

#endif // QT_NO_LINEEDIT

QT_END_NAMESPACE

#endif // QLINEEDIT_H
