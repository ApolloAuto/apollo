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

#ifndef QQUICKTEXTINPUT_P_H
#define QQUICKTEXTINPUT_P_H

#include "qquickimplicitsizeitem_p.h"
#include <QtGui/qtextoption.h>
#include <QtGui/qvalidator.h>

QT_BEGIN_NAMESPACE

class QQuickTextInputPrivate;
class QValidator;
class Q_QUICK_PRIVATE_EXPORT QQuickTextInput : public QQuickImplicitSizeItem
{
    Q_OBJECT
    Q_ENUMS(HAlignment)
    Q_ENUMS(VAlignment)
    Q_ENUMS(WrapMode)
    Q_ENUMS(EchoMode)
    Q_ENUMS(SelectionMode)
    Q_ENUMS(CursorPosition)
    Q_ENUMS(RenderType)

    Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)
    Q_PROPERTY(int length READ length NOTIFY textChanged)
    Q_PROPERTY(QColor color READ color WRITE setColor NOTIFY colorChanged)
    Q_PROPERTY(QColor selectionColor READ selectionColor WRITE setSelectionColor NOTIFY selectionColorChanged)
    Q_PROPERTY(QColor selectedTextColor READ selectedTextColor WRITE setSelectedTextColor NOTIFY selectedTextColorChanged)
    Q_PROPERTY(QFont font READ font WRITE setFont NOTIFY fontChanged)
    Q_PROPERTY(HAlignment horizontalAlignment READ hAlign WRITE setHAlign RESET resetHAlign NOTIFY horizontalAlignmentChanged)
    Q_PROPERTY(HAlignment effectiveHorizontalAlignment READ effectiveHAlign NOTIFY effectiveHorizontalAlignmentChanged)
    Q_PROPERTY(VAlignment verticalAlignment READ vAlign WRITE setVAlign NOTIFY verticalAlignmentChanged)
    Q_PROPERTY(WrapMode wrapMode READ wrapMode WRITE setWrapMode NOTIFY wrapModeChanged)

    Q_PROPERTY(bool readOnly READ isReadOnly WRITE setReadOnly NOTIFY readOnlyChanged)
    Q_PROPERTY(bool cursorVisible READ isCursorVisible WRITE setCursorVisible NOTIFY cursorVisibleChanged)
    Q_PROPERTY(int cursorPosition READ cursorPosition WRITE setCursorPosition NOTIFY cursorPositionChanged)
    Q_PROPERTY(QRectF cursorRectangle READ cursorRectangle NOTIFY cursorRectangleChanged)
    Q_PROPERTY(QQmlComponent *cursorDelegate READ cursorDelegate WRITE setCursorDelegate NOTIFY cursorDelegateChanged)
    Q_PROPERTY(int selectionStart READ selectionStart NOTIFY selectionStartChanged)
    Q_PROPERTY(int selectionEnd READ selectionEnd NOTIFY selectionEndChanged)
    Q_PROPERTY(QString selectedText READ selectedText NOTIFY selectedTextChanged)

    Q_PROPERTY(int maximumLength READ maxLength WRITE setMaxLength NOTIFY maximumLengthChanged)
    Q_PROPERTY(QValidator* validator READ validator WRITE setValidator NOTIFY validatorChanged)
    Q_PROPERTY(QString inputMask READ inputMask WRITE setInputMask NOTIFY inputMaskChanged)
    Q_PROPERTY(Qt::InputMethodHints inputMethodHints READ inputMethodHints WRITE setInputMethodHints NOTIFY inputMethodHintsChanged)

    Q_PROPERTY(bool acceptableInput READ hasAcceptableInput NOTIFY acceptableInputChanged)
    Q_PROPERTY(EchoMode echoMode READ echoMode WRITE setEchoMode NOTIFY echoModeChanged)
    Q_PROPERTY(bool activeFocusOnPress READ focusOnPress WRITE setFocusOnPress NOTIFY activeFocusOnPressChanged)
    Q_PROPERTY(QString passwordCharacter READ passwordCharacter WRITE setPasswordCharacter NOTIFY passwordCharacterChanged)
    Q_PROPERTY(int passwordMaskDelay READ passwordMaskDelay WRITE setPasswordMaskDelay RESET resetPasswordMaskDelay NOTIFY passwordMaskDelayChanged REVISION 3)
    Q_PROPERTY(QString displayText READ displayText NOTIFY displayTextChanged)
    Q_PROPERTY(bool autoScroll READ autoScroll WRITE setAutoScroll NOTIFY autoScrollChanged)
    Q_PROPERTY(bool selectByMouse READ selectByMouse WRITE setSelectByMouse NOTIFY selectByMouseChanged)
    Q_PROPERTY(SelectionMode mouseSelectionMode READ mouseSelectionMode WRITE setMouseSelectionMode NOTIFY mouseSelectionModeChanged)
    Q_PROPERTY(bool persistentSelection READ persistentSelection WRITE setPersistentSelection NOTIFY persistentSelectionChanged)
    Q_PROPERTY(bool canPaste READ canPaste NOTIFY canPasteChanged)
    Q_PROPERTY(bool canUndo READ canUndo NOTIFY canUndoChanged)
    Q_PROPERTY(bool canRedo READ canRedo NOTIFY canRedoChanged)
    Q_PROPERTY(bool inputMethodComposing READ isInputMethodComposing NOTIFY inputMethodComposingChanged)
    Q_PROPERTY(qreal contentWidth READ contentWidth NOTIFY contentSizeChanged)
    Q_PROPERTY(qreal contentHeight READ contentHeight NOTIFY contentSizeChanged)
    Q_PROPERTY(RenderType renderType READ renderType WRITE setRenderType NOTIFY renderTypeChanged)

public:
    QQuickTextInput(QQuickItem * parent=0);
    ~QQuickTextInput();

    void componentComplete() Q_DECL_OVERRIDE;

    enum EchoMode {//To match QLineEdit::EchoMode
        Normal,
        NoEcho,
        Password,
        PasswordEchoOnEdit
    };

    enum HAlignment {
        AlignLeft = Qt::AlignLeft,
        AlignRight = Qt::AlignRight,
        AlignHCenter = Qt::AlignHCenter
    };

    enum VAlignment {
        AlignTop = Qt::AlignTop,
        AlignBottom = Qt::AlignBottom,
        AlignVCenter = Qt::AlignVCenter
    };

    enum WrapMode {
        NoWrap = QTextOption::NoWrap,
        WordWrap = QTextOption::WordWrap,
        WrapAnywhere = QTextOption::WrapAnywhere,
        WrapAtWordBoundaryOrAnywhere = QTextOption::WrapAtWordBoundaryOrAnywhere, // COMPAT
        Wrap = QTextOption::WrapAtWordBoundaryOrAnywhere
    };

    enum SelectionMode {
        SelectCharacters,
        SelectWords
    };

    enum CursorPosition {
        CursorBetweenCharacters,
        CursorOnCharacter
    };

    enum RenderType { QtRendering,
                      NativeRendering
                    };

    //Auxilliary functions needed to control the TextInput from QML
    Q_INVOKABLE void positionAt(QQmlV4Function *args) const;
    Q_INVOKABLE QRectF positionToRectangle(int pos) const;
    Q_INVOKABLE void moveCursorSelection(int pos);
    Q_INVOKABLE void moveCursorSelection(int pos, SelectionMode mode);

    RenderType renderType() const;
    void setRenderType(RenderType renderType);

    QString text() const;
    void setText(const QString &);

    int length() const;

    QFont font() const;
    void setFont(const QFont &font);

    QColor color() const;
    void setColor(const QColor &c);

    QColor selectionColor() const;
    void setSelectionColor(const QColor &c);

    QColor selectedTextColor() const;
    void setSelectedTextColor(const QColor &c);

    HAlignment hAlign() const;
    void setHAlign(HAlignment align);
    void resetHAlign();
    HAlignment effectiveHAlign() const;

    VAlignment vAlign() const;
    void setVAlign(VAlignment align);

    WrapMode wrapMode() const;
    void setWrapMode(WrapMode w);

    bool isReadOnly() const;
    void setReadOnly(bool);

    bool isCursorVisible() const;
    void setCursorVisible(bool on);

    int cursorPosition() const;
    void setCursorPosition(int cp);

    QRectF cursorRectangle() const;

    int selectionStart() const;
    int selectionEnd() const;

    QString selectedText() const;

    int maxLength() const;
    void setMaxLength(int ml);

    QValidator * validator() const;
    void setValidator(QValidator* v);

    QString inputMask() const;
    void setInputMask(const QString &im);

    EchoMode echoMode() const;
    void setEchoMode(EchoMode echo);

    QString passwordCharacter() const;
    void setPasswordCharacter(const QString &str);

    int passwordMaskDelay() const;
    void setPasswordMaskDelay(int delay);
    void resetPasswordMaskDelay();

    QString displayText() const;

    QQmlComponent* cursorDelegate() const;
    void setCursorDelegate(QQmlComponent*);

    bool focusOnPress() const;
    void setFocusOnPress(bool);

    bool autoScroll() const;
    void setAutoScroll(bool);

    bool selectByMouse() const;
    void setSelectByMouse(bool);

    SelectionMode mouseSelectionMode() const;
    void setMouseSelectionMode(SelectionMode mode);

    bool persistentSelection() const;
    void setPersistentSelection(bool persist);

    bool hasAcceptableInput() const;

#ifndef QT_NO_IM
    QVariant inputMethodQuery(Qt::InputMethodQuery property) const Q_DECL_OVERRIDE;
    Q_REVISION(3) Q_INVOKABLE QVariant inputMethodQuery(Qt::InputMethodQuery query, QVariant argument) const;
#endif

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    QRectF clipRect() const Q_DECL_OVERRIDE;

    bool canPaste() const;

    bool canUndo() const;
    bool canRedo() const;

    bool isInputMethodComposing() const;

    Qt::InputMethodHints inputMethodHints() const;
    void setInputMethodHints(Qt::InputMethodHints hints);

    Q_INVOKABLE QString getText(int start, int end) const;

    qreal contentWidth() const;
    qreal contentHeight() const;

Q_SIGNALS:
    void textChanged();
    void cursorPositionChanged();
    void cursorRectangleChanged();
    void selectionStartChanged();
    void selectionEndChanged();
    void selectedTextChanged();
    void accepted();
    void acceptableInputChanged();
    Q_REVISION(2) void editingFinished();
    void colorChanged();
    void selectionColorChanged();
    void selectedTextColorChanged();
    void fontChanged(const QFont &font);
    void horizontalAlignmentChanged(HAlignment alignment);
    void verticalAlignmentChanged(VAlignment alignment);
    void wrapModeChanged();
    void readOnlyChanged(bool isReadOnly);
    void cursorVisibleChanged(bool isCursorVisible);
    void cursorDelegateChanged();
    void maximumLengthChanged(int maximumLength);
    void validatorChanged();
    void inputMaskChanged(const QString &inputMask);
    void echoModeChanged(EchoMode echoMode);
    void passwordCharacterChanged();
    Q_REVISION(3) void passwordMaskDelayChanged(int delay);
    void displayTextChanged();
    void activeFocusOnPressChanged(bool activeFocusOnPress);
    void autoScrollChanged(bool autoScroll);
    void selectByMouseChanged(bool selectByMouse);
    void mouseSelectionModeChanged(SelectionMode mode);
    void persistentSelectionChanged();
    void canPasteChanged();
    void canUndoChanged();
    void canRedoChanged();
    void inputMethodComposingChanged();
    void effectiveHorizontalAlignmentChanged();
    void contentSizeChanged();
    void inputMethodHintsChanged();
    void renderTypeChanged();

private:
    void invalidateFontCaches();
    void ensureActiveFocus();

protected:
    void geometryChanged(const QRectF &newGeometry,
                                 const QRectF &oldGeometry) Q_DECL_OVERRIDE;

    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* ev) Q_DECL_OVERRIDE;
#ifndef QT_NO_IM
    void inputMethodEvent(QInputMethodEvent *) Q_DECL_OVERRIDE;
#endif
    void mouseUngrabEvent() Q_DECL_OVERRIDE;
    bool event(QEvent *e) Q_DECL_OVERRIDE;
    void focusOutEvent(QFocusEvent *event) Q_DECL_OVERRIDE;
    void focusInEvent(QFocusEvent *event) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
    QSGNode *updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *data) Q_DECL_OVERRIDE;
    void updatePolish() Q_DECL_OVERRIDE;

public Q_SLOTS:
    void selectAll();
    void selectWord();
    void select(int start, int end);
    void deselect();
    bool isRightToLeft(int start, int end);
#ifndef QT_NO_CLIPBOARD
    void cut();
    void copy();
    void paste();
#endif
    void undo();
    void redo();
    void insert(int position, const QString &text);
    void remove(int start, int end);
    Q_REVISION(3) void ensureVisible(int position);

private Q_SLOTS:
    void selectionChanged();
    void createCursor();
    void updateCursorRectangle(bool scroll = true);
    void q_canPasteChanged();
    void q_updateAlignment();
    void triggerPreprocess();

#ifndef QT_NO_VALIDATOR
    void q_validatorChanged();
#endif

private:
    friend class QQuickTextUtil;

    Q_DECLARE_PRIVATE(QQuickTextInput)
};

#ifndef QT_NO_VALIDATOR
class Q_AUTOTEST_EXPORT QQuickIntValidator : public QIntValidator
{
    Q_OBJECT
    Q_PROPERTY(QString locale READ localeName WRITE setLocaleName RESET resetLocaleName NOTIFY localeNameChanged)
public:
    QQuickIntValidator(QObject *parent = 0);

    QString localeName() const;
    void setLocaleName(const QString &name);
    void resetLocaleName();

Q_SIGNALS:
    void localeNameChanged();
};

class Q_AUTOTEST_EXPORT QQuickDoubleValidator : public QDoubleValidator
{
    Q_OBJECT
    Q_PROPERTY(QString locale READ localeName WRITE setLocaleName RESET resetLocaleName NOTIFY localeNameChanged)
public:
    QQuickDoubleValidator(QObject *parent = 0);

    QString localeName() const;
    void setLocaleName(const QString &name);
    void resetLocaleName();

Q_SIGNALS:
    void localeNameChanged();
};
#endif

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickTextInput)
#ifndef QT_NO_VALIDATOR
QML_DECLARE_TYPE(QValidator)
QML_DECLARE_TYPE(QQuickIntValidator)
QML_DECLARE_TYPE(QQuickDoubleValidator)
QML_DECLARE_TYPE(QRegExpValidator)
#endif

#endif // QQUICKTEXTINPUT_P_H
