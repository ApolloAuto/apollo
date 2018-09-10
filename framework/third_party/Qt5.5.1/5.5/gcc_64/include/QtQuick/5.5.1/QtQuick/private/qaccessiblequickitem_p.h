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

#ifndef QACCESSIBLEQUICKITEM_H
#define QACCESSIBLEQUICKITEM_H

#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickView>
#include <QtGui/qaccessibleobject.h>

QT_BEGIN_NAMESPACE

#ifndef QT_NO_ACCESSIBILITY

class QTextDocument;

class QAccessibleQuickItem : public QAccessibleObject, public QAccessibleActionInterface, public QAccessibleValueInterface, public QAccessibleTextInterface
{
public:
    QAccessibleQuickItem(QQuickItem *item);

    QRect rect() const;
    QRect viewRect() const;

    bool clipsChildren() const;
    QAccessibleInterface *childAt(int x, int y) const;

    QAccessibleInterface *parent() const;
    QAccessibleInterface *child(int index) const;
    int childCount() const;
    int indexOfChild(const QAccessibleInterface *iface) const;
    QList<QQuickItem *> childItems() const;

    QAccessible::State state() const;
    QAccessible::Role role() const;
    QString text(QAccessible::Text) const;

    bool isAccessible() const;

    // Action Interface
    QStringList actionNames() const;
    void doAction(const QString &actionName);
    QStringList keyBindingsForAction(const QString &actionName) const;

    // Value Interface
    QVariant currentValue() const;
    void setCurrentValue(const QVariant &value);
    QVariant maximumValue() const;
    QVariant minimumValue() const;
    QVariant minimumStepSize() const;


    // Text Interface
    void selection(int selectionIndex, int *startOffset, int *endOffset) const;
    int selectionCount() const;
    void addSelection(int startOffset, int endOffset);
    void removeSelection(int selectionIndex);
    void setSelection(int selectionIndex, int startOffset, int endOffset);

    // cursor
    int cursorPosition() const;
    void setCursorPosition(int position);

    // text
    QString text(int startOffset, int endOffset) const;
    QString textBeforeOffset(int offset, QAccessible::TextBoundaryType boundaryType,
                                     int *startOffset, int *endOffset) const;
    QString textAfterOffset(int offset, QAccessible::TextBoundaryType boundaryType,
                                    int *startOffset, int *endOffset) const;
    QString textAtOffset(int offset, QAccessible::TextBoundaryType boundaryType,
                                 int *startOffset, int *endOffset) const;
    int characterCount() const;

    // character <-> geometry
    QRect characterRect(int /* offset */) const { return QRect(); }
    int offsetAtPoint(const QPoint & /* point */) const { return -1; }

    void scrollToSubstring(int /* startIndex */, int /* endIndex */) {}
    QString attributes(int /* offset */, int *startOffset, int *endOffset) const { *startOffset = 0; *endOffset = 0; return QString(); }

    QTextDocument *textDocument() const;

protected:
    QQuickItem *item() const { return static_cast<QQuickItem*>(object()); }
    void *interface_cast(QAccessible::InterfaceType t);

private:
    QTextDocument *m_doc;
};

QRect itemScreenRect(QQuickItem *item);
QList<QQuickItem *> accessibleUnignoredChildren(QQuickItem *item, bool paintOrder = false);

#endif // QT_NO_ACCESSIBILITY

QT_END_NAMESPACE

#endif // QACCESSIBLEQUICKITEM_H
