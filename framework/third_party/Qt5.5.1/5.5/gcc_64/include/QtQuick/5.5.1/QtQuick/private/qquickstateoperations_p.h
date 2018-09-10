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

#ifndef QQUICKSTATEOPERATIONS_P_H
#define QQUICKSTATEOPERATIONS_P_H

#include "qquickitem.h"
#include "qquickanchors_p.h"

#include <QtQuick/private/qquickstate_p.h>

#include <QtQml/qqmlscriptstring.h>

QT_BEGIN_NAMESPACE

class QQuickParentChangePrivate;
class Q_AUTOTEST_EXPORT QQuickParentChange : public QQuickStateOperation, public QQuickStateActionEvent
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickParentChange)

    Q_PROPERTY(QQuickItem *target READ object WRITE setObject)
    Q_PROPERTY(QQuickItem *parent READ parent WRITE setParent)
    Q_PROPERTY(QQmlScriptString x READ x WRITE setX)
    Q_PROPERTY(QQmlScriptString y READ y WRITE setY)
    Q_PROPERTY(QQmlScriptString width READ width WRITE setWidth)
    Q_PROPERTY(QQmlScriptString height READ height WRITE setHeight)
    Q_PROPERTY(QQmlScriptString scale READ scale WRITE setScale)
    Q_PROPERTY(QQmlScriptString rotation READ rotation WRITE setRotation)
public:
    QQuickParentChange(QObject *parent=0);
    ~QQuickParentChange();

    QQuickItem *object() const;
    void setObject(QQuickItem *);

    QQuickItem *parent() const;
    void setParent(QQuickItem *);

    QQuickItem *originalParent() const;

    QQmlScriptString x() const;
    void setX(QQmlScriptString x);
    bool xIsSet() const;

    QQmlScriptString y() const;
    void setY(QQmlScriptString y);
    bool yIsSet() const;

    QQmlScriptString width() const;
    void setWidth(QQmlScriptString width);
    bool widthIsSet() const;

    QQmlScriptString height() const;
    void setHeight(QQmlScriptString height);
    bool heightIsSet() const;

    QQmlScriptString scale() const;
    void setScale(QQmlScriptString scale);
    bool scaleIsSet() const;

    QQmlScriptString rotation() const;
    void setRotation(QQmlScriptString rotation);
    bool rotationIsSet() const;

    ActionList actions() Q_DECL_OVERRIDE;

    void saveOriginals() Q_DECL_OVERRIDE;
    //virtual void copyOriginals(QQuickStateActionEvent*);
    void execute(Reason reason = ActualChange) Q_DECL_OVERRIDE;
    bool isReversable() Q_DECL_OVERRIDE;
    void reverse(Reason reason = ActualChange) Q_DECL_OVERRIDE;
    EventType type() const Q_DECL_OVERRIDE;
    bool override(QQuickStateActionEvent*other) Q_DECL_OVERRIDE;
    void rewind() Q_DECL_OVERRIDE;
    void saveCurrentValues() Q_DECL_OVERRIDE;
};

class QQuickAnchorChanges;
class QQuickAnchorSetPrivate;
class Q_AUTOTEST_EXPORT QQuickAnchorSet : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QQmlScriptString left READ left WRITE setLeft RESET resetLeft)
    Q_PROPERTY(QQmlScriptString right READ right WRITE setRight RESET resetRight)
    Q_PROPERTY(QQmlScriptString horizontalCenter READ horizontalCenter WRITE setHorizontalCenter RESET resetHorizontalCenter)
    Q_PROPERTY(QQmlScriptString top READ top WRITE setTop RESET resetTop)
    Q_PROPERTY(QQmlScriptString bottom READ bottom WRITE setBottom RESET resetBottom)
    Q_PROPERTY(QQmlScriptString verticalCenter READ verticalCenter WRITE setVerticalCenter RESET resetVerticalCenter)
    Q_PROPERTY(QQmlScriptString baseline READ baseline WRITE setBaseline RESET resetBaseline)

public:
    QQuickAnchorSet(QObject *parent=0);
    virtual ~QQuickAnchorSet();

    QQmlScriptString left() const;
    void setLeft(const QQmlScriptString &edge);
    void resetLeft();

    QQmlScriptString right() const;
    void setRight(const QQmlScriptString &edge);
    void resetRight();

    QQmlScriptString horizontalCenter() const;
    void setHorizontalCenter(const QQmlScriptString &edge);
    void resetHorizontalCenter();

    QQmlScriptString top() const;
    void setTop(const QQmlScriptString &edge);
    void resetTop();

    QQmlScriptString bottom() const;
    void setBottom(const QQmlScriptString &edge);
    void resetBottom();

    QQmlScriptString verticalCenter() const;
    void setVerticalCenter(const QQmlScriptString &edge);
    void resetVerticalCenter();

    QQmlScriptString baseline() const;
    void setBaseline(const QQmlScriptString &edge);
    void resetBaseline();

    QQuickAnchors::Anchors usedAnchors() const;

private:
    friend class QQuickAnchorChanges;
    Q_DISABLE_COPY(QQuickAnchorSet)
    Q_DECLARE_PRIVATE(QQuickAnchorSet)
};

class QQuickAnchorChangesPrivate;
class Q_AUTOTEST_EXPORT QQuickAnchorChanges : public QQuickStateOperation, public QQuickStateActionEvent
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickAnchorChanges)

    Q_PROPERTY(QQuickItem *target READ object WRITE setObject)
    Q_PROPERTY(QQuickAnchorSet *anchors READ anchors CONSTANT)

public:
    QQuickAnchorChanges(QObject *parent=0);
    ~QQuickAnchorChanges();

    ActionList actions() Q_DECL_OVERRIDE;

    QQuickAnchorSet *anchors();

    QQuickItem *object() const;
    void setObject(QQuickItem *);

    void execute(Reason reason = ActualChange) Q_DECL_OVERRIDE;
    bool isReversable() Q_DECL_OVERRIDE;
    void reverse(Reason reason = ActualChange) Q_DECL_OVERRIDE;
    EventType type() const Q_DECL_OVERRIDE;
    bool override(QQuickStateActionEvent*other) Q_DECL_OVERRIDE;
    bool changesBindings() Q_DECL_OVERRIDE;
    void saveOriginals() Q_DECL_OVERRIDE;
    bool needsCopy() Q_DECL_OVERRIDE { return true; }
    void copyOriginals(QQuickStateActionEvent*) Q_DECL_OVERRIDE;
    void clearBindings() Q_DECL_OVERRIDE;
    void rewind() Q_DECL_OVERRIDE;
    void saveCurrentValues() Q_DECL_OVERRIDE;

    QList<QQuickStateAction> additionalActions();
    void saveTargetValues() Q_DECL_OVERRIDE;
};

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickParentChange)
QML_DECLARE_TYPE(QQuickAnchorSet)
QML_DECLARE_TYPE(QQuickAnchorChanges)

#endif // QQUICKSTATEOPERATIONS_P_H

