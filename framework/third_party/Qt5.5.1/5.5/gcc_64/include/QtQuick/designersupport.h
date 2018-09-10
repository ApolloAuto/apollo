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

#ifndef DESIGNERSUPPORT_H
#define DESIGNERSUPPORT_H

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


#include <QtQuick/qtquickglobal.h>
#include <QtCore/QtGlobal>
#include <QtCore/QHash>
#include <QtCore/QRectF>

QT_BEGIN_NAMESPACE

class QQuickItem;
class QSGLayer;
class QImage;
class QTransform;
class QQmlContext;
class QQuickView;
class QObject;
class QQuickWindow;

class Q_QUICK_EXPORT DesignerSupport
{
public:
    enum DirtyType {
        TransformOrigin         = 0x00000001,
        Transform               = 0x00000002,
        BasicTransform          = 0x00000004,
        Position                = 0x00000008,
        Size                    = 0x00000010,

        ZValue                  = 0x00000020,
        Content                 = 0x00000040,
        Smooth                  = 0x00000080,
        OpacityValue            = 0x00000100,
        ChildrenChanged         = 0x00000200,
        ChildrenStackingChanged = 0x00000400,
        ParentChanged           = 0x00000800,

        Clip                    = 0x00001000,
        Window                  = 0x00002000,

        EffectReference         = 0x00008000,
        Visible                 = 0x00010000,
        HideReference           = 0x00020000,

        TransformUpdateMask     = TransformOrigin | Transform | BasicTransform | Position | Size | Window,
        ComplexTransformUpdateMask     = Transform | Window,
        ContentUpdateMask       = Size | Content | Smooth | Window,
        ChildrenUpdateMask      = ChildrenChanged | ChildrenStackingChanged | EffectReference | Window,
        AllMask                 = TransformUpdateMask | ContentUpdateMask | ChildrenUpdateMask
    };


    DesignerSupport();
    ~DesignerSupport();

    void refFromEffectItem(QQuickItem *referencedItem, bool hide = true);
    void derefFromEffectItem(QQuickItem *referencedItem, bool unhide = true);

    QImage renderImageForItem(QQuickItem *referencedItem, const QRectF &boundingRect, const QSize &imageSize);

    static bool isDirty(QQuickItem *referencedItem, DirtyType dirtyType);
    static void addDirty(QQuickItem *referencedItem, DirtyType dirtyType);
    static void resetDirty(QQuickItem *referencedItem);

    static QTransform windowTransform(QQuickItem *referencedItem);
    static QTransform parentTransform(QQuickItem *referencedItem);

    static bool isAnchoredTo(QQuickItem *fromItem, QQuickItem *toItem);
    static bool areChildrenAnchoredTo(QQuickItem *fromItem, QQuickItem *toItem);
    static bool hasAnchor(QQuickItem *item, const QString &name);
    static QQuickItem *anchorFillTargetItem(QQuickItem *item);
    static QQuickItem *anchorCenterInTargetItem(QQuickItem *item);
    static QPair<QString, QObject*> anchorLineTarget(QQuickItem *item, const QString &name, QQmlContext *context);
    static void resetAnchor(QQuickItem *item, const QString &name);
    static void emitComponentCompleteSignalForAttachedProperty(QQuickItem *item);


    static QList<QObject*> statesForItem(QQuickItem *item);

    static bool isComponentComplete(QQuickItem *item);

    static int borderWidth(QQuickItem *item);

    static void refreshExpressions(QQmlContext *context);

    static void setRootItem(QQuickView *view, QQuickItem *item);

    static bool isValidWidth(QQuickItem *item);
    static bool isValidHeight(QQuickItem *item);

    static void updateDirtyNode(QQuickItem *item);

    static void activateDesignerWindowManager();
    static void activateDesignerMode();

    static void disableComponentComplete();
    static void enableComponentComplete();

    static void createOpenGLContext(QQuickWindow *window);

    static void polishItems(QQuickWindow *window);

private:
    QHash<QQuickItem*, QSGLayer*> m_itemTextureHash;
};

QT_END_NAMESPACE

#endif // DESIGNERSUPPORT_H
