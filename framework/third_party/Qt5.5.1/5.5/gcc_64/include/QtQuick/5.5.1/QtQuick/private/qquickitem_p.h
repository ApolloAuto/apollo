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

#ifndef QQUICKITEM_P_H
#define QQUICKITEM_P_H

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

#include "qquickitem.h"

#include "qquickanchors_p.h"
#include "qquickanchors_p_p.h"
#include "qquickitemchangelistener_p.h"

#include "qquickwindow_p.h"

#include <QtQuick/qsgnode.h>
#include "qquickclipnode_p.h"

#include <private/qpodvector_p.h>
#include <QtQuick/private/qquickstate_p.h>
#include <private/qqmlnullablevalue_p.h>
#include <private/qqmlnotifier_p.h>
#include <private/qqmlglobal_p.h>
#include <private/qlazilyallocated_p.h>

#include <qqml.h>
#include <qqmlcontext.h>

#include <QtCore/qlist.h>
#include <QtCore/qdebug.h>
#include <QtCore/qelapsedtimer.h>

#include <QtQuick/private/qquickshadereffectsource_p.h>
#include <QtQuick/private/qquickshadereffect_p.h>

QT_BEGIN_NAMESPACE

class QNetworkReply;
class QQuickItemKeyFilter;
class QQuickLayoutMirroringAttached;
class QQuickScreenAttached;

class QQuickContents : public QQuickItemChangeListener
{
public:
    QQuickContents(QQuickItem *item);
    ~QQuickContents();

    QRectF rectF() const { return QRectF(m_x, m_y, m_width, m_height); }

    inline void calcGeometry(QQuickItem *changed = 0);
    void complete();

protected:
    void itemGeometryChanged(QQuickItem *item, const QRectF &newGeometry, const QRectF &oldGeometry) Q_DECL_OVERRIDE;
    void itemDestroyed(QQuickItem *item) Q_DECL_OVERRIDE;
    void itemChildAdded(QQuickItem *, QQuickItem *) Q_DECL_OVERRIDE;
    void itemChildRemoved(QQuickItem *, QQuickItem *) Q_DECL_OVERRIDE;
    //void itemVisibilityChanged(QQuickItem *item)

private:
    bool calcHeight(QQuickItem *changed = 0);
    bool calcWidth(QQuickItem *changed = 0);
    void updateRect();

    QQuickItem *m_item;
    qreal m_x;
    qreal m_y;
    qreal m_width;
    qreal m_height;
};

void QQuickContents::calcGeometry(QQuickItem *changed)
{
    bool wChanged = calcWidth(changed);
    bool hChanged = calcHeight(changed);
    if (wChanged || hChanged)
        updateRect();
}

class QQuickTransformPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QQuickTransform)
public:
    static QQuickTransformPrivate* get(QQuickTransform *transform) { return transform->d_func(); }

    QQuickTransformPrivate();

    QList<QQuickItem *> items;
};


class QQuickItemLayer : public QObject, public QQuickItemChangeListener
{
    Q_OBJECT
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(QSize textureSize READ size WRITE setSize NOTIFY sizeChanged)
    Q_PROPERTY(QRectF sourceRect READ sourceRect WRITE setSourceRect NOTIFY sourceRectChanged)
    Q_PROPERTY(bool mipmap READ mipmap WRITE setMipmap NOTIFY mipmapChanged)
    Q_PROPERTY(bool smooth READ smooth WRITE setSmooth NOTIFY smoothChanged)
    Q_PROPERTY(QQuickShaderEffectSource::WrapMode wrapMode READ wrapMode WRITE setWrapMode NOTIFY wrapModeChanged)
    Q_PROPERTY(QQuickShaderEffectSource::Format format READ format WRITE setFormat NOTIFY formatChanged)
    Q_PROPERTY(QByteArray samplerName READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(QQmlComponent *effect READ effect WRITE setEffect NOTIFY effectChanged)
public:
    QQuickItemLayer(QQuickItem *item);
    ~QQuickItemLayer();

    void classBegin();
    void componentComplete();

    bool enabled() const { return m_enabled; }
    void setEnabled(bool enabled);

    bool mipmap() const { return m_mipmap; }
    void setMipmap(bool mipmap);

    bool smooth() const { return m_smooth; }
    void setSmooth(bool s);

    QSize size() const { return m_size; }
    void setSize(const QSize &size);

    QQuickShaderEffectSource::Format format() const { return m_format; }
    void setFormat(QQuickShaderEffectSource::Format f);

    QRectF sourceRect() const { return m_sourceRect; }
    void setSourceRect(const QRectF &sourceRect);

    QQuickShaderEffectSource::WrapMode wrapMode() const { return m_wrapMode; }
    void setWrapMode(QQuickShaderEffectSource::WrapMode mode);

    QByteArray name() const { return m_name; }
    void setName(const QByteArray &name);

    QQmlComponent *effect() const { return m_effectComponent; }
    void setEffect(QQmlComponent *effect);

    QQuickShaderEffectSource *effectSource() const { return m_effectSource; }

    void itemGeometryChanged(QQuickItem *, const QRectF &, const QRectF &) Q_DECL_OVERRIDE;
    void itemOpacityChanged(QQuickItem *) Q_DECL_OVERRIDE;
    void itemParentChanged(QQuickItem *, QQuickItem *) Q_DECL_OVERRIDE;
    void itemSiblingOrderChanged(QQuickItem *) Q_DECL_OVERRIDE;
    void itemVisibilityChanged(QQuickItem *) Q_DECL_OVERRIDE;

    void updateMatrix();
    void updateGeometry();
    void updateOpacity();
    void updateZ();

Q_SIGNALS:
    void enabledChanged(bool enabled);
    void sizeChanged(const QSize &size);
    void mipmapChanged(bool mipmap);
    void wrapModeChanged(QQuickShaderEffectSource::WrapMode mode);
    void nameChanged(const QByteArray &name);
    void effectChanged(QQmlComponent *component);
    void smoothChanged(bool smooth);
    void formatChanged(QQuickShaderEffectSource::Format format);
    void sourceRectChanged(const QRectF &sourceRect);

private:
    friend class QQuickTransformAnimatorJob;
    friend class QQuickOpacityAnimatorJob;

    void activate();
    void deactivate();
    void activateEffect();
    void deactivateEffect();

    QQuickItem *m_item;
    bool m_enabled;
    bool m_mipmap;
    bool m_smooth;
    bool m_componentComplete;
    QQuickShaderEffectSource::WrapMode m_wrapMode;
    QQuickShaderEffectSource::Format m_format;
    QSize m_size;
    QRectF m_sourceRect;
    QByteArray m_name;
    QQmlComponent *m_effectComponent;
    QQuickItem *m_effect;
    QQuickShaderEffectSource *m_effectSource;
};

class Q_QUICK_PRIVATE_EXPORT QQuickItemPrivate : public QObjectPrivate
{
    Q_DECLARE_PUBLIC(QQuickItem)

public:
    static QQuickItemPrivate* get(QQuickItem *item) { return item->d_func(); }
    static const QQuickItemPrivate* get(const QQuickItem *item) { return item->d_func(); }

    static void registerAccessorProperties();

    QQuickItemPrivate();
    ~QQuickItemPrivate();
    void init(QQuickItem *parent);

    QQmlListProperty<QObject> data();
    QQmlListProperty<QObject> resources();
    QQmlListProperty<QQuickItem> children();
    QQmlListProperty<QQuickItem> visibleChildren();

    QQmlListProperty<QQuickState> states();
    QQmlListProperty<QQuickTransition> transitions();

    QString state() const;
    void setState(const QString &);

    QQuickAnchorLine left() const;
    QQuickAnchorLine right() const;
    QQuickAnchorLine horizontalCenter() const;
    QQuickAnchorLine top() const;
    QQuickAnchorLine bottom() const;
    QQuickAnchorLine verticalCenter() const;
    QQuickAnchorLine baseline() const;

    QQuickItemLayer *layer() const;

    // data property
    static void data_append(QQmlListProperty<QObject> *, QObject *);
    static int data_count(QQmlListProperty<QObject> *);
    static QObject *data_at(QQmlListProperty<QObject> *, int);
    static void data_clear(QQmlListProperty<QObject> *);

    // resources property
    static QObject *resources_at(QQmlListProperty<QObject> *, int);
    static void resources_append(QQmlListProperty<QObject> *, QObject *);
    static int resources_count(QQmlListProperty<QObject> *);
    static void resources_clear(QQmlListProperty<QObject> *);

    // children property
    static void children_append(QQmlListProperty<QQuickItem> *, QQuickItem *);
    static int children_count(QQmlListProperty<QQuickItem> *);
    static QQuickItem *children_at(QQmlListProperty<QQuickItem> *, int);
    static void children_clear(QQmlListProperty<QQuickItem> *);

    // visibleChildren property
    static void visibleChildren_append(QQmlListProperty<QQuickItem> *prop, QQuickItem *o);
    static int visibleChildren_count(QQmlListProperty<QQuickItem> *prop);
    static QQuickItem *visibleChildren_at(QQmlListProperty<QQuickItem> *prop, int index);

    // transform property
    static int transform_count(QQmlListProperty<QQuickTransform> *list);
    static void transform_append(QQmlListProperty<QQuickTransform> *list, QQuickTransform *);
    static QQuickTransform *transform_at(QQmlListProperty<QQuickTransform> *list, int);
    static void transform_clear(QQmlListProperty<QQuickTransform> *list);

    void _q_resourceObjectDeleted(QObject *);
    void _q_windowChanged(QQuickWindow *w);

    enum ChangeType {
        Geometry = 0x01,
        SiblingOrder = 0x02,
        Visibility = 0x04,
        Opacity = 0x08,
        Destroyed = 0x10,
        Parent = 0x20,
        Children = 0x40,
        Rotation = 0x80,
        ImplicitWidth = 0x100,
        ImplicitHeight = 0x200
    };

    Q_DECLARE_FLAGS(ChangeTypes, ChangeType)

    enum GeometryChangeType {
        NoChange = 0,
        XChange = 0x01,
        YChange = 0x02,
        WidthChange = 0x04,
        HeightChange = 0x08,
        SizeChange = WidthChange | HeightChange,
        GeometryChange = XChange | YChange | SizeChange
    };

    Q_DECLARE_FLAGS(GeometryChangeTypes, GeometryChangeType)

    struct ChangeListener {
        ChangeListener(QQuickItemChangeListener *l, QQuickItemPrivate::ChangeTypes t) : listener(l), types(t), gTypes(GeometryChange) {}
        ChangeListener(QQuickItemChangeListener *l, QQuickItemPrivate::GeometryChangeTypes gt) : listener(l), types(Geometry), gTypes(gt) {}
        QQuickItemChangeListener *listener;
        QQuickItemPrivate::ChangeTypes types;
        QQuickItemPrivate::GeometryChangeTypes gTypes;  //NOTE: not used for ==
        bool operator==(const ChangeListener &other) const { return listener == other.listener && types == other.types; }
    };

    struct ExtraData {
        ExtraData();

        qreal z;
        qreal scale;
        qreal rotation;
        qreal opacity;

        QQuickContents *contents;
        QQuickScreenAttached *screenAttached;
        QQuickLayoutMirroringAttached* layoutDirectionAttached;
        QQuickItemKeyFilter *keyHandler;
        mutable QQuickItemLayer *layer;
#ifndef QT_NO_CURSOR
        QCursor cursor;
#endif
        QPointF userTransformOriginPoint;

        int effectRefCount;
        int hideRefCount;

        QSGOpacityNode *opacityNode;
        QQuickDefaultClipNode *clipNode;
        QSGRootNode *rootNode;

        QObjectList resourcesList;

        // Although acceptedMouseButtons is inside ExtraData, we actually store
        // the LeftButton flag in the extra.flag() bit.  This is because it is
        // extremely common to set acceptedMouseButtons to LeftButton, but very
        // rare to use any of the other buttons.
        Qt::MouseButtons acceptedMouseButtons;

        QQuickItem::TransformOrigin origin:5;
        uint transparentForPositioner : 1;

        // 26 bits padding
    };
    QLazilyAllocated<ExtraData> extra;

    QQuickAnchors *anchors() const;
    mutable QQuickAnchors *_anchors;

    inline Qt::MouseButtons acceptedMouseButtons() const;

    QPODVector<QQuickItemPrivate::ChangeListener,4> changeListeners;

    void addItemChangeListener(QQuickItemChangeListener *listener, ChangeTypes types);
    void removeItemChangeListener(QQuickItemChangeListener *, ChangeTypes types);
    void updateOrAddGeometryChangeListener(QQuickItemChangeListener *listener, GeometryChangeTypes types);
    void updateOrRemoveGeometryChangeListener(QQuickItemChangeListener *listener, GeometryChangeTypes types);

    QQuickStateGroup *_states();
    QQuickStateGroup *_stateGroup;

    inline QQuickItem::TransformOrigin origin() const;

    // Bit 0
    quint32 flags:5;
    bool widthValid:1;
    bool heightValid:1;
    bool baselineOffsetValid:1;
    bool componentComplete:1;
    bool keepMouse:1;
    bool keepTouch:1;
    bool hoverEnabled:1;
    bool smooth:1;
    bool antialiasing:1;
    bool focus:1;
    bool activeFocus:1;
    // Bit 16
    bool notifiedFocus:1;
    bool notifiedActiveFocus:1;
    bool filtersChildMouseEvents:1;
    bool explicitVisible:1;
    bool effectiveVisible:1;
    bool explicitEnable:1;
    bool effectiveEnable:1;
    bool polishScheduled:1;
    bool inheritedLayoutMirror:1;
    bool effectiveLayoutMirror:1;
    bool isMirrorImplicit:1;
    bool inheritMirrorFromParent:1;
    bool inheritMirrorFromItem:1;
    bool isAccessible:1;
    bool culled:1;
    bool hasCursor:1;
    // Bit 32
    bool hasCursorInChild:1;
    bool activeFocusOnTab:1;
    bool implicitAntialiasing:1;
    bool antialiasingValid:1;

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
        Antialiasing             = 0x00040000,
        // When you add an attribute here, don't forget to update
        // dirtyToString()

        TransformUpdateMask     = TransformOrigin | Transform | BasicTransform | Position |
                                  Window,
        ComplexTransformUpdateMask     = Transform | Window,
        ContentUpdateMask       = Size | Content | Smooth | Window | Antialiasing,
        ChildrenUpdateMask      = ChildrenChanged | ChildrenStackingChanged | EffectReference | Window
    };

    quint32 dirtyAttributes;
    QString dirtyToString() const;
    void dirty(DirtyType);
    void addToDirtyList();
    void removeFromDirtyList();
    QQuickItem *nextDirtyItem;
    QQuickItem**prevDirtyItem;

    void setCulled(bool);

    QQuickWindow *window;
    int windowRefCount;
    inline QSGContext *sceneGraphContext() const;
    inline QSGRenderContext *sceneGraphRenderContext() const;

    QQuickItem *parentItem;
    QQmlNotifier parentNotifier;

    QList<QQuickItem *> childItems;
    mutable QList<QQuickItem *> *sortedChildItems;
    QList<QQuickItem *> paintOrderChildItems() const;
    void addChild(QQuickItem *);
    void removeChild(QQuickItem *);
    void siblingOrderChanged();

    inline void markSortedChildrenDirty(QQuickItem *child);

    void refWindow(QQuickWindow *);
    void derefWindow();

    QQuickItem *subFocusItem;
    void updateSubFocusItem(QQuickItem *scope, bool focus);

    QTransform windowToItemTransform() const;
    QTransform itemToWindowTransform() const;
    void itemToParentTransform(QTransform &) const;

    static bool focusNextPrev(QQuickItem *item, bool forward);
    static QQuickItem *nextPrevItemInTabFocusChain(QQuickItem *item, bool forward);

    static bool canAcceptTabFocus(QQuickItem *item);

    qreal x;
    qreal y;
    qreal width;
    qreal height;
    qreal implicitWidth;
    qreal implicitHeight;

    qreal baselineOffset;

    QList<QQuickTransform *> transforms;

    inline qreal z() const { return extra.isAllocated()?extra->z:0; }
    inline qreal scale() const { return extra.isAllocated()?extra->scale:1; }
    inline qreal rotation() const { return extra.isAllocated()?extra->rotation:0; }
    inline qreal opacity() const { return extra.isAllocated()?extra->opacity:1; }

    void setAccessible();

    virtual qreal getImplicitWidth() const;
    virtual qreal getImplicitHeight() const;
    virtual void implicitWidthChanged();
    virtual void implicitHeightChanged();

    void setImplicitAntialiasing(bool antialiasing);

    void resolveLayoutMirror();
    void setImplicitLayoutMirror(bool mirror, bool inherit);
    void setLayoutMirror(bool mirror);
    bool isMirrored() const {
        return effectiveLayoutMirror;
    }

    void emitChildrenRectChanged(const QRectF &rect) {
        Q_Q(QQuickItem);
        Q_EMIT q->childrenRectChanged(rect);
    }

    QPointF computeTransformOrigin() const;
    virtual void transformChanged();

    void deliverKeyEvent(QKeyEvent *);
#ifndef QT_NO_IM
    void deliverInputMethodEvent(QInputMethodEvent *);
#endif

    bool isTransparentForPositioner() const;
    void setTransparentForPositioner(bool trans);

    bool calcEffectiveVisible() const;
    bool setEffectiveVisibleRecur(bool);
    bool calcEffectiveEnable() const;
    void setEffectiveEnableRecur(QQuickItem *scope, bool);


    inline QSGTransformNode *itemNode();
    inline QSGNode *childContainerNode();

    /*
      QSGNode order is:
         - itemNode
         - (opacityNode)
         - (clipNode)
         - (rootNode) (shader effect source's root node)
     */

    QSGOpacityNode *opacityNode() const { return extra.isAllocated()?extra->opacityNode:0; }
    QQuickDefaultClipNode *clipNode() const { return extra.isAllocated()?extra->clipNode:0; }
    QSGRootNode *rootNode() const { return extra.isAllocated()?extra->rootNode:0; }

    QSGTransformNode *itemNodeInstance;
    QSGNode *paintNode;

    virtual QSGTransformNode *createTransformNode();

    // A reference from an effect item means that this item is used by the effect, so
    // it should insert a root node.
    void refFromEffectItem(bool hide);
    void derefFromEffectItem(bool unhide);

    void itemChange(QQuickItem::ItemChange, const QQuickItem::ItemChangeData &);

    virtual void mirrorChange() {}

    void setHasCursorInChild(bool hasCursor);

    // recursive helper to let a visual parent mark its visual children
    void markObjects(QV4::ExecutionEngine *e);
};

/*
    Key filters can be installed on a QQuickItem, but not removed.  Currently they
    are only used by attached objects (which are only destroyed on Item
    destruction), so this isn't a problem.  If in future this becomes any form
    of public API, they will have to support removal too.
*/
class QQuickItemKeyFilter
{
public:
    QQuickItemKeyFilter(QQuickItem * = 0);
    virtual ~QQuickItemKeyFilter();

    virtual void keyPressed(QKeyEvent *event, bool post);
    virtual void keyReleased(QKeyEvent *event, bool post);
#ifndef QT_NO_IM
    virtual void inputMethodEvent(QInputMethodEvent *event, bool post);
    virtual QVariant inputMethodQuery(Qt::InputMethodQuery query) const;
#endif
    virtual void componentComplete();

    bool m_processPost;

private:
    QQuickItemKeyFilter *m_next;
};

class QQuickKeyNavigationAttachedPrivate : public QObjectPrivate
{
public:
    QQuickKeyNavigationAttachedPrivate()
        : QObjectPrivate(),
          left(0), right(0), up(0), down(0), tab(0), backtab(0),
          leftSet(false), rightSet(false), upSet(false), downSet(false),
          tabSet(false), backtabSet(false) {}

    QQuickItem *left;
    QQuickItem *right;
    QQuickItem *up;
    QQuickItem *down;
    QQuickItem *tab;
    QQuickItem *backtab;
    bool leftSet : 1;
    bool rightSet : 1;
    bool upSet : 1;
    bool downSet : 1;
    bool tabSet : 1;
    bool backtabSet : 1;
};

class Q_QUICK_PRIVATE_EXPORT QQuickKeyNavigationAttached : public QObject, public QQuickItemKeyFilter
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickKeyNavigationAttached)

    Q_PROPERTY(QQuickItem *left READ left WRITE setLeft NOTIFY leftChanged)
    Q_PROPERTY(QQuickItem *right READ right WRITE setRight NOTIFY rightChanged)
    Q_PROPERTY(QQuickItem *up READ up WRITE setUp NOTIFY upChanged)
    Q_PROPERTY(QQuickItem *down READ down WRITE setDown NOTIFY downChanged)
    Q_PROPERTY(QQuickItem *tab READ tab WRITE setTab NOTIFY tabChanged)
    Q_PROPERTY(QQuickItem *backtab READ backtab WRITE setBacktab NOTIFY backtabChanged)
    Q_PROPERTY(Priority priority READ priority WRITE setPriority NOTIFY priorityChanged)

    Q_ENUMS(Priority)

public:
    QQuickKeyNavigationAttached(QObject * = 0);

    QQuickItem *left() const;
    void setLeft(QQuickItem *);
    QQuickItem *right() const;
    void setRight(QQuickItem *);
    QQuickItem *up() const;
    void setUp(QQuickItem *);
    QQuickItem *down() const;
    void setDown(QQuickItem *);
    QQuickItem *tab() const;
    void setTab(QQuickItem *);
    QQuickItem *backtab() const;
    void setBacktab(QQuickItem *);

    enum Priority { BeforeItem, AfterItem };
    Priority priority() const;
    void setPriority(Priority);

    static QQuickKeyNavigationAttached *qmlAttachedProperties(QObject *);

Q_SIGNALS:
    void leftChanged();
    void rightChanged();
    void upChanged();
    void downChanged();
    void tabChanged();
    void backtabChanged();
    void priorityChanged();

private:
    void keyPressed(QKeyEvent *event, bool post) Q_DECL_OVERRIDE;
    void keyReleased(QKeyEvent *event, bool post) Q_DECL_OVERRIDE;
    void setFocusNavigation(QQuickItem *currentItem, const char *dir,
                            Qt::FocusReason reason = Qt::OtherFocusReason);
};

class QQuickLayoutMirroringAttached : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled RESET resetEnabled NOTIFY enabledChanged)
    Q_PROPERTY(bool childrenInherit READ childrenInherit WRITE setChildrenInherit NOTIFY childrenInheritChanged)

public:
    explicit QQuickLayoutMirroringAttached(QObject *parent = 0);

    bool enabled() const;
    void setEnabled(bool);
    void resetEnabled();

    bool childrenInherit() const;
    void setChildrenInherit(bool);

    static QQuickLayoutMirroringAttached *qmlAttachedProperties(QObject *);
Q_SIGNALS:
    void enabledChanged();
    void childrenInheritChanged();
private:
    friend class QQuickItemPrivate;
    QQuickItemPrivate *itemPrivate;
};

class QQuickKeysAttachedPrivate : public QObjectPrivate
{
public:
    QQuickKeysAttachedPrivate()
        : QObjectPrivate(), inPress(false), inRelease(false)
        , inIM(false), enabled(true), imeItem(0), item(0)
    {}

    //loop detection
    bool inPress:1;
    bool inRelease:1;
    bool inIM:1;

    bool enabled : 1;

    QQuickItem *imeItem;
    QList<QQuickItem *> targets;
    QQuickItem *item;
};

class QQuickKeysAttached : public QObject, public QQuickItemKeyFilter
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(QQuickKeysAttached)

    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(QQmlListProperty<QQuickItem> forwardTo READ forwardTo)
    Q_PROPERTY(Priority priority READ priority WRITE setPriority NOTIFY priorityChanged)

    Q_ENUMS(Priority)

public:
    QQuickKeysAttached(QObject *parent=0);
    ~QQuickKeysAttached();

    bool enabled() const { Q_D(const QQuickKeysAttached); return d->enabled; }
    void setEnabled(bool enabled) {
        Q_D(QQuickKeysAttached);
        if (enabled != d->enabled) {
            d->enabled = enabled;
            Q_EMIT enabledChanged();
        }
    }

    enum Priority { BeforeItem, AfterItem};
    Priority priority() const;
    void setPriority(Priority);

    QQmlListProperty<QQuickItem> forwardTo() {
        Q_D(QQuickKeysAttached);
        return QQmlListProperty<QQuickItem>(this, d->targets);
    }

    void componentComplete() Q_DECL_OVERRIDE;

    static QQuickKeysAttached *qmlAttachedProperties(QObject *);

Q_SIGNALS:
    void enabledChanged();
    void priorityChanged();
    void pressed(QQuickKeyEvent *event);
    void released(QQuickKeyEvent *event);
    void digit0Pressed(QQuickKeyEvent *event);
    void digit1Pressed(QQuickKeyEvent *event);
    void digit2Pressed(QQuickKeyEvent *event);
    void digit3Pressed(QQuickKeyEvent *event);
    void digit4Pressed(QQuickKeyEvent *event);
    void digit5Pressed(QQuickKeyEvent *event);
    void digit6Pressed(QQuickKeyEvent *event);
    void digit7Pressed(QQuickKeyEvent *event);
    void digit8Pressed(QQuickKeyEvent *event);
    void digit9Pressed(QQuickKeyEvent *event);

    void leftPressed(QQuickKeyEvent *event);
    void rightPressed(QQuickKeyEvent *event);
    void upPressed(QQuickKeyEvent *event);
    void downPressed(QQuickKeyEvent *event);
    void tabPressed(QQuickKeyEvent *event);
    void backtabPressed(QQuickKeyEvent *event);

    void asteriskPressed(QQuickKeyEvent *event);
    void numberSignPressed(QQuickKeyEvent *event);
    void escapePressed(QQuickKeyEvent *event);
    void returnPressed(QQuickKeyEvent *event);
    void enterPressed(QQuickKeyEvent *event);
    void deletePressed(QQuickKeyEvent *event);
    void spacePressed(QQuickKeyEvent *event);
    void backPressed(QQuickKeyEvent *event);
    void cancelPressed(QQuickKeyEvent *event);
    void selectPressed(QQuickKeyEvent *event);
    void yesPressed(QQuickKeyEvent *event);
    void noPressed(QQuickKeyEvent *event);
    void context1Pressed(QQuickKeyEvent *event);
    void context2Pressed(QQuickKeyEvent *event);
    void context3Pressed(QQuickKeyEvent *event);
    void context4Pressed(QQuickKeyEvent *event);
    void callPressed(QQuickKeyEvent *event);
    void hangupPressed(QQuickKeyEvent *event);
    void flipPressed(QQuickKeyEvent *event);
    void menuPressed(QQuickKeyEvent *event);
    void volumeUpPressed(QQuickKeyEvent *event);
    void volumeDownPressed(QQuickKeyEvent *event);

private:
    void keyPressed(QKeyEvent *event, bool post) Q_DECL_OVERRIDE;
    void keyReleased(QKeyEvent *event, bool post) Q_DECL_OVERRIDE;
#ifndef QT_NO_IM
    void inputMethodEvent(QInputMethodEvent *, bool post) Q_DECL_OVERRIDE;
    QVariant inputMethodQuery(Qt::InputMethodQuery query) const Q_DECL_OVERRIDE;
#endif
    const QByteArray keyToSignal(int key);

    bool isConnected(const char *signalName);
};

Qt::MouseButtons QQuickItemPrivate::acceptedMouseButtons() const
{
    return ((extra.flag() ? Qt::LeftButton : Qt::MouseButton(0)) |
            (extra.isAllocated() ? extra->acceptedMouseButtons : Qt::MouseButtons(0)));
}

QSGContext *QQuickItemPrivate::sceneGraphContext() const
{
    Q_ASSERT(window);
    return static_cast<QQuickWindowPrivate *>(QObjectPrivate::get(window))->context->sceneGraphContext();
}

QSGRenderContext *QQuickItemPrivate::sceneGraphRenderContext() const
{
    Q_ASSERT(window);
    return static_cast<QQuickWindowPrivate *>(QObjectPrivate::get(window))->context;
}

void QQuickItemPrivate::markSortedChildrenDirty(QQuickItem *child)
{
    // If sortedChildItems == &childItems then all in childItems have z == 0
    // and we don't need to invalidate if the changed item also has z == 0.
    if (child->z() != 0. || sortedChildItems != &childItems) {
        if (sortedChildItems != &childItems)
            delete sortedChildItems;
        sortedChildItems = 0;
    }
}

QQuickItem::TransformOrigin QQuickItemPrivate::origin() const
{
    return extra.isAllocated()?extra->origin:QQuickItem::Center;
}

QSGTransformNode *QQuickItemPrivate::itemNode()
{
    if (!itemNodeInstance) {
        itemNodeInstance = createTransformNode();
        itemNodeInstance->setFlag(QSGNode::OwnedByParent, false);
#ifdef QSG_RUNTIME_DESCRIPTION
        Q_Q(QQuickItem);
        qsgnode_set_description(itemNodeInstance, QString::fromLatin1("QQuickItem(%1:%2)").arg(QString::fromLatin1(q->metaObject()->className())).arg(q->objectName()));
#endif
    }
    return itemNodeInstance;
}

QSGNode *QQuickItemPrivate::childContainerNode()
{
    if (rootNode())
        return rootNode();
    else if (clipNode())
        return clipNode();
    else if (opacityNode())
        return opacityNode();
    else
        return itemNode();
}

Q_DECLARE_OPERATORS_FOR_FLAGS(QQuickItemPrivate::ChangeTypes)

QT_END_NAMESPACE

QML_DECLARE_TYPE(QQuickItemLayer)
QML_DECLARE_TYPE(QQuickKeysAttached)
QML_DECLARE_TYPEINFO(QQuickKeysAttached, QML_HAS_ATTACHED_PROPERTIES)
QML_DECLARE_TYPE(QQuickKeyNavigationAttached)
QML_DECLARE_TYPEINFO(QQuickKeyNavigationAttached, QML_HAS_ATTACHED_PROPERTIES)
QML_DECLARE_TYPE(QQuickLayoutMirroringAttached)
QML_DECLARE_TYPEINFO(QQuickLayoutMirroringAttached, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQUICKITEM_P_H
