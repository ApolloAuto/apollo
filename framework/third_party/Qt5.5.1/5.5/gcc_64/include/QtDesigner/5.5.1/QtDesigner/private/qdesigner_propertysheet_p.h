/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Designer of the Qt Toolkit.
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

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of Qt Designer.  This header
// file may change from version to version without notice, or even be removed.
//
// We mean it.
//

#ifndef QDESIGNER_PROPERTYSHEET_H
#define QDESIGNER_PROPERTYSHEET_H

#include "shared_global_p.h"
#include "dynamicpropertysheet.h"
#include <QtDesigner/propertysheet.h>
#include <QtDesigner/default_extensionfactory.h>
#include <QtDesigner/QExtensionManager>

#include <QtCore/QVariant>
#include <QtCore/QPair>

#include <QtCore/QPointer>

QT_BEGIN_NAMESPACE

class QLayout;
class QDesignerFormEditorInterface;
class QDesignerPropertySheetPrivate;

namespace qdesigner_internal
{
    class DesignerPixmapCache;
    class DesignerIconCache;
    class FormWindowBase;
}

class QDESIGNER_SHARED_EXPORT QDesignerPropertySheet: public QObject, public QDesignerPropertySheetExtension, public QDesignerDynamicPropertySheetExtension
{
    Q_OBJECT
    Q_INTERFACES(QDesignerPropertySheetExtension QDesignerDynamicPropertySheetExtension)
public:
    explicit QDesignerPropertySheet(QObject *object, QObject *parent = 0);
    virtual ~QDesignerPropertySheet();

    int indexOf(const QString &name) const Q_DECL_OVERRIDE;

    int count() const Q_DECL_OVERRIDE;
    QString propertyName(int index) const Q_DECL_OVERRIDE;

    QString propertyGroup(int index) const Q_DECL_OVERRIDE;
    void setPropertyGroup(int index, const QString &group) Q_DECL_OVERRIDE;

    bool hasReset(int index) const Q_DECL_OVERRIDE;
    bool reset(int index) Q_DECL_OVERRIDE;

    bool isAttribute(int index) const Q_DECL_OVERRIDE;
    void setAttribute(int index, bool b) Q_DECL_OVERRIDE;

    bool isVisible(int index) const Q_DECL_OVERRIDE;
    void setVisible(int index, bool b) Q_DECL_OVERRIDE;

    QVariant property(int index) const Q_DECL_OVERRIDE;
    void setProperty(int index, const QVariant &value) Q_DECL_OVERRIDE;

    bool isChanged(int index) const Q_DECL_OVERRIDE;

    void setChanged(int index, bool changed) Q_DECL_OVERRIDE;

    bool dynamicPropertiesAllowed() const Q_DECL_OVERRIDE;
    int addDynamicProperty(const QString &propertyName, const QVariant &value) Q_DECL_OVERRIDE;
    bool removeDynamicProperty(int index) Q_DECL_OVERRIDE;
    bool isDynamicProperty(int index) const Q_DECL_OVERRIDE;
    bool canAddDynamicProperty(const QString &propertyName) const Q_DECL_OVERRIDE;

    bool isDefaultDynamicProperty(int index) const;

    bool isResourceProperty(int index) const;
    QVariant defaultResourceProperty(int index) const;

    qdesigner_internal::DesignerPixmapCache *pixmapCache() const;
    void setPixmapCache(qdesigner_internal::DesignerPixmapCache *cache);
    qdesigner_internal::DesignerIconCache *iconCache() const;
    void setIconCache(qdesigner_internal::DesignerIconCache *cache);
    int createFakeProperty(const QString &propertyName, const QVariant &value = QVariant());

    bool isEnabled(int index) const Q_DECL_OVERRIDE;
    QObject *object() const;

    static bool internalDynamicPropertiesEnabled();
    static void setInternalDynamicPropertiesEnabled(bool v);

protected:
    bool isAdditionalProperty(int index) const;
    bool isFakeProperty(int index) const;
    QVariant resolvePropertyValue(int index, const QVariant &value) const;
    QVariant metaProperty(int index) const;
    void setFakeProperty(int index, const QVariant &value);
    void clearFakeProperties();

    bool isFakeLayoutProperty(int index) const;
    bool isDynamic(int index) const;
    qdesigner_internal::FormWindowBase *formWindowBase() const;
    QDesignerFormEditorInterface *core() const;

public:
    enum PropertyType { PropertyNone,
                        PropertyLayoutObjectName,
                        PropertyLayoutLeftMargin,
                        PropertyLayoutTopMargin,
                        PropertyLayoutRightMargin,
                        PropertyLayoutBottomMargin,
                        PropertyLayoutSpacing,
                        PropertyLayoutHorizontalSpacing,
                        PropertyLayoutVerticalSpacing,
                        PropertyLayoutSizeConstraint,
                        PropertyLayoutFieldGrowthPolicy,
                        PropertyLayoutRowWrapPolicy,
                        PropertyLayoutLabelAlignment,
                        PropertyLayoutFormAlignment,
                        PropertyLayoutBoxStretch,
                        PropertyLayoutGridRowStretch,
                        PropertyLayoutGridColumnStretch,
                        PropertyLayoutGridRowMinimumHeight,
                        PropertyLayoutGridColumnMinimumWidth,
                        PropertyBuddy,
                        PropertyAccessibility,
                        PropertyGeometry,
                        PropertyCheckable,
                        PropertyWindowTitle,
                        PropertyWindowIcon,
                        PropertyWindowFilePath,
                        PropertyWindowOpacity,
                        PropertyWindowIconText,
                        PropertyWindowModality,
                        PropertyWindowModified,
                        PropertyStyleSheet,
                        PropertyText
    };

    enum ObjectType { ObjectNone, ObjectLabel, ObjectLayout, ObjectLayoutWidget };

    static ObjectType objectTypeFromObject(const QObject *o);
    static PropertyType propertyTypeFromName(const QString &name);

protected:
    PropertyType propertyType(int index) const;
    ObjectType objectType() const;

private:
    QDesignerPropertySheetPrivate *d;
};

/* Abstract base class for factories that register a property sheet that implements
 * both QDesignerPropertySheetExtension and QDesignerDynamicPropertySheetExtension
 * by multiple inheritance. The factory maintains ownership of
 * the extension and returns it for both id's. */

class QDESIGNER_SHARED_EXPORT QDesignerAbstractPropertySheetFactory: public QExtensionFactory
{
    Q_OBJECT
    Q_INTERFACES(QAbstractExtensionFactory)
public:
    explicit QDesignerAbstractPropertySheetFactory(QExtensionManager *parent = 0);
    virtual ~QDesignerAbstractPropertySheetFactory();

    QObject *extension(QObject *object, const QString &iid) const;

private slots:
    void objectDestroyed(QObject *object);

private:
    virtual QObject *createPropertySheet(QObject *qObject, QObject *parent) const = 0;

    struct PropertySheetFactoryPrivate;
    PropertySheetFactoryPrivate *m_impl;
};

/* Convenience factory template for property sheets that implement
 * QDesignerPropertySheetExtension and QDesignerDynamicPropertySheetExtension
 * by multiple inheritance. */

template <class Object, class PropertySheet>
class QDesignerPropertySheetFactory : public QDesignerAbstractPropertySheetFactory {
public:
    explicit QDesignerPropertySheetFactory(QExtensionManager *parent = 0);

    static void registerExtension(QExtensionManager *mgr);

private:
    // Does a  qobject_cast on  the object.
    QObject *createPropertySheet(QObject *qObject, QObject *parent) const Q_DECL_OVERRIDE;
};

template <class Object, class PropertySheet>
QDesignerPropertySheetFactory<Object, PropertySheet>::QDesignerPropertySheetFactory(QExtensionManager *parent) :
    QDesignerAbstractPropertySheetFactory(parent)
{
}

template <class Object, class PropertySheet>
QObject *QDesignerPropertySheetFactory<Object, PropertySheet>::createPropertySheet(QObject *qObject, QObject *parent) const
{
    Object *object = qobject_cast<Object *>(qObject);
    if (!object)
        return 0;
    return new PropertySheet(object, parent);
}

template <class Object, class PropertySheet>
void QDesignerPropertySheetFactory<Object, PropertySheet>::registerExtension(QExtensionManager *mgr)
{
    QDesignerPropertySheetFactory *factory = new QDesignerPropertySheetFactory(mgr);
    mgr->registerExtensions(factory, Q_TYPEID(QDesignerPropertySheetExtension));
    mgr->registerExtensions(factory, Q_TYPEID(QDesignerDynamicPropertySheetExtension));
}


// Standard property sheet
typedef QDesignerPropertySheetFactory<QObject, QDesignerPropertySheet> QDesignerDefaultPropertySheetFactory;

QT_END_NAMESPACE

#endif // QDESIGNER_PROPERTYSHEET_H
