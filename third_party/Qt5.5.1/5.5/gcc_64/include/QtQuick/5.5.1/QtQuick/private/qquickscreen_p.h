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

#ifndef QQUICKSCREEN_P_H
#define QQUICKSCREEN_P_H

#include <qqml.h>
#include <QRect>
#include <QSize>
#include <private/qqmlglobal_p.h>

QT_BEGIN_NAMESPACE


class QQuickItem;
class QQuickWindow;
class QScreen;

class Q_AUTOTEST_EXPORT QQuickScreenAttached : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString name READ name NOTIFY nameChanged)
    Q_PROPERTY(int width READ width NOTIFY widthChanged)
    Q_PROPERTY(int height READ height NOTIFY heightChanged)
    Q_PROPERTY(int desktopAvailableWidth READ desktopAvailableWidth NOTIFY desktopGeometryChanged)
    Q_PROPERTY(int desktopAvailableHeight READ desktopAvailableHeight NOTIFY desktopGeometryChanged)
    Q_PROPERTY(qreal logicalPixelDensity READ logicalPixelDensity NOTIFY logicalPixelDensityChanged)
    Q_PROPERTY(qreal pixelDensity READ pixelDensity NOTIFY pixelDensityChanged)
    Q_PROPERTY(qreal devicePixelRatio READ devicePixelRatio NOTIFY devicePixelRatioChanged)
    // TODO Qt 6 Rename primaryOrientation to orientation
    Q_PROPERTY(Qt::ScreenOrientation primaryOrientation READ primaryOrientation NOTIFY primaryOrientationChanged)
    // TODO Qt 6 Remove this orientation -> incomplete device orientation -> better use OrientationSensor
    Q_PROPERTY(Qt::ScreenOrientation orientation READ orientation NOTIFY orientationChanged)
    Q_PROPERTY(Qt::ScreenOrientations orientationUpdateMask READ orientationUpdateMask
               WRITE setOrientationUpdateMask NOTIFY orientationUpdateMaskChanged)

public:
    QQuickScreenAttached(QObject* attachee);

    QString name() const;
    int width() const;
    int height() const;
    int desktopAvailableWidth() const;
    int desktopAvailableHeight() const;
    qreal logicalPixelDensity() const;
    qreal pixelDensity() const;
    qreal devicePixelRatio() const;
    Qt::ScreenOrientation primaryOrientation() const;
    Qt::ScreenOrientation orientation() const;
    Qt::ScreenOrientations orientationUpdateMask() const;
    void setOrientationUpdateMask(Qt::ScreenOrientations mask);

    //Treats int as Qt::ScreenOrientation, due to QTBUG-20639
    Q_INVOKABLE int angleBetween(int a, int b);

    void windowChanged(QQuickWindow*);

Q_SIGNALS:
    void nameChanged();
    void widthChanged();
    void heightChanged();
    void desktopGeometryChanged();
    void logicalPixelDensityChanged();
    void pixelDensityChanged();
    void devicePixelRatioChanged();
    void primaryOrientationChanged();
    void orientationChanged();
    void orientationUpdateMaskChanged();

protected Q_SLOTS:
    void screenChanged(QScreen*);

private:
    QPointer<QScreen> m_screen;
    QQuickWindow* m_window;
    QQuickItem* m_attachee;
    Qt::ScreenOrientations m_updateMask;
    bool m_updateMaskSet;
};

class Q_AUTOTEST_EXPORT QQuickScreen : public QObject
{
    Q_OBJECT
public:
    static QQuickScreenAttached *qmlAttachedProperties(QObject *object){ return new QQuickScreenAttached(object); }
};

QT_END_NAMESPACE

QML_DECLARE_TYPEINFO(QQuickScreen, QML_HAS_ATTACHED_PROPERTIES)

#endif
