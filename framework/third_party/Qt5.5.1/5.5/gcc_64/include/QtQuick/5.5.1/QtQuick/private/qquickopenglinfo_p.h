/****************************************************************************
**
** Copyright (C) 2014 BlackBerry Ltd.
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

#ifndef QQUICKOPENGLINFO_P_H
#define QQUICKOPENGLINFO_P_H

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

#include <QtCore/qobject.h>
#include <QtCore/qpointer.h>
#include <QtGui/qsurfaceformat.h>
#include <QtQml/qqml.h>

QT_BEGIN_NAMESPACE

class QQuickItem;
class QQuickWindow;

class QQuickOpenGLInfo : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int majorVersion READ majorVersion NOTIFY majorVersionChanged FINAL)
    Q_PROPERTY(int minorVersion READ minorVersion NOTIFY minorVersionChanged FINAL)
    Q_PROPERTY(ContextProfile profile READ profile NOTIFY profileChanged FINAL)
    Q_PROPERTY(RenderableType renderableType READ renderableType NOTIFY renderableTypeChanged FINAL)
    Q_ENUMS(ContextProfile RenderableType)

public:
    QQuickOpenGLInfo(QQuickItem *item = 0);

    int majorVersion() const;
    int minorVersion() const;

    // keep in sync with QSurfaceFormat::OpenGLContextProfile
    enum ContextProfile {
        NoProfile = QSurfaceFormat::NoProfile,
        CoreProfile = QSurfaceFormat::CoreProfile,
        CompatibilityProfile = QSurfaceFormat::CompatibilityProfile
    };
    ContextProfile profile() const;

    // keep in sync with QSurfaceFormat::RenderableType
    enum RenderableType {
        Unspecified = QSurfaceFormat::DefaultRenderableType,
        OpenGL = QSurfaceFormat::OpenGL,
        OpenGLES = QSurfaceFormat::OpenGLES
    };
    RenderableType renderableType() const;

    static QQuickOpenGLInfo *qmlAttachedProperties(QObject *object);

Q_SIGNALS:
    void majorVersionChanged();
    void minorVersionChanged();
    void profileChanged();
    void renderableTypeChanged();

private Q_SLOTS:
    void updateFormat();
    void setWindow(QQuickWindow *window);

private:
    QPointer<QQuickWindow> m_window;
    int m_majorVersion;
    int m_minorVersion;
    ContextProfile m_profile;
    RenderableType m_renderableType;
};

QT_END_NAMESPACE

QML_DECLARE_TYPEINFO(QQuickOpenGLInfo, QML_HAS_ATTACHED_PROPERTIES)

#endif // QQUICKOPENGLINFO_P_H
