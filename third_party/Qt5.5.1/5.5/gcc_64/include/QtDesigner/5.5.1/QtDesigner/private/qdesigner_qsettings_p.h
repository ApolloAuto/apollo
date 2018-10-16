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

#ifndef QDESIGNER_QSETTINGS_H
#define QDESIGNER_QSETTINGS_H

#include "shared_global_p.h"

#include <QtDesigner/QDesignerSettingsInterface>

#include <QtCore/QSettings>

QT_BEGIN_NAMESPACE

//  Implements QDesignerSettingsInterface by calls to QSettings
class QDESIGNER_SHARED_EXPORT QDesignerQSettings : public QDesignerSettingsInterface
{
public:
    QDesignerQSettings();

    void beginGroup(const QString &prefix) Q_DECL_OVERRIDE;
    virtual void endGroup();

    bool contains(const QString &key) const Q_DECL_OVERRIDE;
    void setValue(const QString &key, const QVariant &value) Q_DECL_OVERRIDE;
    QVariant value(const QString &key, const QVariant &defaultValue = QVariant()) const Q_DECL_OVERRIDE;
    void remove(const QString &key) Q_DECL_OVERRIDE;

    // The application name to be used for settings. Allows for including
    // the Qt version to prevent settings of different Qt versions from
    // interfering.
    static QString settingsApplicationName();

private:
    QSettings m_settings;
};

QT_END_NAMESPACE

#endif // QDESIGNER_QSETTINGS_H
