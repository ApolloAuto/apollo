/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QAUDIORECORDER_H
#define QAUDIORECORDER_H

#include <QtMultimedia/qmediarecorder.h>
#include <QtMultimedia/qmediaobject.h>
#include <QtMultimedia/qmediaencodersettings.h>
#include <QtMultimedia/qmediaenumdebug.h>

#include <QtCore/qpair.h>

QT_BEGIN_NAMESPACE

class QString;
class QSize;
class QAudioFormat;
QT_END_NAMESPACE

QT_BEGIN_NAMESPACE

class QAudioRecorderPrivate;
class Q_MULTIMEDIA_EXPORT QAudioRecorder : public QMediaRecorder
{
    Q_OBJECT
    Q_PROPERTY(QString audioInput READ audioInput WRITE setAudioInput NOTIFY audioInputChanged)
public:
    QAudioRecorder(QObject *parent = 0);
    ~QAudioRecorder();

    QStringList audioInputs() const;
    QString defaultAudioInput() const;
    QString audioInputDescription(const QString& name) const;

    QString audioInput() const;

public Q_SLOTS:
    void setAudioInput(const QString& name);

Q_SIGNALS:
    void audioInputChanged(const QString& name);
    void availableAudioInputsChanged();

private:
    Q_DISABLE_COPY(QAudioRecorder)
    Q_DECLARE_PRIVATE(QAudioRecorder)
};

QT_END_NAMESPACE

#endif  // QAUDIORECORDER_H
