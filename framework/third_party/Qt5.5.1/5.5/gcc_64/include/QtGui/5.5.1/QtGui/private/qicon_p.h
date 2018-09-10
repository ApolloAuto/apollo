/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
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

#ifndef QICON_P_H
#define QICON_P_H

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

#include <QtCore/qglobal.h>
#include <QtCore/qsize.h>
#include <QtCore/qlist.h>
#include <QtGui/qpixmap.h>
#include <QtGui/qicon.h>
#include <QtGui/qiconengine.h>

#ifndef QT_NO_ICON
QT_BEGIN_NAMESPACE

class QIconPrivate
{
public:
    QIconPrivate();

    ~QIconPrivate() {
        delete engine;
    }

    qreal pixmapDevicePixelRatio(qreal displayDevicePixelRatio, const QSize &requestedSize, const QSize &actualSize);

    QIconEngine *engine;

    QAtomicInt ref;
    int serialNum;
    int detach_no;
};


struct QPixmapIconEngineEntry
{
    QPixmapIconEngineEntry():mode(QIcon::Normal), state(QIcon::Off){}
    QPixmapIconEngineEntry(const QPixmap &pm, QIcon::Mode m = QIcon::Normal, QIcon::State s = QIcon::Off)
        :pixmap(pm), size(pm.size()), mode(m), state(s){}
    QPixmapIconEngineEntry(const QString &file, const QSize &sz = QSize(), QIcon::Mode m = QIcon::Normal, QIcon::State s = QIcon::Off)
        :fileName(file), size(sz), mode(m), state(s){}
    QPixmapIconEngineEntry(const QString &file, const QImage &image, QIcon::Mode m = QIcon::Normal, QIcon::State s = QIcon::Off);
    QPixmap pixmap;
    QString fileName;
    QSize size;
    QIcon::Mode mode;
    QIcon::State state;
    bool isNull() const {return (fileName.isEmpty() && pixmap.isNull()); }
};

inline QPixmapIconEngineEntry::QPixmapIconEngineEntry(const QString &file, const QImage &image, QIcon::Mode m, QIcon::State s)
    : fileName(file), size(image.size()), mode(m), state(s)
{
    pixmap.convertFromImage(image);
    // Reset the devicePixelRatio. The pixmap may be loaded from a @2x file,
    // but be used as a 1x pixmap by QIcon.
    pixmap.setDevicePixelRatio(1.0);
}

class Q_GUI_EXPORT QPixmapIconEngine : public QIconEngine {
public:
    QPixmapIconEngine();
    QPixmapIconEngine(const QPixmapIconEngine &);
    ~QPixmapIconEngine();
    void paint(QPainter *painter, const QRect &rect, QIcon::Mode mode, QIcon::State state) Q_DECL_OVERRIDE;
    QPixmap pixmap(const QSize &size, QIcon::Mode mode, QIcon::State state) Q_DECL_OVERRIDE;
    QPixmapIconEngineEntry *bestMatch(const QSize &size, QIcon::Mode mode, QIcon::State state, bool sizeOnly);
    QSize actualSize(const QSize &size, QIcon::Mode mode, QIcon::State state) Q_DECL_OVERRIDE;
    void addPixmap(const QPixmap &pixmap, QIcon::Mode mode, QIcon::State state) Q_DECL_OVERRIDE;
    void addFile(const QString &fileName, const QSize &size, QIcon::Mode mode, QIcon::State state) Q_DECL_OVERRIDE;

    QString key() const Q_DECL_OVERRIDE;
    QIconEngine *clone() const Q_DECL_OVERRIDE;
    bool read(QDataStream &in) Q_DECL_OVERRIDE;
    bool write(QDataStream &out) const Q_DECL_OVERRIDE;
    void virtual_hook(int id, void *data) Q_DECL_OVERRIDE;

private:
    QPixmapIconEngineEntry *tryMatch(const QSize &size, QIcon::Mode mode, QIcon::State state);
    QVector<QPixmapIconEngineEntry> pixmaps;

    friend Q_GUI_EXPORT QDataStream &operator<<(QDataStream &s, const QIcon &icon);
    friend class QIconThemeEngine;
};

QT_END_NAMESPACE
#endif //QT_NO_ICON
#endif // QICON_P_H
