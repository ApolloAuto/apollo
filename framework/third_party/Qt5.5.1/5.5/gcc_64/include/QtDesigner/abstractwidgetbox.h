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

#ifndef ABSTRACTWIDGETBOX_H
#define ABSTRACTWIDGETBOX_H

#include <QtDesigner/sdk_global.h>

#include <QtCore/QSharedDataPointer>
#include <QtCore/QMetaType>
#include <QtWidgets/QWidget>
#include <QtGui/QIcon>

QT_BEGIN_NAMESPACE

class DomUI;
class QDesignerDnDItemInterface;

class QDesignerWidgetBoxWidgetData;

class QDESIGNER_SDK_EXPORT QDesignerWidgetBoxInterface : public QWidget
{
    Q_OBJECT
public:
    class QDESIGNER_SDK_EXPORT Widget {
    public:
        enum Type { Default, Custom };
        Widget(const QString &aname = QString(), const QString &xml = QString(),
                const QString &icon_name = QString(), Type atype = Default);
        ~Widget();
        Widget(const Widget &w);
        Widget &operator=(const Widget &w);

        QString name() const;
        void setName(const QString &aname);
        QString domXml() const;
        void setDomXml(const QString &xml);
        QString iconName() const;
        void setIconName(const QString &icon_name);
        Type type() const;
        void setType(Type atype);

        bool isNull() const;

    private:
        QSharedDataPointer<QDesignerWidgetBoxWidgetData> m_data;
    };

    typedef QList<Widget> WidgetList;

    class Category {
    public:
        enum Type { Default, Scratchpad };

        Category(const QString &aname = QString(), Type atype = Default)
            : m_name(aname), m_type(atype) {}

        QString name() const { return m_name; }
        void setName(const QString &aname) { m_name = aname; }
        int widgetCount() const { return m_widget_list.size(); }
        Widget widget(int idx) const { return m_widget_list.at(idx); }
        void removeWidget(int idx) { m_widget_list.removeAt(idx); }
        void addWidget(const Widget &awidget) { m_widget_list.append(awidget); }
        Type type() const { return m_type; }
        void setType(Type atype) { m_type = atype; }

        bool isNull() const { return m_name.isEmpty(); }

    private:
        QString m_name;
        Type m_type;
        QList<Widget> m_widget_list;
    };
    typedef QList<Category> CategoryList;

    QDesignerWidgetBoxInterface(QWidget *parent = 0, Qt::WindowFlags flags = 0);
    virtual ~QDesignerWidgetBoxInterface();

    virtual int categoryCount() const = 0;
    virtual Category category(int cat_idx) const = 0;
    virtual void addCategory(const Category &cat) = 0;
    virtual void removeCategory(int cat_idx) = 0;

    virtual int widgetCount(int cat_idx) const = 0;
    virtual Widget widget(int cat_idx, int wgt_idx) const = 0;
    virtual void addWidget(int cat_idx, const Widget &wgt) = 0;
    virtual void removeWidget(int cat_idx, int wgt_idx) = 0;

    int findOrInsertCategory(const QString &categoryName);

    virtual void dropWidgets(const QList<QDesignerDnDItemInterface*> &item_list,
                                const QPoint &global_mouse_pos) = 0;

    virtual void setFileName(const QString &file_name) = 0;
    virtual QString fileName() const = 0;
    virtual bool load() = 0;
    virtual bool save() = 0;
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QT_PREPEND_NAMESPACE(QDesignerWidgetBoxInterface::Widget))

#endif // ABSTRACTWIDGETBOX_H
