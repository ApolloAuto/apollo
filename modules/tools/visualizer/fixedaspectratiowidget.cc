/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/tools/visualizer/fixedaspectratiowidget.h"

#include <QtGui/QContextMenuEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QMenu>
#include <QtWidgets/QStyleOption>

FixedAspectRatioWidget::FixedAspectRatioWidget(QWidget* parent, int index)
    : QWidget(parent), index_(index), refresh_timer_(this), viewer_() {
  viewer_.setParent(this);
  viewer_.setGeometry(geometry());

  refresh_timer_.setObjectName(tr("_refreshTimer"));
  refresh_timer_.setInterval(40);
  connect(&refresh_timer_, SIGNAL(timeout()), &viewer_, SLOT(repaint()));
  setAutoFillBackground(true);
}

void FixedAspectRatioWidget::StartOrStopUpdate(bool b) {
  if (viewer_.is_init_) {
    if (b) {
      refresh_timer_.start();
    } else {
      refresh_timer_.stop();
    }
  }
}

void FixedAspectRatioWidget::SetupDynamicTexture(
    const std::shared_ptr<Texture>& textureObj) {
  if (textureObj == nullptr) {
    viewer_.default_image_->setSizeChanged();
    viewer_.plane_.set_texture(viewer_.default_image_);
  } else {
    viewer_.plane_.set_texture(textureObj);
  }
}

void FixedAspectRatioWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    emit focusOnThis(this);
    QWidget::mouseDoubleClickEvent(event);
  }
}

void FixedAspectRatioWidget::contextMenuEvent(QContextMenuEvent* event) {
  emit focusOnThis(this);
  QMenu m;
  m.addActions(actions());
  m.exec(event->globalPos());
  m.clear();

  QWidget::contextMenuEvent(event);
}

void FixedAspectRatioWidget::resizeEvent(QResizeEvent* revent) {
  QSize size = revent->size();

  {
    double aspect = static_cast<double>(viewer_.plane_.texWidth()) /
                    static_cast<double>(viewer_.plane_.texHeight());

    int wc = 4;
    int hc = 9;
    if (aspect == 4.0 / 3.0) {
      wc = 2;
      hc = 3;
    } else if (aspect == 16.0 / 10.0) {
      wc = 4;
      hc = 10;
    }

    int w = size.width();
    int h = size.height();

    int tmpH = w >> wc;
    w = tmpH << wc;
    size.setWidth(w);
    tmpH *= hc;
    if (tmpH <= h) {
      size.setHeight(tmpH);
    } else {
      tmpH = h / hc;
      h = tmpH * hc;
      size.setHeight(h);
      size.setWidth(tmpH << wc);
    }
  }

  viewer_.setGeometry((revent->size().width() - size.width()) / 2,
                      (revent->size().height() - size.height()) / 2,
                      size.width(), size.height());
  QWidget::resizeEvent(revent);
}

void FixedAspectRatioWidget::paintEvent(QPaintEvent* event) {
  QStyleOption opt;
  opt.init(this);
  QPainter p(this);
  style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);

  QWidget::paintEvent(event);
}
