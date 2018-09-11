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

#ifndef TIMELINEAXIS_H
#define TIMELINEAXIS_H

#include <QGraphicsItemGroup>
#include <QGraphicsTextItem>

class TimeLineAxis : public QGraphicsItemGroup {
 public:
  enum { Type = QGraphicsItemGroup::UserType + QGraphicsItemGroup::Type + 2 };

  explicit TimeLineAxis(QGraphicsItem* parent = nullptr);
  ~TimeLineAxis();

  int type() const { return Type; }

  QString headerText(void) const { return _headerText.toolTip(); }
  void setHeaderText(const QString& rowName);

  void redraw(int headerWidth, int timeLineLength, int height, int scale,
              int sparsity) {
    destroy();
    draw(headerWidth, timeLineLength, height, scale, sparsity);
  }

  void destroy(void);
  void draw(int headerWidth, int timeLineLength, int height, int scale,
            int sparsity);

  void adjustHeaderTextPos(int headerWidth, int height) {
    QRectF rect = _headerText.boundingRect();
    _headerText.setPos((headerWidth - rect.width()) / 2.0f,
                       (height - rect.height()) / 2.0f);
  }

 private:
  QGraphicsTextItem _headerText;
};

#endif  // TIMELINEAXIS_H
