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

#include "time_block_item.h"
#include <QGraphicsSceneMouseEvent>
#include <QMessageBox>
#include <QObject>
#include <QString>

TimeBlockItem::TimeBlockItem(const int64_t& startTimeStamp,
                             const int64_t& endTimeStamp, QGraphicsItem* parent)
    : QGraphicsRectItem(parent),
      _eventId(0),
      _startTimeStamp(startTimeStamp),
      _endTimeStamp(endTimeStamp) {}
