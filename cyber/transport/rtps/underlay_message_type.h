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

#ifndef CYBER_TRANSPORT_RTPS_UNDERLAY_MESSAGE_TYPE_H_
#define CYBER_TRANSPORT_RTPS_UNDERLAY_MESSAGE_TYPE_H_

#include "cyber/base/macros.h"

#include "fastdds/dds/topic/TopicDataType.hpp"
#include "fastrtps/utils/md5.h"
#include "cyber/transport/rtps/underlay_message.h"

namespace apollo {
namespace cyber {
namespace transport {

/*!
 * @brief This class represents the TopicDataType of the type UnderlayMessage
 * defined by the user in the IDL file.
 * @ingroup UNDERLAYMESSAGE
 */
class UnderlayMessageType : public eprosima::fastdds::dds::TopicDataType {
 public:
  using type = UnderlayMessage;

  UnderlayMessageType();
  virtual ~UnderlayMessageType();
  virtual bool serialize(  // NOLINT
      void* data,
      eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;
  virtual bool deserialize(  // NOLINT
      eprosima::fastrtps::rtps::SerializedPayload_t* payload,
      void* data) override;
  virtual std::function<uint32_t()> getSerializedSizeProvider(  // NOLINT
      void* data) override;
  virtual bool getKey(void* data,  // NOLINT
                      eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
                      bool force_md5 = false) override;
  virtual void* createData() override;           // NOLINT
  virtual void deleteData(void* data) override;  // NOLINT
  MD5 m_md5;
  unsigned char* m_keyBuffer;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_RTPS_UNDERLAY_MESSAGE_TYPE_H_
