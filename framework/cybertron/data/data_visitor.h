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

#ifndef CYBERTRON_DATA_DATA_VISITOR_H_
#define CYBERTRON_DATA_DATA_VISITOR_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/data/channel_buffer.h"
#include "cybertron/data/data_dispatcher.h"
#include "cybertron/data/data_visitor_base.h"
#include "cybertron/data/fusion/all_latest.h"
#include "cybertron/data/fusion/data_fusion.h"
#include "cybertron/node/reader_base.h"

namespace apollo {
namespace cybertron {
namespace data {

template <typename T>
using BufferType = CacheBuffer<std::shared_ptr<T>>;

template <typename M0, typename M1 = NullType, typename M2 = NullType,
          typename M3 = NullType>
class DataVisitor : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::vector<std::shared_ptr<ReaderBase>>& readers)
      : buffer_m0_(readers[0]->ChannelId(),
                   new BufferType<M0>(readers[0]->QosProfile().depth())),
        buffer_m1_(readers[1]->ChannelId(),
                   new BufferType<M1>(readers[1]->QosProfile().depth())),
        buffer_m2_(readers[2]->ChannelId(),
                   new BufferType<M2>(readers[2]->QosProfile().depth())),
        buffer_m3_(readers[3]->ChannelId(),
                   new BufferType<M3>(readers[3]->QosProfile().depth())) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_m0_);
    DataDispatcher<M1>::Instance()->AddBuffer(buffer_m1_);
    DataDispatcher<M2>::Instance()->AddBuffer(buffer_m2_);
    DataDispatcher<M3>::Instance()->AddBuffer(buffer_m3_);
    data_notifier_->AddNotifier(buffer_m0_.ChannelId(), notifier_);
    data_fusion_ = new fusion::AllLatest<M0, M1, M2, M3>(
        buffer_m0_, buffer_m1_, buffer_m2_, buffer_m3_);
  }

  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,    // NOLINT
                std::shared_ptr<M2>& m2, std::shared_ptr<M3>& m3) {  // NOLINT
    if (data_fusion_->Fusion(&next_msg_index_, m0, m1, m2, m3)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  fusion::DataFusion<M0, M1, M2, M3>* data_fusion_;
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
  ChannelBuffer<M3> buffer_m3_;
};

template <typename M0, typename M1, typename M2>
class DataVisitor<M0, M1, M2, NullType> : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::vector<std::shared_ptr<ReaderBase>>& readers)
      : buffer_m0_(readers[0]->ChannelId(),
                   new BufferType<M0>(readers[0]->QosProfile().depth())),
        buffer_m1_(readers[1]->ChannelId(),
                   new BufferType<M1>(readers[1]->QosProfile().depth())),
        buffer_m2_(readers[2]->ChannelId(),
                   new BufferType<M2>(readers[2]->QosProfile().depth())) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_m0_);
    DataDispatcher<M1>::Instance()->AddBuffer(buffer_m1_);
    DataDispatcher<M2>::Instance()->AddBuffer(buffer_m2_);
    data_notifier_->AddNotifier(buffer_m0_.ChannelId(), notifier_);
    data_fusion_ =
        new fusion::AllLatest<M0, M1, M2>(buffer_m0_, buffer_m1_, buffer_m2_);
  }

  ~DataVisitor() {
    if (data_fusion_) {
      delete data_fusion_;
    }
  }

  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1,  // NOLINT
                std::shared_ptr<M2>& m2) {                         // NOLINT
    if (data_fusion_->Fusion(&next_msg_index_, m0, m1, m2)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  fusion::DataFusion<M0, M1, M2>* data_fusion_;
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
  ChannelBuffer<M2> buffer_m2_;
};

template <typename M0, typename M1>
class DataVisitor<M0, M1, NullType, NullType> : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::vector<std::shared_ptr<ReaderBase>>& readers)
      : buffer_m0_(readers[0]->ChannelId(),
                   new BufferType<M0>(readers[0]->QosProfile().depth())),
        buffer_m1_(readers[1]->ChannelId(),
                   new BufferType<M1>(readers[1]->QosProfile().depth())) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_m0_);
    DataDispatcher<M1>::Instance()->AddBuffer(buffer_m1_);
    data_notifier_->AddNotifier(buffer_m0_.ChannelId(), notifier_);
    data_fusion_ = new fusion::AllLatest<M0, M1>(buffer_m0_, buffer_m1_);
  }

  ~DataVisitor() {
    if (data_fusion_) {
      delete data_fusion_;
    }
  }

  bool TryFetch(std::shared_ptr<M0>& m0, std::shared_ptr<M1>& m1) {  // NOLINT
    if (data_fusion_->Fusion(&next_msg_index_, m0, m1)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  fusion::DataFusion<M0, M1>* data_fusion_;
  ChannelBuffer<M0> buffer_m0_;
  ChannelBuffer<M1> buffer_m1_;
};

template <typename M0>
class DataVisitor<M0, NullType, NullType, NullType> : public DataVisitorBase {
 public:
  explicit DataVisitor(const std::shared_ptr<ReaderBase>& readers)
      : buffer_(readers->ChannelId(),
                new BufferType<M0>(readers->QosProfile().depth())) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_);
    data_notifier_->AddNotifier(buffer_.ChannelId(), notifier_);
  }

  DataVisitor(uint64_t channel_id, uint32_t queue_size)
      : buffer_(channel_id, new BufferType<M0>(queue_size)) {
    DataDispatcher<M0>::Instance()->AddBuffer(buffer_);
    data_notifier_->AddNotifier(buffer_.ChannelId(), notifier_);
  }

  bool TryFetch(std::shared_ptr<M0>& m0) {  // NOLINT
    if (buffer_.Fetch(&next_msg_index_, m0)) {
      next_msg_index_++;
      return true;
    }
    return false;
  }

 private:
  ChannelBuffer<M0> buffer_;
};

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_VISITOR_H_
