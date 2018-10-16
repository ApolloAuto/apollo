// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file rtps_all.h
 *
 */

#ifndef FASTRTPS_ALL_H_
#define FASTRTPS_ALL_H_

//USER THIS HEADER TO CREATE RAPID PROTOTYPES AND TESTS
//DO NOT INCLUDE IN PROJETCTS WERE COMPILATION TIME OR SIZE IS REVELANT
//SINCE IT INCLUDES ALL NECESSARY HEADERS.

#include "rtps/common/all_common.h"

#include "Domain.h"

#include "participant/Participant.h"
#include "participant/ParticipantListener.h"
#include "publisher/Publisher.h"
#include "subscriber/Subscriber.h"
#include "publisher/PublisherListener.h"
#include "subscriber/SubscriberListener.h"


#include "attributes/ParticipantAttributes.h"
#include "attributes/PublisherAttributes.h"
#include "attributes/SubscriberAttributes.h"

#include "subscriber/SampleInfo.h"
#include "TopicDataType.h"

#include "utils/IPFinder.h"
#include "log/Log.h"
#include "utils/eClock.h"
#include "utils/TimeConversion.h"

#include "qos/ParameterList.h"
#include "qos/QosPolicies.h"

#include "log/Log.h"


#endif /* FASTRTPS_ALL_H_ */
