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

#ifndef STDOUT_CONSUMER_H
#define STDOUT_CONSUMER_H

#include <fastrtps/log/Log.h>

namespace eprosima {
namespace fastrtps {

class StdoutConsumer: public LogConsumer {
public:
   RTPS_DllAPI virtual void Consume(const Log::Entry&);

private:
   void PrintHeader(const Log::Entry&) const;
   void PrintContext(const Log::Entry&) const;
};

} // namespace fastrtps
} // namespace eprosima

#endif
