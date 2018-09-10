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
 * @file RTPSDomain.h
 */



#ifndef RTPSRTPSParticipant_H_
#define RTPSRTPSParticipant_H_

#include "common/Types.h"

#include "attributes/RTPSParticipantAttributes.h"

#include <set>


namespace eprosima{
namespace fastrtps{
namespace rtps{

class RTPSParticipantImpl;
class RTPSParticipant;
class RTPSParticipantListener;
class RTPSWriter;
class WriterAttributes;
class WriterHistory;
class WriterListener;
class RTPSReader;
class ReaderAttributes;
class ReaderHistory;
class ReaderListener;


/**
 * Class RTPSDomain,it manages the creation and destruction of RTPSParticipant RTPSWriter and RTPSReader. It stores
 * a list of all created RTPSParticipant. Is has only static methods.
 * @ingroup RTPS_MODULE
 */
class RTPSDomain
{
	typedef std::pair<RTPSParticipant*,RTPSParticipantImpl*> t_p_RTPSParticipant;
private:
	RTPSDomain();
	/**
	 * DomainRTPSParticipant destructor
	 */
	~RTPSDomain();

public:
	/**
	 * Method to shut down all RTPSParticipants, readers, writers, etc.
	 * It must be called at the end of the process to avoid memory leaks.
	 * It also shut downs the DomainRTPSParticipant.
	 */
	RTPS_DllAPI static void stopAll();

	/**
	 * @brief Create a RTPSParticipant.
	 * @snippet fastrtps_example.cpp ex_RTPSParticipantCreation
	 * @param PParam RTPSParticipant Parameters.
	 * @param plisten Pointer to the ParticipantListener.
	 * @return Pointer to the RTPSParticipant.
	 */
	RTPS_DllAPI static RTPSParticipant* createParticipant(RTPSParticipantAttributes& PParam, RTPSParticipantListener* plisten = nullptr);

	/**
	 * Create a RTPSWriter in a participant.
	 * @param p Pointer to the RTPSParticipant.
	 * @param watt Writer Attributes.
	 * @param hist Pointer to the WriterHistory.
	 * @param listen Pointer to the WriterListener.
	 * @return Pointer to the created RTPSWriter.
	 */
	RTPS_DllAPI static RTPSWriter* createRTPSWriter(RTPSParticipant* p, WriterAttributes& watt, WriterHistory* hist, WriterListener* listen = nullptr);
	/**
	 * Remove a RTPSWriter.
	 * @param writer Pointer to the writer you want to remove.
	 * @return  True if correctly removed.
	 */
	RTPS_DllAPI static bool removeRTPSWriter(RTPSWriter* writer);

	/**
	 * Create a RTPSReader in a participant.
	 * @param p Pointer to the RTPSParticipant.
	 * @param ratt Reader Attributes.
	 * @param hist Pointer to the ReaderHistory.
	 * @param listen Pointer to the ReaderListener.
	 * @return Pointer to the created RTPSReader.
	 */
	RTPS_DllAPI static RTPSReader* createRTPSReader(RTPSParticipant* p, ReaderAttributes& ratt, ReaderHistory* hist, ReaderListener* listen = nullptr);
	/**
	 * Remove a RTPSReader.
	 * @param reader Pointer to the reader you want to remove.
	 * @return  True if correctly removed.
	 */
	RTPS_DllAPI static bool removeRTPSReader(RTPSReader* reader);

	/**
	 * Remove a RTPSParticipant and delete all its associated Writers, Readers, resources, etc.
	 * @param[in] p Pointer to the RTPSParticipant;
	 * @return True if correct.
	 */
	RTPS_DllAPI static bool removeRTPSParticipant(RTPSParticipant* p);

	/**
	 * Set the maximum RTPSParticipantID.
	 * @param maxRTPSParticipantId ID.
	 */
	static inline void setMaxRTPSParticipantId(uint32_t maxRTPSParticipantId) {
		m_maxRTPSParticipantID = maxRTPSParticipantId;
	}



private:
	static uint32_t m_maxRTPSParticipantID;

	static std::vector<t_p_RTPSParticipant> m_RTPSParticipants;

	/**
	 * @brief Get Id to create a RTPSParticipant.
	 * @return Different ID for each call.
	 */
	static inline uint32_t getNewId() {return m_maxRTPSParticipantID++;}

	static std::set<uint32_t> m_RTPSParticipantIDs;

};


}
} /* namespace  */
} /* namespace eprosima */

#endif /* DOMAINRTPSParticipant_H_ */
