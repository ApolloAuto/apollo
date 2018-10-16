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
 * @file XMLEndpointParser.h
 *
 */

#ifndef XMLENDPOINTPARSER_H_
#define XMLENDPOINTPARSER_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <fastrtps/xmlparser/XMLParserCommon.h>
#include <set>
#include <vector>
#include <cstdint>

namespace tinyxml2
{
    class XMLElement;
}

using namespace tinyxml2;

namespace eprosima{
namespace fastrtps{
namespace rtps{
class ReaderProxyData;
class WriterProxyData;
}

namespace xmlparser{

/**
 * Class StaticRTPSParticipantInfo, contains the information of writers and readers loaded from the XML file.
 *@ingroup DISCOVERY_MODULE
 */
class StaticRTPSParticipantInfo{
public:
	StaticRTPSParticipantInfo(){};
	virtual ~StaticRTPSParticipantInfo(){};
	//!RTPS PArticipant name
	std::string m_RTPSParticipantName;
	//!Vector of ReaderProxyData pointer
	std::vector<rtps::ReaderProxyData*> m_readers;
	//!Vector of ReaderProxyData pointer
	std::vector<rtps::WriterProxyData*> m_writers;
};

/**
 * Class XMLEndpointParser used to parse the XML file that contains information about remote endpoints.
 * @ingroup DISCVOERYMODULE
 */
class XMLEndpointParser {
public:
	XMLEndpointParser();
	virtual ~XMLEndpointParser();
	/**
	 * Load the XML file
	 * @param filename Name of the file to load and parse.
	 * @return True if correct.
	 */
	XMLP_ret loadXMLFile(std::string& filename);

    void loadXMLParticipantEndpoint(tinyxml2::XMLElement* xml_endpoint, StaticRTPSParticipantInfo* pdata);

	/**
	 * Load a Reader endpoint.
	 * @param xml_endpoint Reference of a tree child for a reader.
	 * @param pdata Pointer to the RTPSParticipantInfo where the reader must be added.
	 * @return True if correctly added.
	 */
	XMLP_ret loadXMLReaderEndpoint(tinyxml2::XMLElement* xml_endpoint,StaticRTPSParticipantInfo* pdata);
	/**
	 * Load a Writer endpoint.
	 * @param xml_endpoint Reference of a tree child for a writer.
	 * @param pdata Pointer to the RTPSParticipantInfo where the reader must be added.
	 * @return True if correctly added.
	 */
	XMLP_ret loadXMLWriterEndpoint(tinyxml2::XMLElement* xml_endpoint, StaticRTPSParticipantInfo* pdata);
	/**
	 * Look for a reader in the previously loaded endpoints.
	 * @param[in] partname RTPSParticipant name
	 * @param[in] id Id of the reader
	 * @param[out] rdataptr Pointer to pointer to return the information.
	 * @return True if found.
	 */
	XMLP_ret lookforReader(std::string partname, uint16_t id, rtps::ReaderProxyData** rdataptr);
	/**
	 * Look for a writer in the previously loaded endpoints.
	 * @param[in] partname RTPSParticipant name
	 * @param[in] id Id of the writer
	 * @param[out] wdataptr Pointer to pointer to return the information.
	 * @return True if found
	 */
	XMLP_ret lookforWriter(std::string partname, uint16_t id, rtps::WriterProxyData** wdataptr);

private:
	std::set<int16_t> m_endpointIds;
	std::set<uint32_t> m_entityIds;

	std::vector<StaticRTPSParticipantInfo*> m_RTPSParticipants;
};


} /* xmlparser */
} /* namespace */
} /* namespace eprosima */

#endif
#endif /* XMLENDPOINTPARSER_H_ */
