#ifndef XML_PARSER_COMMON_H_
#define XML_PARSER_COMMON_H_

namespace eprosima {
namespace fastrtps {
namespace xmlparser {

#define draw(ident, text, ...) for (uint8_t i = ident + 1; i > 0; --i) (i == 1)? printf(text, ## __VA_ARGS__): printf("\t")

/**
 * Enum class XMLP_ret, used to provide a strongly typed result from the operations within this module.
 * @ingroup XMLPARSER_MODULE
 */
enum class XMLP_ret
{
    XML_ERROR,
    XML_OK,
    XML_NOK
};


extern const char* DEFAULT_FASTRTPS_PROFILES;

extern const char* PROFILES;
extern const char* PROFILE_NAME;
extern const char* PARTICIPANT;
extern const char* PUBLISHER;
extern const char* SUBSCRIBER;
extern const char* RTPS;

/// RTPS Participant attributes
extern const char* DEF_UNI_LOC_LIST;
extern const char* DEF_MULTI_LOC_LIST;
extern const char* DEF_OUT_LOC_LIST;
extern const char* DEF_SEND_PORT;
extern const char* SEND_SOCK_BUF_SIZE;
extern const char* LIST_SOCK_BUF_SIZE;
extern const char* BUILTIN;
extern const char* PORT;
extern const char* USER_DATA;
extern const char* PART_ID;
extern const char* IP4_TO_SEND;
extern const char* IP6_TO_SEND;
extern const char* THROUGHPUT_CONT;
extern const char* USER_TRANS;
extern const char* USE_BUILTIN_TRANS;
extern const char* PROPERTIES_POLICY;
extern const char* NAME;

/// Publisher-subscriber attributes
extern const char* TOPIC;
extern const char* QOS;
extern const char* TIMES;
extern const char* UNI_LOC_LIST;
extern const char* MULTI_LOC_LIST;
extern const char* OUT_LOC_LIST;
//extern const char* THROUGHPUT_CONT;
extern const char* EXP_INLINE_QOS;
extern const char* HIST_MEM_POLICY;
//extern const char* PROPERTIES_POLICY;
extern const char* USER_DEF_ID;
extern const char* ENTITY_ID;

///
extern const char* PROPERTIES;
extern const char* BIN_PROPERTIES;
extern const char* PROPERTY;
extern const char* VALUE;
extern const char* PROPAGATE;
extern const char* PREALLOCATED;
extern const char* PREALLOCATED_WITH_REALLOC;
extern const char* DYNAMIC;
extern const char* LOCATOR;
extern const char* KIND;
extern const char* ADDRESS;
extern const char* RESERVED;
extern const char* UDPv4;
extern const char* UDPv6;
extern const char* INIT_ACKNACK_DELAY;
extern const char* HEARTB_RESP_DELAY;
extern const char* INIT_HEARTB_DELAY;
extern const char* HEARTB_PERIOD;
extern const char* NACK_RESP_DELAY;
extern const char* NACK_SUPRESSION;
extern const char* BY_NAME;
extern const char* BY_VAL;
extern const char* _INFINITE;
extern const char* ZERO;
extern const char* INVALID;
extern const char* SECONDS;
extern const char* FRACTION;
extern const char* SHARED;
extern const char* EXCLUSIVE;

/// QOS
extern const char* DURABILITY;
extern const char* DURABILITY_SRV;
extern const char* DEADLINE;
extern const char* LATENCY_BUDGET;
extern const char* LIVELINESS;
extern const char* RELIABILITY;
extern const char* LIFESPAN;
extern const char* TIME_FILTER;
extern const char* OWNERSHIP;
extern const char* OWNERSHIP_STRENGTH;
extern const char* DEST_ORDER;
extern const char* PRESENTATION;
extern const char* PARTITION;
extern const char* TOPIC_DATA;
extern const char* GROUP_DATA;
extern const char* PUB_MODE;

extern const char* SYNCHRONOUS;
extern const char* ASYNCHRONOUS;
extern const char* NAMES;
extern const char* INSTANCE;
extern const char* GROUP;
extern const char* COHERENT_ACCESS;
extern const char* ORDERED_ACCESS;
extern const char* BY_RECEPTION_TIMESTAMP;
extern const char* BY_SOURCE_TIMESTAMP;
extern const char* MIN_SEPARATION;
extern const char* DURATION;
extern const char* MAX_BLOCK_TIME;
extern const char* _BEST_EFFORT;
extern const char* _RELIABLE;
extern const char* AUTOMATIC;
extern const char* MANUAL_BY_PARTICIPANT;
extern const char* MANUAL_BY_TOPIC;
extern const char* LEASE_DURATION;
extern const char* ANNOUNCE_PERIOD;
extern const char* PERIOD;
extern const char* SRV_CLEAN_DELAY;
extern const char* HISTORY_KIND;
extern const char* HISTORY_DEPTH;
extern const char* MAX_SAMPLES;
extern const char* MAX_INSTANCES;
extern const char* MAX_SAMPLES_INSTANCE;
extern const char* _VOLATILE;
extern const char* _TRANSIENT_LOCAL;
extern const char* _TRANSIENT;
extern const char* _PERSISTENT;
extern const char* KEEP_LAST;
extern const char* KEEP_ALL;
extern const char* _NO_KEY;
extern const char* _WITH_KEY;
extern const char* DATA_TYPE;
extern const char* HISTORY_QOS;
extern const char* RES_LIMITS_QOS;
extern const char* DEPTH;
extern const char* ALLOCATED_SAMPLES;
extern const char* BYTES_PER_SECOND;
extern const char* PERIOD_MILLISECS;
extern const char* PORT_BASE;
extern const char* DOMAIN_ID_GAIN;
extern const char* PARTICIPANT_ID_GAIN;
extern const char* OFFSETD0;
extern const char* OFFSETD1;
extern const char* OFFSETD2;
extern const char* OFFSETD3;
extern const char* SIMPLE_RTPS_PDP;
extern const char* WRITER_LVESS_PROTOCOL;
extern const char* _EDP;
extern const char* DOMAIN_ID;
extern const char* LEASEDURATION;
extern const char* LEASE_ANNOUNCE;
extern const char* SIMPLE_EDP;
extern const char* META_UNI_LOC_LIST;
extern const char* META_MULTI_LOC_LIST;
extern const char* INIT_PEERS_LIST;
extern const char* SIMPLE;
extern const char* STATIC;
extern const char* PUBWRITER_SUBREADER;
extern const char* PUBREADER_SUBWRITER;
extern const char* STATIC_ENDPOINT_XML;
extern const char* ACCESS_SCOPE;

// Endpoint parser
extern const char* STATICDISCOVERY;
extern const char* READER;
extern const char* WRITER;
extern const char* USER_ID;
extern const char* EXPECT_INLINE_QOS;
extern const char* TOPIC_NAME;
extern const char* TOPIC_DATA_TYPE;
extern const char* TOPIC_KIND;
extern const char* RELIABILITY_QOS;
extern const char* UNICAST_LOCATOR;
extern const char* MULTICAST_LOCATOR;
extern const char* _RELIABLE_RELIABILITY_QOS;
extern const char* _BEST_EFFORT_RELIABILITY_QOS;
extern const char* DURABILITY_QOS;
extern const char* _TRANSIENT_LOCAL_DURABILITY_QOS;
extern const char* _VOLATILE_DURABILITY_QOS;
extern const char* OWNERSHIP_QOS;
extern const char* OWNERSHIP_KIND_NOT_PRESENT;
extern const char* _SHARED_OWNERSHIP_QOS;
extern const char* _EXCLUSIVE_OWNERSHIP_QOS;
extern const char* PARTITION_QOS;
extern const char* LIVELINESS_QOS;
extern const char* LIVELINESS_KIND_NOT_PRESENT;
extern const char* _AUTOMATIC_LIVELINESS_QOS;
extern const char* _MANUAL_BY_PARTICIPANT_LIVELINESS_QOS;
extern const char* _MANUAL_BY_TOPIC_LIVELINESS_QOS;
extern const char* LEASE_DURATION_MS;
extern const char* _INF;
extern const char* EPROSIMA_UNKNOWN_STRING;
extern const char* _TRANSIENT_LOCAL_DURABILITY_QOS;
extern const char* _VOLATILE_DURABILITY_QOS;
extern const char* STRENGTH;


} /* xmlparser */
} /* namespace */
} /* namespace eprosima */

#endif
