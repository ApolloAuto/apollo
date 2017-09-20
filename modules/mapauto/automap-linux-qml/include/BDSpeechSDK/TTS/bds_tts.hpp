#ifndef BDS_TTS_HPP
#define BDS_TTS_HPP

#endif // BDS_TTS_HPP

#include <string>

namespace bds{
//======================= constant =========================

extern const std::string SDK_TYPE_TTS;
	
/*
 * BDS_PLATFORM_EVENT_PAUSE
 * Command explanation:     Pause synthesis and playback
 * Required parameters:     -
 */
extern const std::string BDS_PLATFORM_EVENT_PAUSE;
/*
 * BDS_PLATFORM_EVENT_RESUME
 * Command explanation:     Resume synthesis and playback
 * Required parameters:     -
 */
extern const std::string BDS_PLATFORM_EVENT_RESUME;
/*
 * BDS_PLATFORM_EVENT_STOP
 * Command explanation:     Stop synthesis and playback
 * Required parameters:     -
 */
extern const std::string BDS_PLATFORM_EVENT_STOP;

//======================= command =========================

/*
 * BDS_TTS_CMD_LOAD_CONFIG
 * Command explanation:     Modify engine configurations.
 *                          Any existing configurations that are not declared in the passed params map
 *                          will be kept un modified. The ones that are declared will either be added or
 *                          will override the existing values.
 *                          For combined SDK this command is also used for loading offline engine.
 * Required parameters:     - (params map should contain configurations for SDK)
 *
 * Triggered events:        BDS_TTS_RESULT_LOAD_CONFIG
 */
extern const std::string BDS_TTS_CMD_LOAD_CONFIG;
/*
 * BDS_TTS_CMD_QUEUE_SENTENCE
 * Command explanation:     Begin synthesizing with current configuration/queue more text to synthesis
 * Required parameters:     BDS_TTS_QUEUE_SENTENCE_PARAM_TEXT_UINT8
 *                          BDS_TTS_QUEUE_SENTENCE_PARAM_ID_STRING
 *
 *
 * Triggered events:        BDS_TTS_RESULT_QUEUE_SENTENCE
 *
 */
extern const std::string BDS_TTS_CMD_QUEUE_SENTENCE;
/*
 * BDS_TTS_CMD_LOAD_OFFLINE_ENGINE
 * Command explanation:     Loads offline engine
 * Required parameters:     - (Map may contain configurations for SDK/offline engine, the passed
 *                             configs will be applied before attempting to load enging)
 *
 * Triggered events:        BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE
 */
extern const std::string BDS_TTS_CMD_LOAD_OFFLINE_ENGINE;
/*
 * BDS_TTS_CMD_LOAD_OFFLINE_ENGINE_ENGLISH_DATA
 * Command explanation:     Loads english data file to offline engine
 * Required parameters:     - BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_TEXT_DATA_PATH
 *                          - BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_SPEECH_DATA_PATH
 *
 * Triggered events:        BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE_ENGLISH_DATA
 */
extern const std::string BDS_TTS_CMD_LOAD_OFFLINE_ENGINE_ENGLISH_DATA;
/*
 * BDS_TTS_CMD_RELOAD_OFFLINE_ENGINE_DATA
 * Command explanation:     Reload data to offline engine
 * Required parameters:     - BDS_FILE_PATH
 *
 * Triggered events:        BDS_TTS_RESULT_RELOAD_OFFLINE_ENGINE_DATA
 */
extern const std::string BDS_TTS_CMD_RELOAD_OFFLINE_ENGINE_DATA;
/*
 * BDS_TTS_CMD_UNLOAD_OFFLINE_ENGINE
 * Command explanation:     Unloads offline engine (if any loaded)
 * Required parameters:     -
 *
 * Triggered events:        BDS_TTS_RESULT_UNLOAD_OFFLINE_ENGINE
 */
extern const std::string BDS_TTS_CMD_UNLOAD_OFFLINE_ENGINE;
/*
 * BDS_TTS_CMD_VERIFY_OFFLINE_DATA_FILE
 * Command explanation:     Verify passed offline engine data file
 * Required parameters:     BDS_FILE_PATH
 *
 * Triggered events:        BDS_TTS_RESULT_VERIFY_OFFLINE_DATA_FILE
 *                          request will fail if offline TTS support is not enabled
 */
extern const std::string BDS_TTS_CMD_VERIFY_OFFLINE_DATA_FILE;
/*
 * BDS_TTS_CMD_GET_OFFLINE_DATA_FILE_PARAMS
 * Command explanation:
 * Required parameters:     BDS_FILE_PATH
 *
 * Triggered events:        BDS_TTS_RESULT_GET_OFFLINE_DATA_FILE_PARAMS
 *                          Request will fail if offline TTS support is not enabled
 */
extern const std::string BDS_TTS_CMD_GET_OFFLINE_DATA_FILE_PARAMS;
/*
 * BDS_TTS_CMD_GET_OFFLINE_ENGINE_VERSION
 * Command explanation:     Get offline engine version
 * Required parameters:     -
 *
 * Triggered events:        BDS_TTS_RESULT_GET_OFFLINE_ENGINE_VERSION
 *                          Request will fail if offline TTS support is not enabled
 */
extern const std::string BDS_TTS_CMD_GET_OFFLINE_ENGINE_VERSION;
extern const std::string BDS_COMMAND_GET_EVENT_MANAGER_VERSION;
	
extern const std::string BDS_CORE_PLATFORM_CUSTOM_COMMAND;

extern const std::string BDS_COMMAND_SET_WRITABLE_LIBRARY_DATA_PATH;

//============================== callback =============================

/*
 * BDS_PLATFORM_EVENT_VOLUME
 * Command explanation:     Set player volume
 * Required parameters:     BDS_TTS_VOLUME_PARAM_VOLUME_INT
 */
extern const std::string BDS_PLATFORM_EVENT_VOLUME;
/*
 * BDS_PLATFORM_EVENT_CONNECTIVITY_CHANGED
 * Command explanation:     Notify the SDK about changes in internet connectivity.
 * Required parameters:     BDS_PLATFORM_EVENT_CONNECTIVITY_CHANGED_PARAM_NET_STATUS_INT
 *
 */
extern const std::string BDS_PLATFORM_EVENT_CONNECTIVITY_CHANGED;
extern const std::string BDS_CALLBACK_EVENT_MANAGER_VERSION;
extern const std::string BDS_TTS_RESULT_LOAD_CONFIG;
extern const std::string BDS_PLATFORM_EVENT_STREAM_TYPE;
/*
 * BDS_TTS_EVENT_FILTER
 * Event explanation:       Not an event, callback may use this string to filter tts events from other messages.
 *                          All tts events begin with this string.
 * Event parameters:        -
 */
extern const std::string BDS_TTS_EVENT_FILTER;
/*
 * BDS_TTS_EVENT_BEGIN_SYNTHESIS
 * Event explanation:       Triggered when SDK begins synthesizing a new string.
 * Event parameters:        BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 */
extern const std::string BDS_TTS_EVENT_BEGIN_SYNTHESIS;
/*
 * BDS_TTS_EVENT_AUDIO_DATA
 * Event explanation:       Triggered when SDK receives more audio data for string
 * Event parameters:        BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_CHAR_COUNT_INT
 *                          BDS_TTS_EVENT_PARAM_AUDIO_FMT_INT
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 */
extern const std::string BDS_TTS_EVENT_AUDIO_DATA;
/*
 * BDS_TTS_EVENT_FINISH_SYNTHESIS
 * Event explanation:       Triggered when SDK finishes synthesizing a string.
 * Event parameters:        BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 *                          BDS_PARAM_KEY_ERROR_CODE            (If finish due to an error, may exist with 0 value in no errors happened)
 *                          BDS_PARAM_KEY_ERROR_DESCRIPTION    (If finish due to error, may exist with "OK" value if no errors happened)
 */
extern const std::string BDS_TTS_EVENT_FINISH_SYNTHESIS;
/*
 * BDS_TTS_EVENT_BEGIN_PLAY
 * Event explanation:       Triggered when SDK begins playing a new string.
 * Event parameters:        BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 */
extern const std::string BDS_TTS_EVENT_BEGIN_PLAY;
/*
 * BDS_TTS_EVENT_PLAY_PROGRESS
 * Event explanation:       Triggered to indicate charater progress change of tts playback.
 * Event parameters:        BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_CHAR_COUNT_INT
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 */
extern const std::string BDS_TTS_EVENT_PLAY_PROGRESS;
/*
 * BDS_TTS_EVENT_FINISH_PLAY
 * Event explanation:       Triggered when SDK finishes playing a string.
 * Event parameters:        BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 *                          BDS_PARAM_KEY_ERROR_CODE            (If finish due to an error, may exist with 0 value in no errors happened)
 *                          BDS_PARAM_KEY_ERROR_DESCRIPTION    (If finish due to error, may exist with "OK" value if no errors happened)
 */
extern const std::string BDS_TTS_EVENT_FINISH_PLAY;
/*
 * BDS_TTS_EVENT_DID_STOP
 * Event explanation:       Triggered when SDK stops all it's work for any of the following reasons.
 *                          - User stopped by sending BDS_TTS_CMD_STOP command.
 *                          - Nothing more to synthesize/playback.
 *                          - An error ocurred (Message.params will contain error code and description in this case).
 * Event parameters:        BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING             (If stopped due to synthesizer error)
 *                          BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING             (If stopped due to player error)
 *                          BDS_PARAM_KEY_ERROR_CODE            (If stopped due to an error, may exist with 0 value in no errors happened)
 *                          BDS_PARAM_KEY_ERROR_DESCRIPTION    (If stopped due to error, may exist with "OK" value if no errors happened)
 *                          BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 */
extern const std::string BDS_TTS_EVENT_DID_STOP;
/*
 * BDS_TTS_RESULT_FILTER
 * Event explanation:       Not an event, callback may use this string to filter tts command results from other
 *                          messages. All tts command result events begin with this string.
 * Event parameters:        -
 */
extern const std::string BDS_TTS_RESULT_FILTER;
/*
 * BDS_TTS_RESULT_QUEUE_SENTENCE
 * Event explanation:       Triggered to indicate the result of BDS_TTS_CMD_QUEUE_SENTENCE
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE
 *                          * 0 on success.
 *                          * encoded error with domain = ERROR_DOMAIN_TTS and code = TTS_ERROR_TEXT_TOO_LONG if too long text
 *                          * encoded error with domain = ERROR_DOMAIN_TTS and code = TTS_ERROR_TEXT_TOO_SHORT if too short text
 *                          * encoded error with domain = ERROR_DOMAIN_TTS_CORE_INTERNAL
 *                            and code = TTS_CORE_ERROR_CODE_CMD_MISSING_PARAM if request is missing required parameters, check
 *                            error description to find which parameter is missing
 *                          * Any other error will result to 0, followed by an
 *                            error callback by BDS_TTS_EVENT_DID_STOP
 */
extern const std::string BDS_TTS_RESULT_QUEUE_SENTENCE;
/*
 * BDS_TTS_RESULT_LOAD_CONFIG
 * Event explanation:       Triggered to indicate the result of BDS_TTS_CMD_LOAD_CONFIG
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          Parameters can't be modified during ongoing synthesis/playback.
 */
extern const std::string BDS_TTS_RESULT_LOAD_CONFIG;
/*
 * BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE
 * Event explanation:       Triggered to indicate the result of BDS_TTS_CMD_LOAD_OFFLINE_ENGINE
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          Load can't be attempted during ongoing synthesis.
 */
extern const std::string BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE;
/*
 * BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE_ENGLISH_DATA
 * Event explanation:       Triggered to indicate the result of BDS_TTS_CMD_LOAD_OFFLINE_ENGINE_ENGLISH_DATA
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          Load can't be attempted during ongoing synthesis.
 */
extern const std::string BDS_TTS_RESULT_LOAD_OFFLINE_ENGINE_ENGLISH_DATA;
/*
 * BDS_TTS_RESULT_RELOAD_OFFLINE_ENGINE_DATA
 * Event explanation:       Triggered to indicate the result of BDS_TTS_RESULT_RELOAD_OFFLINE_ENGINE_DATA
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          Reload can't be attempted during ongoing synthesis.
 */
extern const std::string BDS_TTS_RESULT_RELOAD_OFFLINE_ENGINE_DATA;
/*
 * BDS_TTS_RESULT_UNLOAD_OFFLINE_ENGINE
 * Event explanation:       Triggered to indicate result of BDS_TTS_CMD_UNLOAD_OFFLINE_ENGINE
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          Unload can't be attempted during ongoing synthesis.
 *                          If engine wasn't loaded, the unload resut will be 0
 */
extern const std::string BDS_TTS_RESULT_UNLOAD_OFFLINE_ENGINE;
/*
 * BDS_TTS_RESULT_VERIFY_OFFLINE_DATA_FILE
 * Event explanation:       Triggered to indicate result of BDS_TTS_CMD_VERIFY_OFFLINE_DATA_FILE
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          BDS_TTS_OFFLINE_DATA_FILE_VERIFY_RESULT_BOOL (available only if BDS_PARAM_KEY_ERROR_CODE == 0)
 */
extern const std::string BDS_TTS_RESULT_VERIFY_OFFLINE_DATA_FILE;
/*
 * BDS_TTS_RESULT_GET_OFFLINE_DATA_FILE_PARAMS
 * Event explanation:       Triggered to indicate result of BDS_TTS_CMD_GET_OFFLINE_DATA_FILE_PARAMS
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          BDS_TTS_OFFLINE_DATA_FILE_INFO_JSON (available only if BDS_PARAM_KEY_ERROR_CODE == 0)
 */
extern const std::string BDS_TTS_RESULT_GET_OFFLINE_DATA_FILE_PARAMS;
/*
 * BDS_TTS_RESULT_GET_OFFLINE_ENGINE_VERSION
 * Event explanation:       Triggered to indicate result of BDS_TTS_CMD_GET_OFFLINE_ENGINE_VERSION
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 *                          BDS_TTS_OFFLINE_ENGINE_VERSION_JSON (available only if BDS_PARAM_KEY_ERROR_CODE == 0)
 */
extern const std::string BDS_TTS_RESULT_GET_OFFLINE_ENGINE_VERSION;
/*
 * BDS_TTS_RESULT_NET_CONNECTIVITY_CHANGED
 * Event explanation:       Triggered to indicate the result of BDS_TTS_CMD_NET_CONNECTIVITY_CHANGED
 * Event parameters:        BDS_PARAM_KEY_ERROR_DESCRIPTION
 *                          BDS_PARAM_KEY_ERROR_CODE (0 on success)
 */
extern const std::string BDS_TTS_RESULT_NET_CONNECTIVITY_CHANGED;

extern const std::string BDS_TTS_RESULT_VOLUME;

//=========================== params ================================
 /*
     * BDS_TTS_PARAM_KEY_STRATEGY_INT Value explanation: Strategy used for TTS Value type: 
     * (TTSStrategy_Online/TTSStrategy_Offline/TTSStrategy_Online_Pri/TTSStrategy_Offline_Pri) Default value:
     * TTSStrategy_Online_Pri
     */
extern const std::string BDS_TTS_PARAM_KEY_STRATEGY_INT;
/*
 * BDS_TTS_CMD_VOLUME
 * Parameter explanation:   Text to synthesize
 * Value type:              bds_shared_ptr<uint8_t> (zero terminated utf-8 data)
 */
extern const std::string BDS_TTS_QUEUE_SENTENCE_PARAM_TEXT_UINT8;
/*
 * BDS_TTS_QUEUE_SENTENCE_PARAM_ID_STRING
 * Parameter explanation:   Task ID for synthesize text. The ID is used for following purposes:
 *                          - As BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING parameter in tts events emitted by EventManagerTTS.
 *                          - As BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING parameter in tts events emitted by EventManagerTTS.
 *                          - As SN parameter when making requests to online tts service.
 *                          - As task ID while passing events about synthesis task internally
 * Value type:              std::string (UUID string)
 */
extern const std::string BDS_TTS_QUEUE_SENTENCE_PARAM_ID_STRING;
/*
 * BDS_TTS_QUEUE_SENTENCE_PARAM_ENABLE_SPEAK_INT
 * Value explanation:   Should the SDK speak, or only synthesize.
 * Value type:          int (0 = only synthesize, 1 = synthesize and speak)
 * Default value:       1 (speak if key is missing)
 */
extern const std::string BDS_TTS_QUEUE_SENTENCE_PARAM_ENABLE_SPEAK_INT;
/*
 * BDS_PLATFORM_EVENT_VOLUME_PARAM_VOLUME_INT
 * Parameter explanation:   Audio player volume setting
 * Value type:              int (value between 0 and 100, 0 being silent and 100 full volume)
 * Default:                 50
 */
extern const std::string BDS_PLATFORM_EVENT_VOLUME_PARAM_VOLUME_INT;
/*
 * BDS_PLATFORM_EVENT_CONNECTIVITY_CHANGED_PARAM_NET_STATUS_INT
 * Parameter explanation:   The current internet connectivity status
 * Value type:              int
 *                          0: Internet is not available
 *                          1: Internet is available, but the connection quality may be bad. Prefer offline engine.
 *                          2: Internet is available, online engine should have preference unless stated otherwice
 *                             in the configurations.
 * Default:                 2

 */
extern const std::string BDS_PLATFORM_EVENT_CONNECTIVITY_CHANGED_PARAM_NET_STATUS_INT;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_LOAD_CHINESE_TEXT_DATA_PATH
 * Value explanation:   Chinese text data to be loaded to embedded engine.
 *
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_LOAD_CHINESE_TEXT_DATA_PATH;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_LOAD_CHINESE_SPEECH_DATA_PATH
 * Value explanation:   Chinese speech data to be loaded to embedded engine.
 *
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_LOAD_CHINESE_SPEECH_DATA_PATH;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_TEXT_DATA_PATH
 * Value explanation:   English text data to be loaded to embedded engine.
 *
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_TEXT_DATA_PATH;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_SPEECH_DATA_PATH
 * Value explanation:   English speech data to be loaded to embedded engine.
 *
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_LOAD_ENGLISH_SPEECH_DATA_PATH;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_XML_INT
 * Value explanation:   Determines if xml support should be enabled in embedded engine
 * Value type:          int (1/0)
 * Default value:       0
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_XML_INT;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_DOMAIN_SYNTH_INT
 * Value explanation:   Determines if domain synthesis should be enabled in embedded engine.
 * Value type:          int (1/0)
 * Default value:       0
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_DOMAIN_SYNTH_INT;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_AUDIO_FMT_INT
 * Value explanation:   Configure audio format produced by embedded engine
 * Value type:          int (enum ETTS_AUDIO_TYPE defines valid values)
 * Default value:       ETTS_AUDIO_TYPE_PCM_16K
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_AUDIO_FMT_INT;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_OPTIMIZATION_LEVEL_INT
 * Value explanation:   VO coder optimization level used by embedded engine.
 * Value type:          int (enum ETTS_VOCODER_OPTIM_LEVEL defines valid values)
 * Default value:       ETTS_VOCODER_OPTIM_LEVEL_0
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_OPTIMIZATION_LEVEL_INT;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_APP_CODE_STRING
 * Value explanation:   App code to use with offline engine
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_APP_CODE_STRING;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_LICENSE_PATH_STRING
 * Value explanation:   File path to license file to be used with embedded engine
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_LICENSE_PATH_STRING;
/*
 * BDS_TTS_PARAM_KEY_OFFLINE_DOMAIN_DATA_FILE
 * Value explanation:   Domain data file path for offline engine.
 *                      Pass empty string to unload domain data.
 * Value type:          std::string
 * Default value:       None
 */
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_DOMAIN_DATA_FILE_PATH_STRING;
/*
 * BDS_FILE_PATH
 * Value explanation:   Generic file path param.
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string BDS_FILE_PATH;

extern const std::string BDS_PARAM_API_KEY;
extern const std::string BDS_PARAM_SECRET_KEY;
extern const std::string BDS_PARAM_KEY_EVENT_MANAGER_VERSION_STRING;
extern const std::string BDS_PLATFORM_PARAM_STREAM_TYPE;
extern const std::string COMMON_PARAM_KEY_DEBUG_LOG_LEVEL;
/*
 * BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING
 * Parameter explanation:   Task ID of the string being spoken
 * Value type:              std::string (value passed as BDS_TTS_QUEUE_SENTENCE_PARAM_ID_STRING with BDS_TTS_CMD_QUEUE_SENTENCE)
 */
extern const std::string BDS_TTS_EVENT_PARAM_SPEAK_ID_STRING;
/*
 * BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING
 * Parameter explanation:   Task ID of the string being synthesized
 * Value type:              std::string (value passed as BDS_TTS_QUEUE_SENTENCE_PARAM_ID_STRING with BDS_TTS_CMD_QUEUE_SENTENCE)
 */
extern const std::string BDS_TTS_EVENT_PARAM_SYNTH_ID_STRING;
/*
 * BDS_TTS_EVENT_PARAM_CHAR_COUNT_INT
 * Parameter explanation:   Character progress.
 * Value type:              int
 */
extern const std::string BDS_TTS_EVENT_PARAM_CHAR_COUNT_INT;
/*
 * BDS_TTS_EVENT_PARAM_AUDIO_FMT_INT
 * Parameter explanation:   Format of data contained in BDS_TTS_EVENT_PARAM_AUDIO_DATA_UINT8
 * Value type:              int (one of values defined in enum BDSAudioFormat)
 */
extern const std::string BDS_TTS_EVENT_PARAM_AUDIO_FMT_INT;
/*
 * BDS_PARAM_KEY_ERROR_CODE
 * Parameter explanation:   Error code
 * Value type:              int32_t
 */
extern const std::string BDS_PARAM_KEY_ERROR_CODE;
/*
 * BDS_PARAM_KEY_ERROR_DESCRIPTION
 * Parameter explanation:   Error description.
 * Value type:              std::string
 */
extern const std::string BDS_PARAM_KEY_ERROR_DESCRIPTION;
/*
 * BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL
 * Parameter explanation:   Indicates if the sentence is license notification sentence, which is
 *                          a sentence added internally by SDK every 20 sentences if offline engine has
 *                          has license issues.
 *                          This flag may be used by outer layer to detect these callbacks and process
 *                          them differently or just ignore them.
 * Value type:              bool
 */
extern const std::string BDS_TTS_EVENT_PARAM_IS_LICENSE_NOTIFICATION_BOOL;
/*
 * BDS_TTS_OFFLINE_DATA_FILE_VERIFY_RESULT_BOOL
 * Parameter explanation:   true if data file passed verifiation, false if verification did not pass
 * Value type:              bool
 */
extern const std::string BDS_TTS_OFFLINE_DATA_FILE_VERIFY_RESULT_BOOL;
/*
 * BDS_TTS_OFFLINE_DATA_FILE_INFO_JSON
 * Parameter explanation:   JSON string containing info about data file.
 * Value type:              string
 */
extern const std::string BDS_TTS_OFFLINE_DATA_FILE_INFO_JSON;
/*
 * BDS_TTS_OFFLINE_ENGINE_VERSION_JSON
 * Parameter explanation:   JSON string containing offline engine version info
 * Value type:              string
 */
extern const std::string BDS_TTS_OFFLINE_ENGINE_VERSION_JSON;

extern const std::string DATA_CHUNK;
	
extern const std::string BDS_CORE_PLATFORM_CUSTOM_COMMAND_NAME_STRING;

extern const std::string BDS_TTS_PARAM_KEY_ONLINE_SPEAKER_INT;
extern const std::string BDS_TTS_PARAM_KEY_SPEED_INT;
extern const std::string BDS_TTS_PARAM_KEY_PITCH_INT;
extern const std::string BDS_TTS_PARAM_KEY_VOLUME_INT;
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_OPTIMIZATION_LEVEL_INT;
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_DOMAIN_SYNTH_INT;
extern const std::string BDS_TTS_PARAM_KEY_OFFLINE_ENABLE_XML_INT;
extern const std::string BDS_TTS_PARAM_KEY_ONLINE_PID_STRING;
extern const std::string BDS_TTS_PARAM_KEY_ONLINE_LANGUAGE_STRING;
extern const std::string BDS_PARAM_KEY_WRITABLE_LIBRARY_DATA_PATH;

}
