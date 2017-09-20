#ifndef _BD_ASR_DEFINITIONS_H_
#define _BD_ASR_DEFINITIONS_H_

#include <string>

namespace bds {
/*
 * SDKType definition to load ASR SDK from
 * BDSpeechSDK::get_instance(const std::string &SDKType, std::string &outErrorDescription);
 */
    extern const std::string SDK_TYPE_ASR;

    /*================================ Message names for using ASR SDK ===============================*/
    extern const std::string ASR_CMD_CONFIG;
    extern const std::string ASR_CMD_START;
    extern const std::string ASR_CMD_PUSH_AUDIO;
    extern const std::string ASR_CMD_STOP;
    extern const std::string ASR_CMD_CANCEL;
    extern const std::string ASR_CMD_KWS_LOAD;
    extern const std::string ASR_CMD_KWS_UNLOAD;
    extern const std::string BDS_COMMAND_SET_WRITABLE_LIBRARY_DATA_PATH;

    /*=============================== Message names for Events from ASR ==============================*/
    extern const std::string asr_callback_name;

    extern const std::string CALLBACK_ASR_STATUS;
    extern const std::string CALLBACK_ASR_RESULT;
    extern const std::string CALLBACK_ASR_LEVEL;
    extern const std::string CALLBACK_ERROR_DESC;
    extern const std::string CALLBACK_ERROR_CODE;
    extern const std::string CALLBACK_ERROR_DOMAIN;

    /*================================ ASR SDK parameter keys ===============================*/
    /*
     * ASR_PARAM_KEY_CHUNK_KEY
     * Value explanation:   Chunk协议授权字段
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_CHUNK_KEY;

    /*
     * ASR_PARAM_KEY_CHUNK_PARAM
     * Value explanation:   Chunk协议透传字段
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_CHUNK_PARAM;

    /*
     * ASR_PARAM_KEY_CHUNK_ENABLE
     * Value explanation:   Chunk协议开关
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_CHUNK_ENABLE;

    extern const std::string BDS_PARAM_KEY_WRITABLE_LIBRARY_DATA_PATH;

/*================================ Params with Command ===============================*/
    /*
     * ASR_PARAM_KEY_REALTIME_DATA
     * Value explanation:   实时透传参数，随命令同步发送
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_REALTIME_DATA;

/*================================ Debug ===============================*/
    /*
     * COMMON_PARAM_KEY_DEBUG_LOG_LEVEL
     * Value explanation:   指定调试日志级别
     * Value type:          TBDVoiceRecognitionDebugLogLevel (int)
     * Default value:       EVRDebugLogLevelOff
     */
    extern const std::string COMMON_PARAM_KEY_DEBUG_LOG_LEVEL;

/*================================ AUDIO ===============================*/
    /*
     * MIC_PARAM_KEY_AUDIO_FILE_PATH
     * Value explanation:   设置音频文件路径（数据源）
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string MIC_PARAM_KEY_AUDIO_FILE_PATH;

    /*
     * MIC_PARAM_KEY_NEED_CACHE
     * Value explanation:   Need cached data in recorder.
     * Value type:          int
     * Value requirement:   Optional
     * Default value:       0
     */
    extern const std::string MIC_PARAM_KEY_NEED_CACHE;

    /*
     * MIC_PARAM_KEY_DISABLE_AUDIO_OPERATION
     * Value explanation:   Disable sdk audio operation (Set audio session disactive).
     * Value type:          int
     * Value requirement:   Optional
     * Default value:       0
     */
    extern const std::string MIC_PARAM_KEY_DISABLE_AUDIO_OPERATION;

/*================================ BASIC ===============================*/
    /*
     * ASR_PARAM_KEY_SDK_VERSION
     * Value explanation:   SDK version
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_SDK_VERSION;

    /*
     * ASR_PARAM_KEY_START_TONE
     * Value explanation:   录音开始提示音
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_START_TONE;

    /*
     * ASR_PARAM_KEY_STRATEGY
     * Value explanation:   语音识别策略
     * Value type:          TBDVoiceRecognitionStrategy (int)
     * Default value:       EVR_STRATEGY_ONLINE
     */
    extern const std::string ASR_PARAM_KEY_STRATEGY;

    /*
     * ASR_PARAM_KEY_SAMPLE_RATE
     * Value explanation:   设置录音采样率，自动模式根据当前网络情况自行调整
     * Value type:          TVoiceRecognitionRecordSampleRateFlags (int)
     * Default value:       EVoiceRecognitionRecordSampleRate16K
     */
    extern const std::string ASR_PARAM_KEY_SAMPLE_RATE;

/*================================ VAD-MFE ===============================*/
    /*
     * ASR_PARAM_KEY_MAX_WAIT_DURATION
     * Value explanation:   设置最大等待语音时间
     * Value type:          int (milliseconds)
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_MAX_WAIT_DURATION;

/*================================ SERVER FUNCTIONS ===============================*/
    /*
     * ASR_PARAM_KEY_DISABLE_PUNCTUATION
     * Value explanation:   是否关闭标点
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_DISABLE_PUNCTUATION;

    /*
     * ASR_PARAM_KEY_ENABLE_SERVER_VAD
     * Value explanation:   服务器端VAD
     * Value type:          int
     * Default value:       1
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_SERVER_VAD;

    /*
     * ASR_PARAM_KEY_ENABLE_CONTACTS
     * Value explanation:   开启通讯录识别功能，将优先返回通讯录识别结果，需事先上传通讯录
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_CONTACTS;

    /*
     * ASR_PARAM_KEY_ENABLE_EARLY_RETURN
     * Value explanation:   服务端开启提前返回，即允许服务端在未收到客户端发送的结束标志前提前结束识别过程
     * Value type:          int
     * Default value:       1
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_EARLY_RETURN;


/*================================ SERVER ===============================*/
    /*
     * ASR_PARAM_KEY_API_SECRET_KEYS
     * Value explanation:   设置API_KEY and SECRET_KEY
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_API_KEY;
    extern const std::string ASR_PARAM_KEY_SECRET_KEY;

    /*
     * ASR_PARAM_KEY_SERVER_URL
     * Value explanation:   设置服务器地址
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_SERVER_URL;

    /*
     * ASR_PARAM_KEY_BROWSER_UA
     * Value explanation:   设置浏览器标识(Http request header)，资源返回时会根据UA适配
     * Value type:          std::string
     * Default value:       -（可通过[UIWebView stringByEvaluatingJavaScriptFromString:@"navigator.userAgent"]获取）
     */
    extern const std::string ASR_PARAM_KEY_BROWSER_USER_AGENT;


/*================================ Recognition ===============================*/
    /*
     * ASR_PARAM_KEY_PROPERTY_LIST
     * Value explanation:   设置识别类型列表，输入法不可与其他类型复合
     * Value type:          std::string(JSON array)
     * Default value:       {"property_list":[EVoiceRecognitionPropertySearch]}
     *
     * NOTE:                Give a JSON string containing "property_list" key. The value for this key
     *                      should be a json array of integers.\
     *
     *Example:              To pass in EVoiceRecognitionPropertyMusic
     *                      and EVoiceRecognitionPropertyVideo the JSON should be as follows:
     *                      {"property_list":[10001,10002]}
     */
    extern const std::string ASR_PARAM_KEY_PROPERTY_LIST;

    /*
     * ASR_PARAM_KEY_PRODUCT_ID
     * Value explanation:   设置产品ID
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_PRODUCT_ID;

    /*
     * ASR_PARAM_KEY_CITY_ID
     * Value explanation:   设置城市ID，仅对地图识别类型有效
     * Value type:          int
     * Default value:       1（全国）
     */
    extern const std::string ASR_PARAM_KEY_CITY_ID;

    /*
     * ASR_PARAM_KEY_PROTOCOL
     * Value explanation:   设置协议类型
     * Value type:          TBDVoiceRecognitionProtocol (int)
     * Default value:       EPROTOCOL_DEFAULT
     */
    extern const std::string ASR_PARAM_KEY_PROTOCOL;

    /*
     * ASR_PARAM_KEY_LANGUAGE
     * Value explanation:   设置识别语言
     * Value type:          TBDVoiceRecognitionLanguage (int)
     * Default value:       EVoiceRecognitionLanguageChinese
     */
    extern const std::string ASR_PARAM_KEY_LANGUAGE;

    /*
     * ASR_PARAM_KEY_ENABLE_NLU
     * Value explanation:   开启语义解析，将返回包含语音的json串
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_NLU;

    /*
     * ASR_PARAM_KEY_ENABLE_LOCAL_VAD
     * Value explanation:   是否需要对录音数据进行端点检测
     * Value type:          int
     * Default value:       1
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_LOCAL_VAD;

    /*
     * ASR_PARAM_KEY_ENABLE_MODEL_VAD
     * Value explanation:   是否使用modelVAD
     * Value type:          int
     * Default value:       0
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_MODEL_VAD;

    /*
     * ASR_PARAM_KEY_MODEL_VAD_DAT_FILE
     * Value explanation:   modelVAD所需资源文件
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_MODEL_VAD_DAT_FILE;

    /*
     * ASR_PARAM_KEY_COMPRESSION_TYPE
     * Value explanation:   录音数据压缩算法
     * Value type:          TBDVoiceRecognitionAudioCompressionType (int)
     * Default value:       EVR_AUDIO_COMPRESSION_BV32
     */
    extern const std::string ASR_PARAM_KEY_COMPRESSION_TYPE;

    /*
     * ASR_PARAM_KEY_ENABLE_DRC
     * Value explanation:   是否进行车载环境下的噪声消除
     * Value type:          int
     * Default value:       1
     */
    extern const std::string ASR_PARAM_KEY_ENABLE_DRC;

/*================================ 扩展参数 ===============================*/
    /*
     * ASR_PARAM_KEY_PAM
     * Value explanation:   扩展参数，多轮对话需要的信息
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_PAM;

    /*
     * ASR_PARAM_KEY_STC
     * Value explanation:   扩展参数，统计信息
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_STC;

    /*
     * ASR_PARAM_KEY_LTP
     * Value explanation:   扩展参数，轻应用参数（uid）
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_LTP;

    /*
     * ASR_PARAM_KEY_TXT
     * Value explanation:   扩展参数，上传文本，如果设置了该字段，将略过语音输入和识别阶段
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_TXT;

    /*
     * ASR_PARAM_KEY_BUA
     * Value explanation:   浏览器标识, use for wise
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_BUA;

/*================================ Configuration for contract with server ===============================*/
    /*
     * ASR_PARAM_KEY_NETWORK_STATUS
     * Value explanation:   当前网络状态
     * Value type:          TBDVoiceRecognitionNetworkType (int)
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_NETWORK_STATUS;

    /*
     * ASR_PARAM_KEY_APP
     * Value explanation:   App name
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_APP;

    /*
     * ASR_PARAM_KEY_PLATFORM
     * Value explanation:   Platform
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_PLATFORM;

    /*
     * ASR_PARAM_KEY_COK
     * Value explanation:   Cookie, use for wise
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_COK;

    /*
     * ASR_PARAM_KEY_PU
     * Value explanation:   Pu, use for wise
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_PU;

    /*
     * ASR_PARAM_KEY_FRM
     * Value explanation:   From, use for wise
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_FRM;

    /*
     * ASR_PARAM_KEY_RSV
     * Value explanation:   Reserve, use for wise
     * Value type:          std::string (json)
     * Default value:       -
     * Note:                Give a json formatted string containing string parameters with keys.
     * Example:             To pass in 2 parameters (reserved_1 and reserved_2 with values "value 1" and "value 2")
     *                      pass following json string.
     *                      {"reserved_1":"value 1","reserved_2":"value 2"}
     */
    extern const std::string ASR_PARAM_KEY_RSV;

/*================================ Offline Engine Verify ===============================*/
    /*
     * OFFLINE_PARAM_KEY_APP_CODE
     * Value explanation:   离线授权所需APPCODE
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string OFFLINE_PARAM_KEY_APP_CODE;

    /*
     * OFFLINE_PARAM_KEY_LICENSE_FILE_PATH
     * Value explanation:   离线授权文件路径
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string OFFLINE_PARAM_KEY_LICENSE_FILE_PATH;

/*================================ Offline Engine KWS ===============================*/
    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_TYPE
     * Value explanation:   离线识别引擎类型
     * Value type:          TBDVoiceRecognitionOfflineEngineType (int)
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_TYPE;

    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_DAT_FILE_PATH
     * Value explanation:   离线识别资源文件路径
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_DAT_FILE_PATH;

    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_GRAMMER_FILE_PATH
     * Value explanation:   离线识别语法文件路径
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_GRAMMER_FILE_PATH;

    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_GRAMMER_SLOT
     * Value explanation:   语法模式离线语法槽
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_GRAMMER_SLOT;

    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_WAKEUP_WORDS_FILE_PATH
     * Value explanation:   唤醒词文件路径，使用了唤醒并使用离线识别的情况下需要设置，其他情况请忽略该参数
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_WAKEUP_WORDS_FILE_PATH;

    /*
     * ASR_PARAM_KEY_OFFLINE_ENGINE_TRIGGERED_WAKEUP_WORD
     * Value explanation:   当前触发唤醒词，唤醒后立即调用识别的情况下配置，其他情况请忽略该参数
     * Value type:          std::string
     * Default value:       -
     */
    extern const std::string ASR_PARAM_KEY_OFFLINE_ENGINE_TRIGGERED_WAKEUP_WORD;
}

#endif
