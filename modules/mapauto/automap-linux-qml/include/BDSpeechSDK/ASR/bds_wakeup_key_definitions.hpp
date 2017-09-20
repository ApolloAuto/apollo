//
//  bds_WakeupParameters.hpp
//  BDASRCore
//
//  Created by baidu on 16/5/19.
//  Copyright © 2016年 baidu. All rights reserved.
//

#ifndef bds_WakeupParameters_hpp
#define bds_WakeupParameters_hpp

#include <string>

namespace bds {
/*
 * SDKType definition to load Wakeup SDK from
 * BDSpeechSDK::get_instance(const std::string &SDKType, std::string &outErrorDescription);
 */
extern const std::string SDK_TYPE_WAKEUP;

/*================================ Message names for using Wakeup SDK ===============================*/

extern const std::string WAK_CMD_CONFIG;
extern const std::string WAK_CMD_START;
extern const std::string WAK_CMD_PUSH_AUDIO;
extern const std::string WAK_CMD_STOP;
extern const std::string WAK_CMD_LOAD_ENGINE;
extern const std::string WAK_CMD_UNLOAD_ENGINE;
extern const std::string BDS_COMMAND_SET_WRITABLE_LIBRARY_DATA_PATH;

/*============================= Message names for events from Wakeup SDK ============================*/
extern const std::string wakeup_callback_name;

extern const std::string CALLBACK_WAK_STATUS;
extern const std::string CALLBACK_WAK_RESULT;
extern const std::string CALLBACK_ERROR_DESC;
extern const std::string CALLBACK_ERROR_DOMAIN;
extern const std::string CALLBACK_ERROR_CODE;

/*================================ Wakeup command parameters ===============================*/

#pragma mark - Debug

/*
 * COMMON_PARAM_KEY_DEBUG_LOG_LEVEL
 * Value explanation:   指定调试日志级别
 * Value type:          TBDVoiceRecognitionDebugLogLevel (int)
 * Default value:       EVRDebugLogLevelOff
 */
extern const std::string COMMON_PARAM_KEY_DEBUG_LOG_LEVEL;

#pragma mark - Offline Engine Verify

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

/*
 * OFFLINE_PARAM_KEY_APP_NAME
 * Value explanation:   离线授权所需APPNAME (Linux)
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string OFFLINE_PARAM_KEY_APP_NAME;

/*
 * OFFLINE_PARAM_KEY_CUID
 * Value explanation:   离线授权所需CUID (Android)
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string OFFLINE_PARAM_KEY_CUID;

/*
 * OFFLINE_PARAM_KEY_JNI_CONTEXT
 * Value explanation:   离线授权所需JNI_CONTEXT (Android)
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string OFFLINE_PARAM_KEY_JNI_CONTEXT;


#pragma mark - Offline Engine Wakeup

/*
 * WP_PARAM_KEY_WAKEUP_WORDS
 * Value explanation:   唤醒词列表
 * Value type:          std::vector
 * Value type:          std::string (json format list of words, UTF-8 encoded, list key "words")
 * Default value:       -
 * Example:             std::string("{\"words\":[\"小度你好\",\"百度你好\"]}")
 */
extern const std::string WP_PARAM_KEY_WAKEUP_WORDS;

/*
 * WP_PARAM_KEY_WAKEUP_WORDS_FILE_PATH
 * Value explanation:   唤醒词文件路径
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string WP_PARAM_KEY_WAKEUP_WORDS_FILE_PATH;

/*
 * WP_PARAM_KEY_WAKEUP_DAT_FILE_PATH
 * Value explanation:   唤醒引擎模型文件路径
 * Value type:          std::string
 * Default value:       -
 */
extern const std::string WP_PARAM_KEY_WAKEUP_DAT_FILE_PATH;

extern const std::string BDS_PARAM_KEY_WRITABLE_LIBRARY_DATA_PATH;

}

#endif /* bds_WakeupParameters_hpp */
