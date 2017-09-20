//
//  bds_ASRDefines.hpp
//  SDKTester
//
//  Created by baidu on 16/2/23.
//  Copyright © 2016年 baidu. All rights reserved.
//

#ifndef bds_ASRDefines_h
#define bds_ASRDefines_h

#include <string>
//#include "bds_int_types.h"
#include <stdint.h>
namespace bds {
    
#pragma mark - 设定采样率
typedef enum TBDVoiceRecognitionRecordSampleRateFlags
{
    EVoiceRecognitionRecordSampleRateAuto = 0,
    EVoiceRecognitionRecordSampleRate8K,
    EVoiceRecognitionRecordSampleRate16K,
} TBDVoiceRecognitionRecordSampleRateFlags;

#pragma mark - 语音识别类型
typedef enum TBDVoiceRecognitionProperty
{
    EVoiceRecognitionPropertyMusic = 10001, // 音乐
    EVoiceRecognitionPropertyVideo = 10002, // 视频
    EVoiceRecognitionPropertyApp = 10003, // 应用
    EVoiceRecognitionPropertyWeb = 10004, // web
    EVoiceRecognitionPropertySearch = 10005, // 热词
    EVoiceRecognitionPropertyEShopping = 10006, // 电商&购物
    EVoiceRecognitionPropertyHealth = 10007, // 健康&母婴
    EVoiceRecognitionPropertyCall = 10008, // 打电话
    EVoiceRecognitionPropertySong = 10009, // 录歌识别
    EVoiceRecognitionPropertyShake = 10010, // 摇一摇拾台
    EVoiceRecognitionPropertyMedicalCare = 10052, // 医疗
    EVoiceRecognitionPropertyCar = 10053, // 汽车
    EVoiceRecognitionPropertyCatering = 10054, // 娱乐餐饮
    EVoiceRecognitionPropertyFinanceAndEconomics = 10055, // 财经
    EVoiceRecognitionPropertyGame = 10056, // 游戏
    EVoiceRecognitionPropertyCookbook = 10057, // 菜谱
    EVoiceRecognitionPropertyAssistant = 10058, // 助手
    EVoiceRecognitionPropertyRecharge = 10059, // 话费充值
    EVoiceRecognitionPropertyMap = 10060,  // 地图
    EVoiceRecognitionPropertyInput = 20000, // 输入
    // 离线垂类
    EVoiceRecognitionPropertyContacts = 100014, // 联系人指令
    EVoiceRecognitionPropertySetting = 100016, // 手机设置
    EVoiceRecognitionPropertyTVInstruction = 100018, // 电视指令
    EVoiceRecognitionPropertyPlayerInstruction = 100019, // 播放器指令
    EVoiceRecognitionPropertyRadio = 100020, // 收音机
} TBDVoiceRecognitionProperty;

#pragma mark - 设置识别语言
typedef enum TBDVoiceRecognitionLanguage
{
    EVoiceRecognitionLanguageChinese = 0,
    EVoiceRecognitionLanguageCantonese,
    EVoiceRecognitionLanguageEnglish,
    EVoiceRecognitionLanguageSichuanDialect,
} TBDVoiceRecognitionLanguage;

#pragma mark - 语音识别请求资源类型
typedef enum TBDVoiceRecognitionProtocol
{
    EPROTOCOL_DEFAULT = 0,
    EPROTOCOL_SEARCH_NBEST = 1,
    EPROTOCOL_INPUT_NBEST_PROTOCOL = 2,
    EPROTOCOL_POST_PROTOCOL = 101,
    EPROTOCOL_WISE_PROTOCOL = 300,
    EPROTOCOL_WISE_TEXT_PROTOCOL = 301,
    EPROTOCOL_AUDIO_DA_PROTOCOL = 302,
    EPROTOCOL_NLU_PROTOCOL = 303,
    EPROTOCOL_NLU_TEXT_PROTOCOL = 304,
    EPROTOCOL_WISE_NLU_PROTOCOL = 305,
    EPROTOCOL_TALK_PROTOCOL = 306,
    EPROTOCOL_SEARCH_MUSIC_PROTOCOL = 1000,
} TBDVoiceRecognitionProtocol;

#pragma mark - 调试日志级别
typedef enum TBDVoiceRecognitionDebugLogLevel
{
    EVRDebugLogLevelOff = 0,
    EVRDebugLogLevelFatal = 1,
    EVRDebugLogLevelError = 2,
    EVRDebugLogLevelWarning = 3,
    EVRDebugLogLevelInformation = 4,
    EVRDebugLogLevelDebug = 5,
    EVRDebugLogLevelTrace = 6
} TBDVoiceRecognitionDebugLogLevel;

#pragma mark - 语音识别状态
typedef enum TBDVoiceRecognitionClientWorkStatus
{
    EVoiceRecognitionClientWorkStatusStartWorkIng,           // 识别工作开始，开始采集及处理数据
    EVoiceRecognitionClientWorkStatusStart,                  // 检测到用户开始说话
    EVoiceRecognitionClientWorkStatusEnd,                    // 本地声音采集结束结束，等待识别结果返回并结束录音
    EVoiceRecognitionClientWorkStatusNewRecordData,          // 录音数据回调
    EVoiceRecognitionClientWorkStatusFlushData,              // 连续上屏
    EVoiceRecognitionClientWorkStatusFinish,                 // 语音识别功能完成，服务器返回正确结果
    EVoiceRecognitionClientWorkStatusMeterLevel,             // 当前音量回调
    EVoiceRecognitionClientWorkStatusCancel,                 // 用户取消
    EVoiceRecognitionClientWorkStatusError,                  // 发生错误
    /* 离线引擎状态 */
    EVoiceRecognitionClientWorkStatusLoaded,                 // 离线引擎加载完成
    EVoiceRecognitionClientWorkStatusUnLoaded,               // 离线引擎卸载完成
    /* CHUNK状态 */
    EVoiceRecognitionClientWorkStatusChunkThirdData,         // CHUNK: 识别结果中的第三方数据，如语义理解
    EVoiceRecognitionClientWorkStatusChunkEnd                // CHUNK: 识别过程结束
} TBDVoiceRecognitionClientWorkStatus;

#pragma mark - 语音识别策略
typedef enum TBDVoiceRecognitionStrategy
{
    EVR_STRATEGY_ONLINE = 0,        // 在线识别
    EVR_STRATEGY_OFFLINE,           // 离线识别
    EVR_STRATEGY_ONLINE_PRI,        // 在线优先
    EVR_STRATEGY_OFFLINE_PRI,       // 离线优先
    EVR_STRATEGY_BOTH,              // 并行模式
} TBDVoiceRecognitionStrategy;

#pragma mark - 语音识别离线引擎类型
typedef enum TBDVoiceRecognitionOfflineEngineType
{
    EVR_OFFLINE_ENGINE_INPUT = 0,              // 离线引擎输入法模式
    EVR_OFFLINE_ENGINE_NAVI,                   // 离线引擎导航模式
    EVR_OFFLINE_ENGINE_GRAMMER,                // 离线引擎语法模式
} TBDVoiceRecognitionOfflineEngineType;

#pragma mark - 识别结果类型
typedef enum TBDVoiceRecognitionResultType
{
    EVR_RESULT_TYPE_ERROR = -1,                 // Error code
    EVR_RESULT_TYPE_EMPTY = 0,                  // Empty reply
    EVR_RESULT_TYPE_PARTIAL = 1,                // Partial result
    EVR_RESULT_TYPE_NBEST = 2,                  // Nbest result
    EVR_RESULT_TYPE_CN = 3,                     // CN result
    EVR_RESULT_TYPE_RESOURCE = 4,               // NLU & resource result
    EVR_RESULT_TYPE_THIRD = 5,                  // CHUNK: Third party data
    EVR_RESULT_TYPE_FINISH = 6,                 // CHUNK: Finish
    EVR_RESULT_TYPE_END = 7,                    // CHUNK: End
} TBDVoiceRecognitionResultType;

#pragma mark - 网络类型
typedef enum TBDVoiceRecognitionNetworkType
{
    EVR_NETWORK_TYPE_NO = 0,
    EVR_NETWORK_TYPE_2G,
    EVR_NETWORK_TYPE_3G,
    EVR_NETWORK_TYPE_4G,
    EVR_NETWORK_TYPE_WIFI,
} TBDVoiceRecognitionNetworkType;
    
#pragma mark - 语音压缩类型
typedef enum TBDVoiceRecognitionAudioCompressionType
{
    EVR_AUDIO_COMPRESSION_MIN = 0,
    EVR_AUDIO_COMPRESSION_PCM = 1,
    EVR_AUDIO_COMPRESSION_BV32 = 2,
    EVR_AUDIO_COMPRESSION_AMR = 3,
    EVR_AUDIO_COMPRESSION_MAX = 4,
} TBDVoiceRecognitionAudioCompressionType;

#pragma mark - 语音识别错误通知状态分类
typedef enum TVoiceRecognitionClientErrorDomain
{
    EVRClientErrorDomainRecord          = 10,   // 录音设备出错
    EVRClientErrorDomainVAD             = 20,   // 语音数据处理过程出错
    EVRClientErrorDomainOnline          = 30,   // 在线识别引擎出错
    EVRClientErrorDomainLocalNetwork    = 31,   // 本地网络联接出错
    EVRClientErrorDomainHTTP            = 32,   // HTTP协议错误
    EVRClientErrorDomainServer          = 33,   // 服务器返回错误
    EVRClientErrorDomainOffline         = 34,   // 离线引擎返回错误
    EVRClientErrorDomainCommom          = 40,   // 其他错误
} TVoiceRecognitionClientErrorDomain;

#pragma mark - 语音识别错误通知状态
typedef enum TVoiceRecognitionClientErrorCode
{
    EVRClientErrorCodeRecoderException = (EVRClientErrorDomainRecord << 16) | (0x0000FFFF & 1),                // 录音设备异常
    EVRClientErrorCodeRecoderNoPermission = (EVRClientErrorDomainRecord << 16) | (0x0000FFFF & 2),             // 无录音权限
    EVRClientErrorCodeRecoderUnAvailable = (EVRClientErrorDomainRecord << 16) | (0x0000FFFF & 3),              // 录音设备不可用
    EVRClientErrorCodeInterruption = (EVRClientErrorDomainRecord << 16) | (0x0000FFFF & 4),                    // 录音中断
    
    EVRClientErrorCodeVADException = (EVRClientErrorDomainVAD << 16) | (0x0000FFFF & 1),                       // 前端库异常
    EVRClientErrorCodeNoSpeech = (EVRClientErrorDomainVAD << 16) | (0x0000FFFF & 2),                           // 用户未说话
    EVRClientErrorCodeShort = (EVRClientErrorDomainVAD << 16) | (0x0000FFFF & 3),                              // 用户说话声音太短
    
	EVRClientErrorCodeOnlineExceptioin = (EVRClientErrorDomainOnline << 16) | (0x0000FFFF & 1),                // 在线识别引擎异常
    EVRClientErrorCodeOnlineNetworkUnavailable = (EVRClientErrorDomainOnline << 16) | (0x0000FFFF & 2),        // 网络不可用
    EVRClientErrorCodeOnlineTokenFailed = (EVRClientErrorDomainOnline << 16) | (0x0000FFFF & 3),               // 获取token失败
	EVRClientErrorCodeOnlineResolveUrlFailed = (EVRClientErrorDomainOnline << 16) | (0x0000FFFF & 4),          // 解析url失败
    EVRClientErrorCodeLocalTimeout = (EVRClientErrorDomainLocalNetwork << 16) | (0x0000FFFF & 1),              // 请求超时
    
    EVRClientErrorCodeServerParamError = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3001),            // 协议参数错误
    EVRClientErrorCodeServerRecognError = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3002),           // 识别过程出错
    EVRClientErrorCodeServerNoFindResult = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3003),          // 没有找到匹配结果
    EVRClientErrorCodeServerAppNameUnknownError = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3004),   // AppnameUnkown错误
    EVRClientErrorCodeServerSpeechQualityProblem = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3005),  // 声音不符合识别要求
    EVRClientErrorCodeServerSpeechTooLong = (EVRClientErrorDomainServer << 16) | (0x0000FFFF & -3006),         // 语音过长
	
    EVRClientErrorCodeCommonBusy = (EVRClientErrorDomainCommom << 16) | (0x0000FFFF & 1),                      // 识别器忙
    EVRClientErrorCodeCommonPropertyListInvalid = (EVRClientErrorDomainCommom << 16) | (0x0000FFFF & 2),       // 垂类设置有误
    EVRClientErrorCodeCommonEnqueueError = (EVRClientErrorDomainCommom << 16) | (0x0000FFFF & 3)               // 语音数据enqueue失败
} TVoiceRecognitionClientErrorCode;

#pragma mark -- INTERNAL DEFINES


#pragma mark - 语音识别模式
typedef enum TBDVoiceRecognitionMode {
    EVR_MODE_MULTI_SENTENCE = 0,
    EVR_MODE_SINGLE_SENTENCE,
    EVR_MODE_MUSIC,
    EVR_MODE_SHAKE,
} TBDVoiceRecognitionMode;

}

#endif /* bds_ASRDefines_h */
