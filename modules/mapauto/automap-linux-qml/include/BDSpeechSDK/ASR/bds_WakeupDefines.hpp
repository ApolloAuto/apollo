//
//  bds_WakeupDefines.hpp
//  BDASRCore
//
//  Created by baidu on 16/5/20.
//  Copyright © 2016年 baidu. All rights reserved.
//

#ifndef bds_WakeupDefines_hpp
#define bds_WakeupDefines_hpp

namespace bds {

#pragma mark - 唤醒引擎状态
typedef enum TWakeupEngineWorkStatus
{
    EWakeupEngineWorkStatusStarted,     // 引擎开始工作
    EWakeupEngineWorkStatusStopped,     // 引擎关闭完成
    EWakeupEngineWorkStatusLoaded,      // 唤醒引擎加载完成
    EWakeupEngineWorkStatusUnLoaded,    // 唤醒引擎卸载完成
    EWakeupEngineWorkStatusTriggered,   // 命中唤醒词
    EWakeupEngineWorkStatusError,       // 引擎发生错误
} TWakeupEngineWorkStatus;

#pragma mark - 唤醒引擎错误分类
typedef enum TWakeupEngineErrorDomain
{
    EWakeupEngineErrorDomainRecord          = 10,   // 录音设备出错
    EWakeupEngineErrorDomainEngine          = 38,   // 录音设备出错
} TWakeupEngineErrorDomain;

#pragma mark - 唤醒引擎错误状态
typedef enum TWakeupEngineErrorCode
{
    EWakeupEngineRecoderException       = (EWakeupEngineErrorDomainRecord << 16) | (0x0000FFFF & 1),    // 录音设备异常
    EWakeupEngineRecoderNoPermission    = (EWakeupEngineErrorDomainRecord << 16) | (0x0000FFFF & 2),    // 无录音权限
    EWakeupEngineRecoderUnAvailable     = (EWakeupEngineErrorDomainRecord << 16) | (0x0000FFFF & 3),    // 录音设备不可用
    EWakeupEngineRecoderInterruption    = (EWakeupEngineErrorDomainRecord << 16) | (0x0000FFFF & 4),    // 录音中断
    
    EWakeupEngineExceptioin             = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 1),    // 唤醒引擎异常
    EWakeupEngineNoLicense              = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 2),    // 无授权文件
    EWakeupEngineLicenseInvalid         = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 3),    // 授权文件异常
    EWakeupEngineWakeupWordsInvalid     = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 4),    // 唤醒次异常
    EWakeupEngineDatFileInvalid         = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 5),    // 模型文件异常
    EWakeupEngineInitializeFailed       = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 6),    // 引擎初始化失败
    EWakeupEngineAllocMemFailed         = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 7),    // 内存分配失败
    EWakeupEngineResetFailed            = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 8),    // 引擎重置失败
    EWakeupEngineFreeFailed             = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 9),    // 引擎释放失败
    EWakeupEngineArchiNotSupportted     = (EWakeupEngineErrorDomainEngine << 16) | (0x0000FFFF & 10),   // 引擎不支持该架构
} TWakeupEngineErrorCode;

}

#endif /* bds_WakeupDefines_hpp */
