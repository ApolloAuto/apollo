import * as TYPES from './actionTypes';

export const menuStoreUtils = {
    // 插件安装成功
    isCertSuccess: (status: TYPES.ENUM_CERT_STATUS) => status === TYPES.ENUM_CERT_STATUS.SUCCESS,
    // 插件状态未知
    isCertUnknow: (status: TYPES.ENUM_CERT_STATUS) => status === TYPES.ENUM_CERT_STATUS.UNKNOW,
};
