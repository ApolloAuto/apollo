/* eslint-disable camelcase */
import Logger from '@dreamview/log';
import { mergeMap, throwError, of } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { webSocketManager } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';
import { DownloadRecord, Files } from '../models/response-message.model';
import { SocketNameEnum } from '../WebSocketManager/type';
import {
    AccountInfo,
    DynamicModel,
    DynamicModelRecord,
    PluginApiNames,
    ScenarioSet,
    ScenarioSetRecord,
    V2xInfo,
    V2xInfoRecord,
    VehicleInfo,
    VehicleInfoRecord,
    HDMapSet,
} from './types';

const logger = Logger.getInstance(__filename);

export class PluginApi {
    get webSocketManager() {
        return webSocketManager;
    }

    generateRequest<Req>(req: ReqDataType<Req>): RequestDataType<Req> {
        return {
            ...req,
            data: {
                ...req.data,
                source: 'dreamview',
                target: 'studio_connector',
                sourceType: 'module',
                targetType: 'plugins',
            },
            type: 'PluginRequest',
        };
    }

    request<Req, Res>(req: ReqDataType<Req>) {
        if (!webSocketManager.isPluginConnected()) return Promise.reject();
        return webSocketManager
            .request<Req, Res>(this.generateRequest(req), SocketNameEnum.PLUGIN, req.data.requestId)
            .then((res) => {
                if (res.data.info.code !== 0) {
                    logger.error(
                        `Received error message from PluginRequest name: ${req.data.name}, message: ${JSON.stringify(
                            res.data.info,
                            null,
                            0,
                        )}`,
                    );
                }

                if (res.data.info.code === 0) {
                    return Promise.resolve(res.data.info.data);
                }

                return Promise.reject(res.data.info.message || res.data.info.error_msg);
            })
            .catch((err) => {
                logger.error(
                    `Received error message from PluginRequest name: ${req.data.name}, message: ${JSON.stringify(
                        err,
                        null,
                        0,
                    )}`,
                );
                return Promise.reject(err);
            });
    }

    /**
     * myService.sendRequest(myRequest).subscribe(
     *   data => {
     *     // 这是成功的情况，你可以处理你的数据
     *     console.log(data);
     *   },
     *   error => {
     *     // 这是错误的情况，你可以处理你的错误
     *     console.error(error);
     *   }
     * )
     */
    requestStream<Req, Res>(req: ReqDataType<Req>) {
        return webSocketManager
            .requestStream<Req, Res>(this.generateRequest(req), SocketNameEnum.PLUGIN, req.data.requestId)
            .pipe(
                mergeMap((res) => {
                    if (res.data.info.code !== 0) {
                        logger.error(
                            `Received error message from PluginRequest name: ${
                                req.data.name
                            }, message: ${JSON.stringify(res.data.info, null, 0)}`,
                        );
                        return throwError(new Error(res.data.info.message));
                    }

                    return of(res.data.info.data);
                }),
                catchError((error) => {
                    // Handle the error here
                    console.error(error);
                    return throwError(error);
                }),
            );
    }

    // 检查插件状态，若没有插件则不展示profile manager入口
    checkCertStatus() {
        return this.request<'', null>({
            data: {
                info: '',
                name: PluginApiNames.CheckCertStatus,
            },
        });
    }

    // 下载record
    downloadRecord(data: string, requestId: string) {
        return this.requestStream<string, DownloadRecord>({
            data: {
                info: data,
                name: PluginApiNames.DownloadRecord,
                requestId,
            },
        });
    }

    // 更新record下载进度
    refreshDownloadRecord(data: string, requestId: string) {
        return this.requestStream<string, DownloadRecord>({
            data: {
                info: data,
                name: PluginApiNames.DownloadRecord,
                requestId,
            },
        });
    }

    // 获取record列表
    getRecordsList() {
        return this.request<'', Files>({
            data: {
                info: '',
                name: PluginApiNames.GetRecordsList,
            },
        });
    }

    getAuthorizationInfo() {
        return this.request<any, any>({
            data: {
                info: '',
                name: 'FetchAuthorizationInfo',
            },
        });
    }

    checkInHwDevice(deviceId: string) {
        return this.request<any, any>({
            data: {
                info: deviceId,
                name: 'CheckInHwDevice',
            },
        });
    }

    getSubscribeList(options: { useCache?: boolean } = {}) {
        const { useCache = false } = options;
        return this.request<any, any>({
            data: {
                info: useCache ? '1' : '',
                name: 'GetSubscriberList',
            },
        });
    }

    updateLiscence() {
        return this.request<any, any>({
            data: {
                info: '',
                name: 'CheckCertificateStatus',
            },
        });
    }

    getSubscribeAccountInfo() {
        return this.request<any, any>({
            data: {
                info: '',
                name: 'GetSubscriberInfo',
            },
        });
    }

    getCloudDeviceList() {
        return this.request<any, any>({
            data: {
                info: JSON.stringify({ page_number: 1, page_size: 100 }),
                name: 'GetSubscriberDevicesList',
            },
        });
    }

    changeSubscribe(id: string) {
        return this.request<any, any>({
            data: {
                info: id,
                name: 'GetSubscriberToken',
            },
        });
    }

    // 获取用户信息
    getAccountInfo() {
        return this.request<'', AccountInfo>({
            data: {
                info: '',
                name: PluginApiNames.GetAccountInfo,
            },
        });
    }

    getVehicleInfo() {
        return this.request<'', VehicleInfoRecord>({
            data: {
                info: '',
                name: PluginApiNames.GetVehicleInfo,
            },
        });
    }

    //
    // 重置为初始值
    resetVehicleConfig(vehicleId: string) {
        return this.request<string, VehicleInfo>({
            data: {
                info: vehicleId,
                name: PluginApiNames.ResetVehicleConfig,
            },
        });
    }

    // 更新云端数据至本地
    refreshVehicleConfig(vehicleId: string) {
        return this.request<string, VehicleInfo>({
            data: {
                info: vehicleId,
                name: PluginApiNames.RefreshVehicleConfig,
            },
        });
    }

    uploadVehicleConfig(vehicleId: string) {
        return this.request<string, VehicleInfo>({
            data: {
                info: vehicleId,
                name: PluginApiNames.UploadVehicleConfig,
            },
        });
    }

    getV2xInfo() {
        return this.request<'', V2xInfoRecord>({
            data: {
                info: '',
                name: PluginApiNames.GetV2xInfo,
            },
        });
    }

    // 更新云端数据至本地
    refreshV2xConf(obuId: string) {
        return this.request<string, V2xInfoRecord>({
            data: {
                info: obuId,
                name: PluginApiNames.RefreshV2xConf,
            },
        });
    }

    // 缺少重置V2X接口
    uploadV2xConf(obuId: string) {
        return this.request<string, V2xInfo>({
            data: {
                info: obuId,
                name: PluginApiNames.UploadV2xConf,
            },
        });
    }

    resetV2xConfig(obuId: string) {
        return this.request<string, V2xInfo>({
            data: {
                info: obuId,
                name: PluginApiNames.ResetV2xConfig,
            },
        });
    }

    getDynamicModelList() {
        return this.request<'', DynamicModelRecord>({
            data: {
                info: '',
                name: PluginApiNames.GetDynamicModelList,
            },
        });
    }

    // 动力学模型列表（一期无）
    downloadDynamicModel(name: string) {
        return this.requestStream<string, DynamicModel>({
            data: {
                info: name,
                name: PluginApiNames.DownloadDynamicModel,
            },
        });
    }

    getScenarioSetList() {
        return this.request<'', ScenarioSetRecord>({
            data: {
                info: '',
                name: PluginApiNames.GetScenarioSetList,
            },
        });
    }

    // 场景集的更新和下载是同一个接口
    downloadScenarioSet(scenarioSetId: string, is_classic: boolean, requestId: string) {
        return this.requestStream<string, Partial<ScenarioSet>>({
            data: {
                info: JSON.stringify({ scenarioSetId, is_classic }),
                name: PluginApiNames.DownloadScenarioSet,
                requestId,
            },
        });
    }

    // 下载record
    downloadHDMap(data: string, requestId: string) {
        return this.requestStream<string, Partial<HDMapSet>>({
            data: {
                info: data,
                name: PluginApiNames.DownloadHDMap,
                requestId,
            },
        });
    }

    // 更新record下载进度
    refreshDownloadHDMap(data: string, requestId: string) {
        return this.requestStream<string, Partial<HDMapSet>>({
            data: {
                info: data,
                name: PluginApiNames.DownloadHDMap,
                requestId,
            },
        });
    }

    // 获取HDMap列表
    getHDMapList() {
        return this.request<'', Files>({
            data: {
                info: '',
                name: PluginApiNames.GetMapList,
            },
        });
    }
}
