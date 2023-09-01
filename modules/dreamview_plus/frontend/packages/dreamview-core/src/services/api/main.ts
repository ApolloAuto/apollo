import Logger from '@dreamview/log';
import { webSocketManager } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';
import { HMIActions, HMIDataPayload, MainApiTypes } from './types';
import { Metadata, SocketNameEnum } from '../WebSocketManager/type';

const logger = Logger.getInstance(__filename);

export class MainApi {
    get webSocketManager() {
        return webSocketManager;
    }

    generateRequest<Req>(req: ReqDataType<Req>): RequestDataType<Req> {
        return {
            ...req,
            data: {
                ...req.data,
                source: 'dreamview',
                target: 'dreamview',
                sourceType: 'module',
                targetType: 'dreamview',
            },
            type: req.type,
        };
    }

    request<Req, Res>(req: ReqDataType<Req>) {
        if (!webSocketManager.isMainConnected()) return Promise.reject();
        return webSocketManager
            .request<Req, Res>(this.generateRequest(req), SocketNameEnum.MAIN, req.data.requestId)
            .then((res) => {
                if (res.data.info.code !== 0) {
                    logger.error(
                        `Received error message from ${this.generateRequest(req).type}, message: ${JSON.stringify(
                            res.data.info,
                            null,
                            0,
                        )}`,
                    );
                }

                if (res.data.info.code === 0) {
                    return Promise.resolve(res.data.info.data);
                }

                return Promise.reject(res.data.info.message);
            })
            .catch((err) => {
                logger.error(
                    `Received error message from ${this.generateRequest(req).type}, message: ${JSON.stringify(
                        err,
                        null,
                        0,
                    )}`,
                );
                return Promise.reject(err);
            });
    }

    requestWithoutRes<Req>(req: ReqDataType<Req>) {
        if (!webSocketManager.isMainConnected()) return Promise.reject();
        return webSocketManager
            .request<Req, never>(this.generateRequest(req), SocketNameEnum.MAIN, 'noResponse')
            .then(() => Promise.resolve())
            .catch((err) => {
                logger.error(
                    `Received error message from ${this.generateRequest(req).type}, message: ${JSON.stringify(
                        err,
                        null,
                        0,
                    )}`,
                );
                return Promise.reject(err);
            });
    }

    startPlayRecorder() {
        return this.requestWithoutRes<undefined>({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.StartPlayRecorder,
        });
    }

    resetRecordProgress(info: { progress: number }) {
        return this.requestWithoutRes<{ progress: string }>({
            data: {
                name: '',
                info: {
                    progress: `${info.progress}`,
                },
            },
            type: MainApiTypes.ResetRecordProgress,
        });
    }

    stopRecord() {
        return this.requestWithoutRes<{ action: HMIActions.StopRecord }>({
            data: {
                name: '',
                info: {
                    action: HMIActions.StopRecord,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    ctrRecorderAction(actionType: 'pause' | 'continue', progress: number) {
        return this.request<{ actionType: string; progress: number }, undefined>({
            data: {
                name: '',
                info: {
                    actionType,
                    progress,
                },
            },
            type: MainApiTypes.PlayRecorderAction,
        });
    }

    changeSetupMode(mode: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeMode,
                    value: mode,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    changeMap(map: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeMap,
                    value: map,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    changeVehicle(vehicle: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeVehicle,
                    value: vehicle,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    changeOperation(operation: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeOperation,
                    value: operation,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    changeRecord(record: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeRecord,
                    value: record,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    deleteRecord(record: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteRecord,
                    value: record,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    deleteVehicleConfig(vehicleName: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteVehicle,
                    value: vehicleName,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    deleteV2XConfig(vehicleName: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteV2X,
                    value: vehicleName,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    loadRecords() {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.LoadRecords,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    startModule(module: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.StartModule,
                    value: module,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    stopModule(module: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.StopModule,
                    value: module,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    setupAllModule() {
        return this.requestWithoutRes<{ action: HMIActions.SetupMode }>({
            data: {
                name: '',
                info: {
                    action: HMIActions.SetupMode,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    resetAllModule() {
        return this.requestWithoutRes<{ action: HMIActions.ResetMode }>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ResetMode,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    dumpFrame() {
        return this.request<'', null>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.Dump,
        });
    }

    resetSimWorld() {
        return this.request<'', null>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.Reset,
        });
    }

    updateMetaData() {
        return this.request<'', Metadata>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.GetDataHandlerConf,
        });
    }
}
