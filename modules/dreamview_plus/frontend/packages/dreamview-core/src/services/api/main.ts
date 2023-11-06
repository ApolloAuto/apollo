import Logger from '@dreamview/log';
import { apollo } from '@dreamview/dreamview';
import { webSocketManager } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';
import {
    HMIActions,
    HMIDataPayload,
    MainApiTypes,
    DefaultRoutings,
    StartScenario,
    SaveDefaultRoutingInfo,
    GetStartPointInfo,
    SetStartPointInfo,
    CheckCycleRoutingInfo,
    IsCheckCycleRouting,
    CheckRoutingPointInfo,
    CheckRoutingPointLegal,
    SendRoutingRequestInfo,
    SendDefaultCycleRoutingRequestInfo,
    SendParkingRoutingRequestInfo,
    GetMapElementIdsInfo,
    GetMapElementsByIdsInfo,
    HMIActionsOperationInfo,
    StartCycle,
    HDMapSwitchInfo,
} from './types';
import { Metadata, SocketNameEnum } from '../WebSocketManager/type';

type IMapElementIds = apollo.dreamview.IMapElementIds;
type IMap = apollo.hdmap.IMap;

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

    // 开始录制数据包
    startRecordPackets() {
        return this.requestWithoutRes<undefined>({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.StartRecordPackets,
        });
    }

    // 停止录制
    stopRecordPackets() {
        return this.requestWithoutRes<undefined>({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.StopRecordPackets,
        });
    }

    // 保存录制内容
    saveRecordPackets(newName: string) {
        return this.request<{ newName: string }, any>({
            data: {
                name: '',
                info: {
                    newName,
                },
            },
            type: MainApiTypes.SaveRecordPackets,
        });
    }

    // 删除录制内容
    deleteRecordPackets() {
        return this.requestWithoutRes<undefined>({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.DeleteRecordPackets,
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

    // 和changeScenarios合并为一个接口
    // changeScenariosSet(scenariosSetId: string) {
    //     return this.requestWithoutRes<HMIDataPayload>({
    //         data: {
    //             name: '',
    //             info: {
    //                 action: HMIActions.ChangeScenariosSet,
    //                 value: scenariosSetId,
    //             },
    //         },
    //         type: MainApiTypes.HMIAction,
    //     });
    // }

    changeScenarios(scenariosId: string, scenariosSetId: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeScenarios,
                    value: `${scenariosSetId},${scenariosId}`,
                },
            },
            type: MainApiTypes.HMIAction,
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
        return this.request<HMIDataPayload, HDMapSwitchInfo>({
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
        return this.request<HMIDataPayload, HMIActionsOperationInfo>({
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

    deleteHDMap(record: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteHDMap,
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

    deleteScenarioSet(id: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteScenarios,
                    value: id,
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

    loadScenarios() {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.LoadScenarios,
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

    triggerPncMonitor(planning: boolean) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    planning,
                },
            },
            type: MainApiTypes.TriggerPncMonitor,
        });
    }

    getDefaultRoutings() {
        return this.request<'', DefaultRoutings>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.GetDefaultRoutings,
        });
    }

    startCycle(info: StartCycle) {
        return this.request<StartCycle, any>({
            data: {
                name: '',
                info,
            },
            type: MainApiTypes.SendDefaultCycleRoutingRequest,
        });
    }

    startScenario(info: StartScenario) {
        return this.request<StartScenario, any>({
            data: {
                name: '',
                info,
            },
            type: MainApiTypes.SendScenarioSimulationRequest,
        });
    }

    stopScenario() {
        return this.requestWithoutRes<any>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.StopScenarioSimulation,
        });
    }

    deleteDefaultRouting(name: string) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    name,
                },
            },
            type: MainApiTypes.DeleteDefaultRouting,
        });
    }

    saveDefaultRouting(info: SaveDefaultRoutingInfo) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.SaveDefaultRouting,
        });
    }

    getStartPoint() {
        return this.request<'', GetStartPointInfo>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.GetStartPoint,
        });
    }

    setStartPoint(info: SetStartPointInfo) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.SetStartPoint,
        });
    }

    setResetPoint() {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.SetStartPoint,
        });
    }

    checkCycleRouting(info: CheckCycleRoutingInfo) {
        return this.request<CheckCycleRoutingInfo, IsCheckCycleRouting>({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.CheckCycleRouting,
        });
    }

    checkRoutingPoint(info: CheckRoutingPointInfo) {
        return this.request<CheckRoutingPointInfo, CheckRoutingPointLegal>({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.CheckRoutingPoint,
        });
    }

    sendRoutingRequest(info: SendRoutingRequestInfo) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.SendRoutingRequest,
        });
    }

    resetSimControl() {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {},
            },
            type: MainApiTypes.ResetSimControl,
        });
    }

    sendDefaultCycleRoutingRequest(info: SendDefaultCycleRoutingRequestInfo) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.SendDefaultCycleRoutingRequest,
        });
    }

    sendParkingRoutingRequest(info: SendParkingRoutingRequestInfo) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.SendParkingRoutingRequest,
        });
    }

    getMapElementIds(info: GetMapElementIdsInfo) {
        return this.request<GetMapElementIdsInfo, IMapElementIds>({
            data: {
                name: '',
                info: {
                    ...info,
                },
            },
            type: MainApiTypes.GetMapElementIds,
        });
    }

    getMapElementsByIds(mapElementIds: IMapElementIds) {
        return this.request<GetMapElementsByIdsInfo, IMap>({
            data: {
                name: '',
                info: {
                    param: {
                        mapElementIds,
                    },
                },
            },
            type: MainApiTypes.GetMapElementsByIds,
        });
    }
}
