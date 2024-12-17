import Logger from '@dreamview/log';
import { mergeMap, throwError, of, catchError } from 'rxjs';
import { apollo } from '@dreamview/dreamview';
import { webSocketManager, Metadata, SocketNameEnum } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';
import {
    HMIActions,
    HMIDataPayload,
    MainApiTypes,
    DefaultRoutings,
    StartScenario,
    CheckMapCollectInfo,
    CreateMapFileInfo,
    ExportMapFileInfo,
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
    OBJECT_STORE_TYPE,
    IAddObjectStoreParams,
    IDeleteObjectStoreParams,
    IPutObjectStoreParams,
    IGetObjectStoreParams,
    IGetTuplesObjectStoreParams,
    RoutePathInfo,
} from './types';

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

    requestStream<Req, Res>(req: ReqDataType<Req>) {
        return webSocketManager
            .requestStream<Req, Res>(this.generateRequest(req), SocketNameEnum.MAIN, req.data.requestId)
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

    startPlayRecorder() {
        return this.requestWithoutRes<undefined>({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.StartPlayRecorder,
        });
    }

    startPlayRTKRecorder() {
        return this.request<'', undefined>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.StartPlayRtkRecorder,
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

    startAutoDrive() {
        return this.requestWithoutRes<{ action: HMIActions.StartAutoDrive }>({
            data: {
                name: '',
                info: {
                    action: HMIActions.StartAutoDrive,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    getInitData() {
        return this.request<any, { currentMode: any; currentOperation: any }>({
            data: {
                name: '',
                info: {},
            },
            type: MainApiTypes.GetInitData,
        });
    }

    getPanelPluginInitData() {
        return this.request<any, any>({
            data: {
                name: '',
                info: {},
            },
            type: 'GetDvPluginPanelsJson',
        }).then((r) => JSON.parse(r || '[]'));
    }

    loadDynamic() {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    action: HMIActions.LOAD_DYNAMIC_MODELS,
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

    changeRTKRecord(record: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeRTKRecord,
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

    deleteDynamicModel(dynamicName: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.DeleteDynamic,
                    value: dynamicName,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    changeDynamicModel(dynamicName: string) {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    action: HMIActions.ChangeDynamic,
                    value: dynamicName,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    loadMaps() {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.LoadMaps,
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

    loadRecord(recordId: string) {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.LoadRecord,
                    value: recordId,
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

    loadRTKRecords() {
        return this.requestWithoutRes<HMIDataPayload>({
            data: {
                name: '',
                info: {
                    action: HMIActions.LoadRTKRecords,
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

    resetScenario() {
        return this.requestWithoutRes<any>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.ResetScenarioSimulation,
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

    addObjectStore<T>(type: OBJECT_STORE_TYPE, panelId: string, value: T) {
        return this.request<IAddObjectStoreParams, T>({
            data: {
                name: '',
                info: {
                    key: `${type}_${panelId}`,
                    value: JSON.stringify(value),
                },
            },
            type: MainApiTypes.AddObjectStore,
        });
    }

    /**
     * @description
     * 将对象存储到本地数据库中。
     * 如果对象已经存在，则会覆盖原有的对象。
     *
     * @param {object} prop - 包含两个属性的对象：
     * - `type` {string} - 对象类型，用于区分不同类型的对象；
     * - `value` {any} - 需要存储的对象值。
     *
     * @returns {Promise<T>} - Promise 对象，resolve 回调函数参数为传入的 `value`。
     *
     * @throws {Error} - 当 `type` 或 `value` 未提供时抛出错误。
     */
    putObjectStore<T>(prop: { type: OBJECT_STORE_TYPE; value: T }) {
        return this.request<IPutObjectStoreParams, T>({
            data: {
                name: '',
                info: {
                    key: prop.type,
                    value: JSON.stringify(prop.value),
                },
            },
            type: MainApiTypes.PutObjectStore,
        });
    }

    /**
     * @description
     * 将图表对象存储到数据库中，并返回一个 Promise。
     * 如果传入的 `prop` 参数中的 `type` 和 `panelId` 属性都不为空，则会在请求中携带这些信息。
     *
     * @param prop {object} 包含以下属性的对象：
     * - type {string} 对象存储类型，可选值为 'chart'、'dashboard'、'workbench'。默认值为 'chart'。
     * - panelId {string} 面板 ID，可选值为字符串。默认值为 ''。
     * - value {T} 需要存储的值，类型为 T。
     *
     * @returns {Promise<T>} 返回一个 Promise，resolve 时会返回传入的 `prop.value`，reject 时会返回错误信息。
     */
    putChartObjectStore<T>(prop: { type: OBJECT_STORE_TYPE; panelId: string; value: T }) {
        return this.request<IPutObjectStoreParams, T>({
            data: {
                name: '',
                info: {
                    key: `${prop.type}_${prop.panelId}`,
                    value: JSON.stringify(prop.value),
                },
            },
            type: MainApiTypes.PutObjectStore,
        });
    }

    deleteObjectStore<T>(prop: { type: OBJECT_STORE_TYPE; panelId: string }) {
        return this.request<IDeleteObjectStoreParams, any>({
            data: {
                name: '',
                info: {
                    key: `${prop.type}_${prop.panelId}`,
                },
            },
            type: MainApiTypes.DeleteObjectStore,
        });
    }

    /**
     * @description
     * 获取图表对象存储，返回 Promise，resolve 值为指定类型的数组或空数组。
     *
     * @param prop {object} - 包含以下属性：
     * - type {string} - 对象存储类型，可选值为 'chart'、'filter'、'linkage'。
     * - panelId {string} - 面板 ID。
     *
     * @returns {Promise<T>} - Promise，resolve 值为指定类型的数组或空数组。
     *
     * @throws {Error} - 如果请求失败，则 reject 一个 Error 对象。
     */
    getChartObjectStore<T>(prop: { type: OBJECT_STORE_TYPE; panelId: string }) {
        return this.request<IGetObjectStoreParams, string>({
            data: {
                name: '',
                info: {
                    key: `${prop.type}_${prop.panelId}`,
                },
            },
            type: MainApiTypes.GetObjectStore,
        }).then((r) => JSON.parse(r || '[]') as T);
    }

    /**
     * @description
     * 获取对象存储，返回 Promise<T>。其中 T 为指定类型的对象。
     *
     * @param {object} prop - 包含 type 属性的对象，表示对象存储的类型。
     * @param {string} prop.type - 必需，字符串类型，表示对象存储的类型。
     *
     * @returns {Promise<T>} - Promise 对象，resolve 时返回 T 类型的对象，reject 时抛出错误信息。
     */
    getObjectStore<T>(prop: { type: OBJECT_STORE_TYPE }) {
        return this.request<IGetObjectStoreParams, string>({
            data: {
                name: '',
                info: {
                    key: prop.type,
                },
            },
            type: MainApiTypes.GetObjectStore,
        }).then((r) => JSON.parse(r || '{}') as T);
    }

    /**
     * 获取当前布局信息
     *
     * @returns {Promise<any>} Promise对象，resolve返回一个JSON对象，reject返回错误信息
     */
    getCurrentLayout() {
        return this.request<any, any>({
            data: {
                name: '',
                info: {},
            },
            type: MainApiTypes.GetCurrentLayout,
        }).then((r) => JSON.parse(r || '{}'));
    }

    /**
     * @description 获取默认布局，返回一个 Promise，resolve 值为一个对象，包含 name、info 两个属性
     * @returns {Promise<Object>} resolve 的对象包含 name、info 两个属性，name 是字符串类型，info 是任意类型的对象
     */
    getDefaultLayout() {
        return this.request<any, any>({
            data: {
                name: '',
                info: {},
            },
            type: MainApiTypes.GetDefaultLayout,
        }).then((r) => JSON.parse(r || '{}'));
    }

    getTuplesObjectStore<T>(prop: { type: OBJECT_STORE_TYPE }) {
        return this.request<IGetTuplesObjectStoreParams, { key: string; value: string }[]>({
            data: {
                name: '',
                info: {
                    type: prop.type,
                },
            },
            type: MainApiTypes.GetTuplesObjectStore,
        }).then((r) => {
            const val = r || [];
            return val.map((item) => ({ key: item.key, value: JSON.parse(item.value) as T }));
        });
    }

    startTerminal() {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: undefined,
            },
            type: MainApiTypes.StartTerminal,
        });
    }

    stopAutoDrive() {
        return this.requestWithoutRes({
            data: {
                name: '',
                info: {
                    action: HMIActions.DISENGAGE,
                },
            },
            type: MainApiTypes.HMIAction,
        });
    }

    getRoutePath() {
        return this.request<'', RoutePathInfo>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.RequestRoutePath,
        });
    }

    checkMapCollect() {
        return this.request<'', CheckMapCollectInfo>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.CheckMapCollectStatus,
        });
    }

    startMapCollect() {
        return this.request<'', null>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.StartRecordMapData,
        });
    }

    endMapCollect() {
        return this.request<'', null>({
            data: {
                name: '',
                info: '',
            },
            type: MainApiTypes.StopRecordMapData,
        });
    }

    createMapFile(requestId: string) {
        return this.requestStream<'', CreateMapFileInfo>({
            data: {
                name: '',
                info: '',
                requestId,
            },
            type: MainApiTypes.StartMapCreator,
        });
    }

    breakMapfile(requestId: string) {
        return this.request<{ requestId: string }, null>({
            data: {
                name: '',
                info: {
                    requestId,
                },
            },
            type: MainApiTypes.BreakMapCreator,
        });
    }

    exportMapFile(info: { source_path: string }) {
        return this.request<{ source_path: string }, ExportMapFileInfo>({
            data: {
                name: '',
                info: {
                    source_path: info.source_path,
                },
            },
            type: MainApiTypes.ExportMapFile,
        });
    }
}
