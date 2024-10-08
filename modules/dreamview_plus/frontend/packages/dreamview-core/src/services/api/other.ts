import Logger from '@dreamview/log';
import { mergeMap, throwError, of } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { webSocketManager, SocketNameEnum } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';
import { StartScenario, MainApiTypes, HMIDataPayload, OtherApiTypes, SimHMIAction } from './types';

const logger = Logger.getInstance(__filename);

export class OtherApi {
    get webSocketManager() {
        return webSocketManager;
    }

    getSocketIns(socketName: SocketNameEnum) {
        webSocketManager.initBySocketName(socketName);
        return webSocketManager.connectionManager.get(socketName);
    }

    deleteSocketIns(socketName: SocketNameEnum) {
        return webSocketManager.connectionManager.delete(socketName);
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

    async request<Req, Res>(req: ReqDataType<Req>, socketName: SocketNameEnum) {
        try {
            const res = await webSocketManager.request<Req, Res>(
                this.generateRequest(req),
                socketName,
                req.data.requestId,
            );
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
        } catch (err) {
            logger.error(
                `Received error message from PluginRequest name: ${req.data.name}, message: ${JSON.stringify(
                    err,
                    null,
                    0,
                )}`,
            );
            return Promise.reject(err);
        }
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
    requestStream<Req, Res>(req: ReqDataType<Req>, socketName: SocketNameEnum) {
        return webSocketManager.requestStream<Req, Res>(this.generateRequest(req), socketName, req.data.requestId).pipe(
            mergeMap((res) => {
                if (res.data.info.code !== 0) {
                    logger.error(
                        `Received error message from PluginRequest name: ${req.data.name}, message: ${JSON.stringify(
                            res.data.info,
                            null,
                            0,
                        )}`,
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

    async requestWithoutRes<Req>(req: ReqDataType<Req>, socketName: SocketNameEnum) {
        try {
            await webSocketManager.request<Req, never>(this.generateRequest(req), socketName, 'noResponse');
            return await Promise.resolve();
        } catch (err) {
            logger.error(
                `Received error message from ${this.generateRequest(req).type}, message: ${JSON.stringify(
                    err,
                    null,
                    0,
                )}`,
            );
            return Promise.reject(err);
        }
    }

    // 发送场景仿真请求
    startScenario(info: StartScenario) {
        return this.request<StartScenario, any>(
            {
                data: {
                    name: '',
                    info,
                },
                type: OtherApiTypes.SendScenarioSimulationRequest,
            },
            SocketNameEnum.SIMULATION,
        );
    }

    stopScenario() {
        return this.requestWithoutRes<any>(
            {
                data: {
                    name: '',
                    info: '',
                },
                type: OtherApiTypes.StopScenarioSimulation,
            },
            SocketNameEnum.SIMULATION,
        );
    }

    resetScenario() {
        return this.requestWithoutRes<any>(
            {
                data: {
                    name: '',
                    info: '',
                },
                type: OtherApiTypes.ResetScenarioSimulation,
            },
            SocketNameEnum.SIMULATION,
        );
    }

    loadScenarios() {
        return this.requestWithoutRes<HMIDataPayload>(
            {
                data: {
                    name: '',
                    info: {
                        action: SimHMIAction.LOAD_SCENARIOS,
                    },
                },
                type: MainApiTypes.SimHMIAction,
            },
            SocketNameEnum.SIMULATION,
        );
    }

    changeScenarios(scenariosId: string, scenariosSetId: string) {
        return this.request<HMIDataPayload, { currentScenarioMap: string }>(
            {
                data: {
                    name: '',
                    info: {
                        action: SimHMIAction.CHANGE_SCENARIO,
                        value: `${scenariosSetId},${scenariosId}`,
                    },
                },
                type: MainApiTypes.SimHMIAction,
            },
            SocketNameEnum.SIMULATION,
        );
    }
}
