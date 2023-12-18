import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import MultiKeyMap from '../../../util/MultiKeyMap';
import { Emptyable } from '../../../util/similarFunctions';
import CountedSubject from '../../../util/CountedSubject';
import { StreamMessageData } from '../type';
import { type WebSocketManager } from '../websocket-manager.service';

export type DataSubjectType = MultiKeyMap<
    { name: string; channel?: Emptyable<string> },
    CountedSubject<StreamMessageData<unknown> | unknown>
>;

export abstract class WebSocketPlugin {
    // 数据源名称
    abstract dataName: StreamDataNames;

    // 流入数据流的名称
    abstract inflowDataName?: StreamDataNames;

    // 流出数据流的名称
    abstract outflowDataName: StreamDataNames;

    abstract channelName?: string;

    abstract handleInflow?<T = any>(data: T, dataSubject: DataSubjectType, webSocketManager: WebSocketManager): void;

    abstract handleSubscribeData?<T = any, R = any>(data: T): R | void;

    abstract handleSubscribeChannelData?<T = any, R = any>(data: T, channel: string): R | void;

    abstract handleFuzzyChannelDataSubscription?<T = any, R = any>(data: T, channel: string): R | void;
    // todo: 请求响应和请求响应流插件机制实现
    // handleRequest?(request: RequestMessage<any>, socket: SocketNameEnum): RequestMessage<any>;
    // handleRequestStream?(request: RequestMessage<any>, socket: SocketNameEnum): Observable<ResponseMessage<any>>;
}
