import { RequestMessage } from '../WebSocketManager/type';
import { MainApiTypes, PluginApiTypes, StreamApiTypes } from '../api/types';

export type RequestDataType<T> = Omit<RequestMessage<T>, 'data' | 'action'> & {
    data: Omit<RequestMessage<T>['data'], 'requestId'>;
};

export type ReqDataType<T> = {
    data: Omit<RequestMessage<T>['data'], 'requestId' | 'sourceType' | 'source' | 'targetType' | 'target'> & {
        requestId?: string;
    };
    // 主通讯单独配置， 插件通讯统一使用PluginRequest
    type?: MainApiTypes | PluginApiTypes | StreamApiTypes;
};
