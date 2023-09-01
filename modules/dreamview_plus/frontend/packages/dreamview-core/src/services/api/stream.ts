import { webSocketManager } from '../WebSocketManager';
import { ReqDataType, RequestDataType } from '../models/request-message.model';

export class StreamApi {
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

    subscribeToData<T>(name: string) {
        return webSocketManager.subscribeToData<T>(name);
    }

    subscribeToDataWithChannel<T>(name: string, channel: string) {
        return webSocketManager.subscribeToDataWithChannel<T>(name, channel);
    }

    subscribeToDataWithChannelFuzzy<T>(name: string) {
        return webSocketManager.subscribeToDataWithChannelFuzzy<T>(name);
    }
}
