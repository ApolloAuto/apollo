import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import { apollo } from '@dreamview/dreamview';
import { isEqual, isNil } from 'lodash';
import { DataSubjectType, WebSocketPlugin } from './type';
import { RequestMessageActionEnum } from '../type';
import { type WebSocketManager } from '../websocket-manager.service';
import MapElementsManager from './MapElementsManager';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type IMap = apollo.hdmap.IMap;

export default class MapMessageHandlerPlugin implements WebSocketPlugin {
    dataName = StreamDataNames.Map;

    inflowDataName = StreamDataNames.SIM_WORLD;

    outflowDataName = StreamDataNames.Map;

    mapElementsManager: MapElementsManager = new MapElementsManager();

    // 接受sim_world数据中mapElementIds
    handleInflow<T>(data: T, dataSubject: DataSubjectType, webSocketManager: WebSocketManager): void {
        const infolwData = data as ISimulationWorld;
        const mapElementIds = infolwData?.mapElementIds;

        const [mapElements, mapIdsWithoutCache] = this.mapElementsManager.getMapElement(mapElementIds);
        // mapIdsWithoutCache如果不为{}
        if (!isNil(mapIdsWithoutCache) && !isEqual(mapIdsWithoutCache, {})) {
            webSocketManager.sendSubscriptionMessage(
                RequestMessageActionEnum.SUBSCRIBE_MESSAGE_TYPE,
                StreamDataNames.Map,
                null,
                {
                    param: { mapElementIds: mapIdsWithoutCache },
                    dataFrequencyMs: 0,
                },
            );
        }

        if (mapElements) {
            const outFlowSubject = dataSubject.getByExactKey({ name: this.outflowDataName });
            outFlowSubject?.next({ data: mapElements });
        }
    }

    handleSubscribeData<T = IMap, R = IMap>(data: T): R {
        const map = data as IMap;
        // 不存在header的情况下，为内部懒加载使用缓存数据，直接返回map
        if (isNil(map?.header)) {
            return map as unknown as R;
        }

        // 存在header的情况下，为外部请求数据，更新缓存数据
        if ((data as IMap).header) {
            this.mapElementsManager.updateMapElement(data as IMap);
        }

        return undefined;
    }
}
