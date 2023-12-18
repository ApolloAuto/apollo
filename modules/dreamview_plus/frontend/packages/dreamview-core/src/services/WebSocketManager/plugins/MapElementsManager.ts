import { apollo } from '@dreamview/dreamview';
import { isArray, isEqual, isNil } from 'lodash';
import MultiKeyMap from '@dreamview/dreamview-core/src/util/MultiKeyMap';

export type IMapElementIds = apollo.dreamview.IMapElementIds;
export type IMap = apollo.hdmap.IMap;

export type IMapHeader = apollo.hdmap.IHeader;
export type ICrosswalk = apollo.hdmap.ICrosswalk;
export type IJunction = apollo.hdmap.IJunction;
export type ILane = apollo.hdmap.ILane;
export type IStopSign = apollo.hdmap.IStopSign;
export type ISignal = apollo.hdmap.ISignal;
export type IYieldSign = apollo.hdmap.IYieldSign;
export type IOverlap = apollo.hdmap.IOverlap;
export type IClearArea = apollo.hdmap.IClearArea;
export type ISpeedBump = apollo.hdmap.ISpeedBump;
export type IRoad = apollo.hdmap.IRoad;
export type IParkingSpace = apollo.hdmap.IParkingSpace;
export type IPNCJunction = apollo.hdmap.IPNCJunction;
export type IRSU = apollo.hdmap.IRSU;

export type KeyOfElementIds = keyof IMapElementIds;

export type IMapElement =
    | ICrosswalk
    | IJunction
    | ILane
    | IStopSign
    | ISignal
    | IYieldSign
    | IOverlap
    | IClearArea
    | ISpeedBump
    | IRoad
    | IParkingSpace
    | IPNCJunction
    | IRSU;

const MAP_ELEMENTS_REQUEST_CACHE_TIMEOUT = 3000;

export default class MapElementsManager {
    private mapHeader: IMapHeader;

    private mapElementCache: MultiKeyMap<
        {
            type: KeyOfElementIds;
            id?: string;
        },
        IMapElement
    >;

    private mapRequestCache: MultiKeyMap<{ type: KeyOfElementIds; id: string }, number>;

    constructor() {
        this.mapElementCache = new MultiKeyMap();
        this.mapRequestCache = new MultiKeyMap();
    }

    updateMapElement(map: IMap) {
        if (!isEqual(this.mapHeader, map.header)) {
            this.mapHeader = map.header;
            this.clear();
        }

        Object.keys(map)
            .filter((category) => category !== 'header')
            .forEach((key) => {
                const mapElements = map[key as KeyOfElementIds];
                if (isArray(mapElements) && mapElements.length > 0) {
                    mapElements.forEach((element) => {
                        this.mapElementCache.set(
                            {
                                type: key as KeyOfElementIds,
                                id: element.id.id,
                            },
                            element,
                        );
                    });
                }
            });
    }

    getMapElement(mapElementIds: IMapElementIds): [any, IMapElementIds] {
        const mapElements: any = {};
        const mapIdsWithoutCache: IMapElementIds = {};
        const now = Date.now();
        Object.keys(mapElementIds).forEach((category) => {
            const ids = mapElementIds[category as KeyOfElementIds];
            if (isArray(ids) && ids.length > 0) {
                // 需要按照格式重新组装mapElements，并给出没有缓存的id
                mapElements[category as KeyOfElementIds] = ids
                    .map((id) => {
                        const mapElement = this.mapElementCache.getByExactKey({
                            type: category as KeyOfElementIds,
                            id,
                        });
                        if (!isNil(mapElement)) {
                            return mapElement;
                        }

                        const lastRequestTime = this.mapRequestCache.getByExactKey({
                            type: category as KeyOfElementIds,
                            id,
                        });

                        if (isNil(lastRequestTime) || now - lastRequestTime >= MAP_ELEMENTS_REQUEST_CACHE_TIMEOUT) {
                            if (!mapIdsWithoutCache[category as KeyOfElementIds]) {
                                mapIdsWithoutCache[category as KeyOfElementIds] = [];
                            }
                            mapIdsWithoutCache[category as KeyOfElementIds].push(id);
                            this.mapRequestCache.set({ type: category as KeyOfElementIds, id }, now);
                        }
                        return null;
                    })
                    .filter((mapElement: IMapElement | null) => mapElement !== null);
            }
        });
        return [mapElements, mapIdsWithoutCache];
    }

    getAllMapElements(): IMap {
        const map: IMap = { header: this.mapHeader };
        const allEntries = this.mapElementCache.getAllEntries();
        allEntries.forEach(([key, mapElements]) => {
            if (!isNil(mapElements)) {
                const category = key.type;
                if (!map[category]) {
                    map[category] = [];
                }
                map[category].push(mapElements as any);
            }
        });

        return map;
    }

    getMapElementById(mapElementId: { type: KeyOfElementIds; id: string }): IMapElement {
        return this.mapElementCache.getByExactKey(mapElementId);
    }

    clear() {
        this.mapElementCache.clear();
        this.mapRequestCache.clear();
    }

    // 校验地图ids是否全部没有值
    static isMapElementIdsEmpty(IMapElementIds: IMapElementIds): boolean {
        return Object.keys(IMapElementIds).every((key) => {
            const mapElementIds = IMapElementIds[key as KeyOfElementIds];
            return isArray(mapElementIds) && mapElementIds.length === 0;
        });
    }

    // 找到第一个地图元素
    static findFirstMapElement(IMapElementIds: IMapElementIds): { type: KeyOfElementIds; id: string } {
        const firstMapElement = Object.keys(IMapElementIds).find((key) => {
            const mapElementIds = IMapElementIds[key as KeyOfElementIds];
            return isArray(mapElementIds) && mapElementIds.length > 0;
        });
        if (isNil(firstMapElement)) {
            return null;
        }
        const firstMapElements = IMapElementIds[firstMapElement as KeyOfElementIds];
        return { type: firstMapElement as KeyOfElementIds, id: firstMapElements[0] };
    }

    // 获取任意一个元素中任意一个坐标点作为偏移位置
    // eslint-disable-next-line consistent-return
    static getOffsetPosition(mapElement: IMapElement) {
        if ('polygon' in mapElement) {
            const point = mapElement.polygon.point;
            if (isArray(point)) {
                return point[0];
            }
            return point;
        }
        if ('centralCurve' in mapElement) {
            const centralCurve = mapElement.centralCurve;
            const segment = centralCurve.segment;
            if (isArray(segment)) {
                return segment[0].startPosition;
            }
        }

        if ('stopLine' in mapElement) {
            const stopLine = mapElement.stopLine;
            if (isArray(stopLine)) {
                return stopLine[0]?.segment[0]?.startPosition;
            }
        }

        if ('position' in mapElement) {
            if (isArray(mapElement.position)) {
                return mapElement.position[0]?.segment[0]?.startPosition;
            }
        }

        return { x: 0, y: 0, z: 0 };
    }
}
