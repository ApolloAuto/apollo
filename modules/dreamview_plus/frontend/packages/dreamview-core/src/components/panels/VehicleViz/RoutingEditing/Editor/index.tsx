import React, { useCallback, useEffect, useState } from 'react';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { useNavigate } from '@dreamview/dreamview-core/src/util/SimpleRouter';
import ViewBtn from '@dreamview/dreamview-core/src/components/panels/VehicleViz/ViewBtn';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import MapElementsManager from '@dreamview/dreamview-core/src/services/WebSocketManager/plugins/MapElementsManager';
import { Vector3 } from 'three';
import { message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { useRoutingEditor } from '@dreamview/dreamview-core/src/hooks/useCarviz';
import { Subscription } from 'rxjs';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import useStyle from '../../useStyle';
import { ViewType } from '../../types';
import RoutingEditingFunctionalArea from '../RoutingEditingFunctionalArea';
import { useVizStore } from '../../VizStore';
import { initRoutingEditor } from '../../VizStore/actions';
import { MutexToolNameEnum } from '../RoutingEditingFunctionalArea/types';
import RoutingEditingOperationArea from '../RoutingEditingOperationArea';
import { PointType, RouteOrigin } from '../RouteManager/type';
import { DefaultPoint } from '../../../../../services/api/types';

export function Editor() {
    const { t } = useTranslation('routeEditing');

    const [{ routeManager }, dispatch] = useVizStore();

    const { currentRouteManager } = routeManager;

    const { registerFullScreenHooks } = usePanelContext();

    const { mainApi, isMainConnected } = useWebSocketServices();

    const [routingEditor, uid] = useRoutingEditor();

    const [mapElementsManager] = useState(() => new MapElementsManager());

    // 互斥的激活工具名称
    const [activeName, setActiveName] = useState<Nullable<MutexToolNameEnum>>(null);

    const changeActiveName = (name: MutexToolNameEnum) => {
        if (activeName === name) return;
        setActiveName(name);
    };

    const { classes } = useStyle();

    const [hmi] = usePickHmiStore();

    const navigate = useNavigate();

    useEffect(() => {
        // 注册退出全屏的钩子
        registerFullScreenHooks({
            afterExitFullScreen: () => {
                navigate('/');
            },
        });
    }, []);

    useEffect(() => {
        routingEditor.init();
        routingEditor.view.changeViewType(ViewType.MAP, false);
        dispatch(initRoutingEditor({ routingEditor }));
    }, []);

    const getMapElements = useCallback(
        (cameraCenter: Vector3) => {
            mainApi
                ?.getMapElementIds({ point: { x: cameraCenter.x ?? 0, y: cameraCenter.y ?? 0 }, radius: 200 })
                .then((res) => {
                    if (MapElementsManager.isMapElementIdsEmpty(res)) return;
                    const [mapElementsAlreadyRender, mapIdsWithoutCache] = mapElementsManager.getMapElement(res);

                    mainApi.getMapElementsByIds(mapIdsWithoutCache).then((mapElements) => {
                        mapElementsManager.updateMapElement(mapElements);
                        routingEditor?.updateMap(mapElementsManager.getAllMapElements());
                        routingEditor.render();
                    });
                });
        },
        [isMainConnected],
    );

    useEffect(() => {
        let subScribeCurrentRouteSubscription: Subscription;
        routingEditor.removeAll();
        routingEditor.deactiveAll();
        mapElementsManager.clear();

        const initFunc = (p: DefaultPoint, isInit = true) => {
            routingEditor.coordinates.initialize(p.x, p.y);
            if (isInit) routingEditor.initiationMarker.init(p);

            getMapElements(new Vector3(p.x, p.y, 0));

            subScribeCurrentRouteSubscription = routeManager.currentRouteManager.subScribeCurrentRoute(
                (currentRoute) => {
                    const { routeOrigin, routePoint } = currentRoute;

                    if (routeOrigin === RouteOrigin.CREATING_ROUTE) {
                        const routeInitialPoint = routePoint.routeInitialPoint;
                        const routeWayPoint = routePoint.routeWayPoint;

                        routingEditor.initiationMarker.init(routeInitialPoint);

                        routingEditor.pathwayMarker.init(routeWayPoint);
                    }
                },
            );
        };

        mainApi?.getMapElementIds({ radius: 20 }).then((res) => {
            const [mapElementsAlreadyRender, mapIdsWithoutCache] = mapElementsManager.getMapElement(res);

            mainApi.getMapElementsByIds(mapIdsWithoutCache).then((mapElements) => {
                mapElementsManager.updateMapElement(mapElements);
                // 找到第一个地图元素，将其坐标设置为offset
                const firstElement = MapElementsManager.findFirstMapElement(mapIdsWithoutCache);
                if (firstElement) {
                    try {
                        // 暂时使用主车位置定位
                        // const element = mapElementsManager.getMapElementById(firstElement);
                        // const point = MapElementsManager.getOffsetPosition(element);
                        // const offsetX = point.x || 0;
                        // const offsetY = point.y || 0;
                        routingEditor?.updateMap(mapElements);
                        routingEditor.render();
                        routingEditor.setCameraUpdateCallback(getMapElements);

                        mainApi
                            ?.getStartPoint()
                            .then((p) => {
                                initFunc(p);
                            })
                            .catch(() => {
                                // 获取起始点失败，使用虚拟位置
                                mainApi.getVirtualStartPoint().then((p) => {
                                    initFunc(p, false);
                                });
                            });
                    } catch (e) {
                        console.error('getMapElementById error', e);
                    }
                }
            });
        });

        return () => {
            subScribeCurrentRouteSubscription?.unsubscribe();
        };
    }, [isMainConnected, hmi.currentMap]);

    useEffect(() => {
        const unSubScribeCurrentCheckPoint = currentRouteManager.subScribeCurrentCheckPoint((currentCheckPoint) => {
            if (isMainConnected) {
                const checkPoint = currentCheckPoint;
                const checkPointType = currentCheckPoint.type;
                delete checkPoint.type;
                mainApi
                    .checkRoutingPoint({
                        point: {
                            ...checkPoint,
                            id: 0,
                        },
                    })
                    .then((res) => {
                        if (!res.isLegal) {
                            if (checkPointType === PointType.INITIAL_POINT) {
                                routingEditor.initiationMarker.undo();
                                message({ type: 'error', content: t('checkStartPointTooltip') });
                            }
                            if (checkPointType === PointType.WAY_POINT) {
                                routingEditor.pathwayMarker.undo();
                                message({ type: 'error', content: t('checkPointTooltip') });
                            }
                        } else if (checkPointType === PointType.INITIAL_POINT) {
                            mainApi.setStartPoint({
                                point: {
                                    ...checkPoint,
                                },
                            });
                        }
                    });
            }
        });
        return () => {
            unSubScribeCurrentCheckPoint.unsubscribe();
        };
    }, [isMainConnected]);

    return (
        <div className={classes['viz-container']}>
            <div id={uid} className={classes['web-gl']} />
            <RoutingEditingFunctionalArea
                carviz={routingEditor}
                activeName={activeName}
                changeActiveName={changeActiveName}
            />
            <div className={classes['view-btn-container']}>
                <ViewBtn carviz={routingEditor} activeName={activeName} changeActiveName={changeActiveName} />
            </div>
            <div className={classes['view-ope-container']}>
                <RoutingEditingOperationArea />
            </div>
        </div>
    );
}
