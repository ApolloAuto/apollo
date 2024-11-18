import React, { useCallback, useEffect, useState } from 'react';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopoverSpec';
import { RoutingEditor } from '@dreamview/dreamview-carviz';
import { Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import {
    CreatePathwayMarkerCallbackRes,
    FunctionalOperation,
    PointData,
} from '@dreamview/dreamview-carviz/src/render/type';
import { message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import RoutingEditingFunctionalItem from './RoutingEditingFunctionalItem';
import { FunctionalNameEnum, MutexToolNameEnum, CommonRoutingOrigin } from './types';
import { FunctionalItemNoActiveEnum } from './const';
import FunctionalItemNoActive from './RoutingEditingFunctionalItemNoActive';
import RoutingEditingFunctionalRelocate from './RoutingEditingFunctionalRelocate';
import RoutingEditingFunctionalWay from './RoutingEditingFunctionalWay';
import RoutingEditingFunctionalLoop from './RoutingEditingFunctionalLoop';
import RoutingEditingFunctionalFavorite from './RoutingEditingFunctionalFavorite';
import { useVizStore } from '../../VizStore';
import { RouteInfo, RouteOrigin, PointType } from '../RouteManager/type';
import useWebSocketServices from '../../../../../services/hooks/useWebSocketServices';
import { usePickHmiStore, HMIModeOperation } from '../../../../../store/HmiStore';

interface IRoutingEditingFunctionalAreaProps {
    carviz: RoutingEditor;
    changeActiveName?: (name: MutexToolNameEnum) => void;
    activeName?: Nullable<MutexToolNameEnum>;
}

function RoutingEditingFunctionalArea(props: IRoutingEditingFunctionalAreaProps) {
    const { carviz, changeActiveName, activeName } = props;

    const [hmi, dispatch] = usePickHmiStore();

    const [{ routeManager, routingEditor }] = useVizStore();

    const { isMainConnected, mainApi } = useWebSocketServices();

    const { t } = useTranslation('routeEditing');

    const routeManagerMix = routeManager.currentRouteMix;

    const [checkedItem, setCheckedItem] = useState<Nullable<FunctionalNameEnum>>(null);

    const relocateDisState = hmi.currentOperation === HMIModeOperation.AUTO_DRIVE;

    const { classes, cx } = useStyle({ relocateDisState });

    const routeChange = useCallback(
        (operation: FunctionalOperation, point: PointData | CreatePathwayMarkerCallbackRes) => {
            const route: RouteInfo = {
                routeOrigin: RouteOrigin.EDITING_ROUTE,
                routePoint: {
                    routeInitialPoint: carviz.initiationMarker.initiationMarkerPosition,
                    routeWayPoint: carviz.pathwayMarker.pathWatMarkerPosition,
                },
            };
            if (!point) return;

            if ('lastPosition' in point) {
                // 当前逻辑用于途经点
                const lastPoint = point.lastPosition;
                if (operation === 'edit') {
                    routeManager.currentRouteManager?.setScribeCurrentCheckPoint({
                        ...lastPoint,
                        type: PointType.WAY_POINT,
                    });
                }
                routeManager.currentRouteManager?.setCurrentRoute(route);
            } else {
                // 当前逻辑用于起始点
                if (operation === 'edit') {
                    routeManager.currentRouteManager?.setScribeCurrentCheckPoint({
                        ...point,
                        type: PointType.INITIAL_POINT,
                    });
                }
                routeManager.currentRouteManager?.setCurrentRoute(route);
            }
        },
        [carviz.initiationMarker, carviz.pathwayMarker, routeManager.currentRouteManager],
    );

    useEffect(() => {
        if (!Object.values(MutexToolNameEnum).includes(activeName)) {
            // 非互斥工具，不做修改
            return;
        }
        if (![MutexToolNameEnum.RELOCATE, MutexToolNameEnum.WAYPOINT].includes(activeName)) {
            setCheckedItem(null);
        }
    }, [activeName, carviz]);

    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'r') {
                if (checkedItem !== FunctionalNameEnum.RELOCATE) {
                    carviz?.deactiveAll();
                    setCheckedItem(FunctionalNameEnum.RELOCATE);
                    changeActiveName?.(MutexToolNameEnum.RELOCATE);
                    carviz?.initiationMarker.active(routeChange);
                }
            }

            if (e.key === 'w') {
                if (checkedItem !== FunctionalNameEnum.WAYPOINT) {
                    carviz?.deactiveAll();
                    setCheckedItem(FunctionalNameEnum.WAYPOINT);
                    changeActiveName?.(MutexToolNameEnum.WAYPOINT);
                    carviz?.pathwayMarker.active(routeChange);
                }
            }

            if ((e.ctrlKey || e.metaKey) && e.key === 'z') {
                if (checkedItem === FunctionalNameEnum.RELOCATE) {
                    carviz?.initiationMarker.undo();
                }
                if (checkedItem === FunctionalNameEnum.WAYPOINT) {
                    carviz?.pathwayMarker.undo();
                }
            }
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [checkedItem]);

    const handleClicked = useCallback(
        (name: FunctionalNameEnum) => () => {
            if (name === FunctionalNameEnum.RELOCATE && relocateDisState) {
                return;
            }
            if (checkedItem === name) {
                if (Object.values(MutexToolNameEnum).includes(name as any)) {
                    carviz?.deactiveAll();
                }
                setCheckedItem(null);
                return;
            }

            switch (name) {
                case FunctionalNameEnum.RELOCATE:
                    setCheckedItem(name);
                    carviz?.deactiveAll();
                    changeActiveName?.(MutexToolNameEnum.RELOCATE);
                    carviz?.initiationMarker.active(routeChange);
                    break;
                case FunctionalNameEnum.WAYPOINT:
                    setCheckedItem(name);
                    carviz?.deactiveAll();
                    changeActiveName?.(MutexToolNameEnum.WAYPOINT);
                    carviz?.pathwayMarker.active(routeChange);
                    break;
                // case FunctionalNameEnum.LOOP:
                //     // eslint-disable-next-line no-case-declarations
                //     const wayCount = routingEditor.pathwayMarker.positionsCount;
                //     if (wayCount > 0) {
                //         if (isMainConnected) {
                //             mainApi.getStartPoint().then((res) => {
                //                 const startPoint = { x: res.x, y: res.y, heading: res?.heading };
                //                 const endPoint = routingEditor.pathwayMarker.lastPosition;
                //                 mainApi.checkCycleRouting({ start: startPoint, end: endPoint }).then((cycle) => {
                //                     if (cycle.isCycle) {
                //                         setCheckedItem(name);
                //                         carviz?.deactiveAll();
                //                     } else {
                //                         const currentRouteMixValue = {
                //                             currentRouteLoop: { currentRouteLoopState: false },
                //                         };
                //                         routeManagerMix.setCurrentRouteMix(currentRouteMixValue);
                //                         message({
                //                             type: 'error',
                //                             content: t('NoLoopMessage'),
                //                         });
                //                     }
                //                 });
                //             });
                //         }
                //     } else {
                //         message({ type: 'error', content: t('NoWayPointMessage') });
                //     }
                //     break;
                case FunctionalNameEnum.LOOP:
                    // eslint-disable-next-line no-case-declarations
                    const wayCount = routingEditor.pathwayMarker.positionsCount;
                    if (wayCount > 0) {
                        setCheckedItem(name);
                        carviz?.deactiveAll();
                    } else {
                        message({ type: 'error', content: t('NoWayPointMessage') });
                    }
                    break;
                case FunctionalNameEnum.FAVORITE:
                    setCheckedItem(name);
                    carviz?.deactiveAll();
                    break;
                default:
                    break;
            }
        },
        [carviz, changeActiveName, checkedItem],
    );

    const functionalRelocate =
        checkedItem === FunctionalNameEnum.RELOCATE ? (
            <RoutingEditingFunctionalRelocate />
        ) : (
            <FunctionalItemNoActive
                functionalItemNoActiveText={
                    relocateDisState
                        ? FunctionalItemNoActiveEnum.FunctionalRelocateNoActiveDis
                        : FunctionalItemNoActiveEnum.FunctionalRelocateNoActive
                }
            />
        );
    const functionalWay =
        checkedItem === FunctionalNameEnum.WAYPOINT ? (
            <RoutingEditingFunctionalWay />
        ) : (
            <FunctionalItemNoActive functionalItemNoActiveText={FunctionalItemNoActiveEnum.FunctionaWayNoActive} />
        );

    const functionalLoop =
        checkedItem === FunctionalNameEnum.LOOP ? (
            <RoutingEditingFunctionalLoop />
        ) : (
            <FunctionalItemNoActive functionalItemNoActiveText={FunctionalItemNoActiveEnum.FunctionalLoopNoActive} />
        );

    const functionalFavorite =
        checkedItem === FunctionalNameEnum.FAVORITE ? (
            <RoutingEditingFunctionalFavorite activeOrigin={CommonRoutingOrigin.FROM_FULLSCREEN} />
        ) : (
            <FunctionalItemNoActive
                functionalItemNoActiveText={FunctionalItemNoActiveEnum.FunctionalFavoriteNoActive}
            />
        );

    return (
        <div className={classes['routing-editing-function-area']}>
            <div className={classes['routing-editing-function-area__group']}>
                {/* 重定位 */}
                <Popover
                    content={functionalRelocate}
                    trigger='hover'
                    placement='right'
                    mouseLeaveDelay={0.5}
                    destroyTooltipOnHide
                    rootClassName={cx(
                        checkedItem === FunctionalNameEnum.RELOCATE
                            ? classes['custom-popover-functinal']
                            : classes['custom-popover-ordinary'],
                    )}
                >
                    <div className={cx({ [classes['func-relocate-ele']]: relocateDisState })}>
                        <RoutingEditingFunctionalItem
                            functionalName={FunctionalNameEnum.RELOCATE}
                            checkedItem={checkedItem}
                            onClick={handleClicked(FunctionalNameEnum.RELOCATE)}
                            disable={relocateDisState}
                        />
                    </div>
                </Popover>
                {/* 途经点 */}
                <Popover
                    content={functionalWay}
                    trigger='hover'
                    placement='right'
                    mouseLeaveDelay={0.5}
                    destroyTooltipOnHide
                    rootClassName={cx(
                        checkedItem === FunctionalNameEnum.WAYPOINT
                            ? classes['custom-popover-functinal']
                            : classes['custom-popover-ordinary'],
                    )}
                >
                    <div>
                        <RoutingEditingFunctionalItem
                            functionalName={FunctionalNameEnum.WAYPOINT}
                            checkedItem={checkedItem}
                            onClick={handleClicked(FunctionalNameEnum.WAYPOINT)}
                        />
                    </div>
                </Popover>
                {/* 循环路由 */}
                <Popover
                    content={functionalLoop}
                    trigger='hover'
                    placement='right'
                    mouseLeaveDelay={0.5}
                    destroyTooltipOnHide
                    rootClassName={cx(
                        checkedItem === FunctionalNameEnum.LOOP
                            ? classes['custom-popover-functinal']
                            : classes['custom-popover-ordinary'],
                    )}
                >
                    <div>
                        <RoutingEditingFunctionalItem
                            functionalName={FunctionalNameEnum.LOOP}
                            checkedItem={checkedItem}
                            onClick={handleClicked(FunctionalNameEnum.LOOP)}
                        />
                    </div>
                </Popover>
            </div>
            {/* 收藏路径 */}
            <Popover
                content={functionalFavorite}
                trigger='hover'
                placement='rightTop'
                destroyTooltipOnHide
                rootClassName={cx(
                    checkedItem === FunctionalNameEnum.FAVORITE
                        ? classes['custom-popover-functinal']
                        : classes['custom-popover-ordinary'],
                )}
            >
                <div>
                    <RoutingEditingFunctionalItem
                        functionalName={FunctionalNameEnum.FAVORITE}
                        checkedItem={checkedItem}
                        onClick={handleClicked(FunctionalNameEnum.FAVORITE)}
                    />
                </div>
            </Popover>
        </div>
    );
}

export default React.memo(RoutingEditingFunctionalArea);
