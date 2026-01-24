import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { message } from '@dreamview/dreamview-ui';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopoverSpec';
import { RoutingEditor } from '@dreamview/dreamview-carviz';
import { Nullable } from '@dreamview/dreamview-core/src/util/similarFunctions';
import {
    CreatePathwayMarkerCallbackRes,
    FunctionalOperation,
    PointData,
} from '@dreamview/dreamview-carviz/src/render/type';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import RoutingEditingFunctionalItem from './RoutingEditingFunctionalItem';
import { CommonRoutingOrigin, FunctionalNameEnum, MutexToolNameEnum } from './types';
import { FunctionalItemNoActiveEnum } from './const';
import FunctionalItemNoActive from './RoutingEditingFunctionalItemNoActive';
import RoutingEditingFunctionalRelocate from './RoutingEditingFunctionalRelocate';
import RoutingEditingFunctionalWay from './RoutingEditingFunctionalWay';
import RoutingEditingFunctionalLoop from './RoutingEditingFunctionalLoop';
import RoutingEditingFunctionalFavorite from './RoutingEditingFunctionalFavorite';
import { useVizStore } from '../../VizStore';
import { PointType, RouteInfo, RouteOrigin } from '../RouteManager/type';
import useWebSocketServices from '../../../../../services/hooks/useWebSocketServices';
import { HMIModeOperation, usePickHmiStore } from '../../../../../store/HmiStore';

interface IRoutingEditingFunctionalAreaProps {
    carviz: RoutingEditor;
    changeActiveName?: (name: MutexToolNameEnum) => void;
    activeName?: Nullable<MutexToolNameEnum>;
}

function RoutingEditingFunctionalArea(props: IRoutingEditingFunctionalAreaProps) {
    const { carviz, changeActiveName, activeName } = props;

    const [hmi] = usePickHmiStore();

    const [{ routeManager, routingEditor }] = useVizStore();

    const { isMainConnected, mainApi } = useWebSocketServices();

    const { t } = useTranslation('routeEditing');

    const routeManagerMix = routeManager.currentRouteMix;

    const [checkedItem, setCheckedItem] = useState<Nullable<FunctionalNameEnum>>(null);

    const relocateDisState = hmi.currentOperation === HMIModeOperation.AUTO_DRIVE;

    const indoorLocationDisState = useMemo(() => hmi.modules.get('IndoorLocalization'), [hmi.modules]);
    // todo： 测试使用
    // const indoorLocationDisState = useMemo(() => true, [hmi.modules]);

    // @ts-ignore
    const { classes, cx } = useStyle({ relocateDisState });

    const indoorLocationTimestepRef = useRef<number>(0);

    // 检查定位状态
    const checkIndoorLocationState = () =>
        new Promise<void>((resolve, reject) => {
            mainApi
                .checkIndoorLocalizationInitPointStatus()
                .then(() => {
                    resolve();
                })
                .catch(() => {
                    reject();
                });
        });

    const isTimeDiffLimit30S = useCallback(() => {
        const currentTime = new Date().getTime();
        if (currentTime - indoorLocationTimestepRef.current > 30000) {
            return true;
        }
        return false;
    }, [indoorLocationTimestepRef]);

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

            if ((point as CreatePathwayMarkerCallbackRes)?.origin === FunctionalNameEnum.INDOOR_LOCALIZATION) {
                if (operation === 'edit') {
                    if (!isTimeDiffLimit30S()) {
                        // 当前存在定时器，不允许执行，撤销点操作
                        carviz.indoorLocalizationMarker.undo();
                        return;
                    }
                    indoorLocationTimestepRef.current = new Date().getTime();
                    // ts-ignore
                    const start = point?.lastPosition;
                    message({
                        type: 'loading',
                        content:
                            'The Initialization point has been sent and is waiting for the indoor localization module to complete initialization',
                        key: 'sendIndoorLocalizationInitPoint',
                        duration: 30,
                    });
                    mainApi
                        .sendIndoorLocalizationInitPoint(start)
                        .then(() => {
                            // 发送成功，开启检查定位定时器
                            const timer = setInterval(() => {
                                checkIndoorLocationState().then(
                                    () => {
                                        message.destory('sendIndoorLocalizationInitPoint');
                                        clearInterval(timer);
                                        carviz.indoorLocalizationMarker.reset();
                                        message({
                                            type: 'success',
                                            content: 'IndoorLocalization Success',
                                            key: 'checkIndoorLocalizationState',
                                        });
                                        mainApi?.getStartPoint().then((p) => {
                                            // 识别的起始位置信息
                                            carviz.initiationMarker.init(p);
                                        });
                                    },
                                    () => {
                                        const isTimeDiff = isTimeDiffLimit30S();
                                        if (isTimeDiff) {
                                            clearInterval(timer);
                                            carviz.indoorLocalizationMarker.reset();
                                            message({
                                                type: 'error',
                                                content: 'IndoorLocalization Fail',
                                                key: 'checkIndoorLocalizationState',
                                            });
                                        }
                                    },
                                );
                            }, 10000);
                        })
                        .catch(() => {
                            // 发送失败
                            message({
                                type: 'error',
                                content: 'send IndoorLocalization Init Point Fail',
                                key: 'sendIndoorLocalizationInitPoint',
                            });
                        });
                }

                return;
            }

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
        if (
            ![MutexToolNameEnum.RELOCATE, MutexToolNameEnum.WAYPOINT, MutexToolNameEnum.INDOOR_LOCALIZATION].includes(
                activeName,
            )
        ) {
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

            if (e.key === 'i') {
                if (checkedItem !== FunctionalNameEnum.INDOOR_LOCALIZATION) {
                    carviz?.deactiveAll();
                    setCheckedItem(FunctionalNameEnum.INDOOR_LOCALIZATION);
                    changeActiveName?.(MutexToolNameEnum.INDOOR_LOCALIZATION);
                    carviz?.indoorLocalizationMarker.active(routeChange);
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
                case FunctionalNameEnum.LOOP:
                    // eslint-disable-next-line no-case-declarations
                    const wayCount = routingEditor.pathwayMarker.positionsCount;
                    if (wayCount > 0) {
                        if (isMainConnected) {
                            mainApi.getStartPoint().then((res) => {
                                const startPoint = { x: res.x, y: res.y, heading: res?.heading };
                                const endPoint = routingEditor.pathwayMarker.lastPosition;
                                mainApi.checkCycleRouting({ start: startPoint, end: endPoint }).then((cycle) => {
                                    if (cycle.isCycle) {
                                        setCheckedItem(name);
                                        carviz?.deactiveAll();
                                    } else {
                                        const currentRouteMixValue = {
                                            currentRouteLoop: { currentRouteLoopState: false },
                                        };
                                        routeManagerMix.setCurrentRouteMix(currentRouteMixValue);
                                        message({
                                            type: 'error',
                                            content: t('NoLoopMessage'),
                                        });
                                    }
                                });
                            });
                        }
                    } else {
                        message({ type: 'error', content: t('NoWayPointMessage') });
                    }
                    break;
                case FunctionalNameEnum.FAVORITE:
                    setCheckedItem(name);
                    carviz?.deactiveAll();
                    break;
                case FunctionalNameEnum.INDOOR_LOCALIZATION:
                    setCheckedItem(name);
                    carviz?.deactiveAll();
                    changeActiveName?.(MutexToolNameEnum.INDOOR_LOCALIZATION);
                    carviz?.indoorLocalizationMarker.active(routeChange);
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

    const functionalIndoorLocation = useMemo(
        () => (
            <FunctionalItemNoActive
                functionalItemNoActiveText={
                    indoorLocationDisState
                        ? FunctionalItemNoActiveEnum.FunctionalIndoorLocationNoActive
                        : FunctionalItemNoActiveEnum.FunctionalIndoorLocationNoActiveDis
                }
            />
        ),
        [indoorLocationDisState],
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
                    {/* @ts-ignore */}
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
                {/* 室内定位点 */}
                <Popover
                    content={functionalIndoorLocation}
                    trigger='hover'
                    placement='right'
                    mouseLeaveDelay={0.5}
                    destroyTooltipOnHide
                    rootClassName={cx(
                        checkedItem === FunctionalNameEnum.INDOOR_LOCALIZATION
                            ? classes['custom-popover-functinal']
                            : classes['custom-popover-ordinary'],
                    )}
                >
                    <div>
                        <RoutingEditingFunctionalItem
                            functionalName={FunctionalNameEnum.INDOOR_LOCALIZATION}
                            checkedItem={checkedItem}
                            onClick={handleClicked(FunctionalNameEnum.INDOOR_LOCALIZATION)}
                            disable={!indoorLocationDisState}
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
