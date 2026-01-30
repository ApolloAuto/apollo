import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { Button, IconPark, message, useImagePrak } from '@dreamview/dreamview-ui';
import EventEmitter from 'eventemitter3';
import { useTranslation } from 'react-i18next';
import { RoutingEditor } from '@dreamview/dreamview-carviz';
import showModal from '../../../../../../util/modal';
import RoutingEditingFunctionalCreateModal from './RoutingEditingFunctionalCreateModal';
import CustomScroll from '../../../../../CustomScroll';
import useWebSocketServices from '../../../../../../services/hooks/useWebSocketServices';
import { DefaultRouting } from '../../../../../../services/api/types';
import { CurrentRoute, RouteOrigin } from '../../RouteManager/type';
import { MainApi } from '../../../../../../services/api';
import { useVizStore } from '../../../VizStore';
import useStyle from './useStyle';
import { RouteManager } from '../../RouteManager';
import { CommonRoutingOrigin } from '../types';
import { useNavigate } from '../../../../../../util/SimpleRouter';
import { usePanelContext, IPanelContext } from '../../../../base/store/PanelStore';
import { useEventEmitter } from '../../../../../../store/EventHandlersStore';
import { BusinessEventTypes } from '../../../../../../store/EventHandlersStore/eventType';

interface RoutingEditingFunctionalFavoriteProps {
    activeOrigin: CommonRoutingOrigin;
    destroyFunFavoriteNotFullScreen?: () => void;
}
interface RoutingEditingFunctionalCreateModalProps {
    EE: EventEmitter;
    mainApi: MainApi | null;
    isMainConnected: boolean;
    routeManager: RouteManager;
    routingEditor: RoutingEditor;
    panelContext: IPanelContext;
    defaultRoutingList: DefaultRouting[];
    createCommonRoutingSuccessFunctional: () => void;
}

function RoutingEditingFunctionalFavorite(props: RoutingEditingFunctionalFavoriteProps) {
    const { activeOrigin, destroyFunFavoriteNotFullScreen } = props;

    const { cx, classes } = useStyle();

    const SourceEmptyImg = useImagePrak('ic_default_page_no_data');

    const EE = useEventEmitter();

    const navigate = useNavigate();

    const panelContext = usePanelContext();

    const { enterFullScreen } = panelContext;

    const { t } = useTranslation('routeEditing');

    const [{ routeManager, routingEditor }] = useVizStore();

    const currentRouteManager = routeManager.currentRouteManager;

    const { mainApi, isMainConnected } = useWebSocketServices();

    const [activeRouting, setActiveRouting] = useState(null);

    const [routingList, setRoutingList] = useState([] as DefaultRouting[]);

    const compareRoutingPoint = (creatingList: DefaultRouting[], editingRouting: CurrentRoute) => {
        const editingRoutingMix = [
            editingRouting?.routePoint?.routeInitialPoint,
            ...(editingRouting?.routePoint?.routeWayPoint || []),
        ];
        return creatingList.find((item) => {
            if (item.point.length !== editingRoutingMix.length) {
                return false;
            }
            return item.point.every(
                (pointItem, pointIndex) =>
                    pointItem?.heading === editingRoutingMix[pointIndex]?.heading &&
                    pointItem.x === editingRoutingMix[pointIndex]?.x &&
                    pointItem.y === editingRoutingMix[pointIndex]?.y,
            );
        });
    };

    const handleCommonRouting = (clickRouting: DefaultRouting) => {
        setActiveRouting(clickRouting);
        const panelId = panelContext.panelId;
        const initialPoint = clickRouting.point[0];
        const wayPoint = clickRouting.point.slice(1);
        const cycleNumber = clickRouting?.cycleNumber;
        const routeInfo = { initialPoint, wayPoint, cycleNumber };
        routeManager.currentRouteMix.setCurrentRouteMix({
            currentRouteLoop: {
                currentRouteLoopState: !!cycleNumber,
                currentRouteLoopTimes: cycleNumber,
            },
        });
        mainApi.setStartPoint({ point: initialPoint }).then(() => {
            currentRouteManager.setCurrentRoute({
                routeOrigin: RouteOrigin.CREATING_ROUTE,
                routePoint: {
                    routeInitialPoint: initialPoint,
                    routeWayPoint: wayPoint,
                },
            });
            EE.emit(BusinessEventTypes.SimControlRoute, { panelId, routeInfo });
        });
    };

    const deleteCommonRouting = (deleteRoutingName: string) => (event: any) => {
        event.stopPropagation();
        if (isMainConnected) {
            mainApi.deleteDefaultRouting(deleteRoutingName).then(() => {
                mainApi.getDefaultRoutings().then((res) => {
                    setRoutingList(res.defaultRoutings.reverse());
                });
            });
        }
    };

    const createCommonRoutingSuccessFunctional = () => {
        if (isMainConnected) {
            mainApi.getDefaultRoutings().then((res) => {
                setRoutingList(res.defaultRoutings.reverse());
            });
        }
    };

    const createCommonRouting = useCallback(() => {
        if (activeOrigin === CommonRoutingOrigin.FROM_FULLSCREEN) {
            if (routingEditor.pathwayMarker.positionsCount && isMainConnected) {
                showModal<RoutingEditingFunctionalCreateModalProps>(
                    {
                        EE,
                        mainApi,
                        routeManager,
                        routingEditor,
                        panelContext,
                        isMainConnected,
                        defaultRoutingList: routingList,
                        createCommonRoutingSuccessFunctional,
                    },
                    RoutingEditingFunctionalCreateModal,
                );
            } else {
                message({ type: 'error', content: t('NoWayPointMessage') });
            }
        }
        if (activeOrigin === CommonRoutingOrigin.FROM_NOT_FULLSCREEN) {
            destroyFunFavoriteNotFullScreen();
            navigate('/routing');
            enterFullScreen();
        }
    }, [
        EE,
        mainApi,
        routingList,
        routeManager,
        routingEditor,
        panelContext,
        activeOrigin,
        routeManager,
        isMainConnected,
        navigate,
        enterFullScreen,
    ]);

    useEffect(() => {
        if (isMainConnected) {
            mainApi.getDefaultRoutings().then((res) => {
                setRoutingList(res.defaultRoutings.reverse());
            });
        }
    }, [isMainConnected]);

    useEffect(() => {
        setActiveRouting(compareRoutingPoint(routingList, currentRouteManager.getCurrentRoute()));
    }, [routingList]);

    useEffect(() => {
        if (activeOrigin !== CommonRoutingOrigin.FROM_NOT_FULLSCREEN && isMainConnected) {
            mainApi.getStartPoint().then((res) => {
                const startPoint = { x: res.x, y: res.y, heading: res?.heading };
                const endPoint = routingEditor.pathwayMarker.lastPosition;
                mainApi.checkCycleRouting({ start: startPoint, end: endPoint }).then((cycle) => {
                    if (!cycle.isCycle) {
                        routeManager.currentRouteMix.setCurrentRouteMix({
                            currentRouteLoop: { currentRouteLoopState: false },
                        });
                    }
                });
            });
        }
    }, [isMainConnected, activeOrigin]);

    return (
        <CustomScroll className={classes['favorite-scroll']}>
            {activeOrigin !== CommonRoutingOrigin.FROM_NOT_FULLSCREEN && (
                <Button className={classes['favorite-creating-op']} onClick={createCommonRouting}>
                    {t('routeCreateCommonBtn')}
                </Button>
            )}
            {routingList.length ? (
                <div className={classes['favorite-common-co']}>
                    {routingList.map((item) => (
                        <div
                            key={item.name}
                            onClick={() => handleCommonRouting(item)}
                            className={cx(
                                classes['favorite-common-item'],
                                activeRouting &&
                                    activeRouting.name === item.name &&
                                    classes['favorite-common-item-active'],
                            )}
                        >
                            <div
                                className={cx('favorite-common-item-name-cx', classes['favorite-common-item-name'])}
                                title={item.name}
                            >
                                {item.name}
                            </div>
                            {item.cycleNumber && (
                                <div className={cx('favorite-common-item-op-no-hover')}>
                                    <span
                                        className={cx(
                                            'favorite-common-item-op-no-hover-title-cx',
                                            classes['favorite-common-item-op-no-hover-title'],
                                        )}
                                    >
                                        {t('looptimes')}
                                    </span>
                                    <span
                                        className={cx(
                                            'favorite-common-item-op-no-hover-val-cx',
                                            classes['favorite-common-item-op-no-hover-val'],
                                        )}
                                    >
                                        {item.cycleNumber}
                                    </span>
                                </div>
                            )}
                            <div className={cx('favorite-common-item-op-hover')}>
                                <span
                                    onClick={deleteCommonRouting(item.name)}
                                    className={cx(
                                        'favorite-common-item-op-hover-remove-cx',
                                        classes['favorite-common-item-op-hover-remove'],
                                    )}
                                >
                                    <IconPark name='IcRemoveAllPoints' />
                                </span>
                            </div>
                        </div>
                    ))}
                </div>
            ) : (
                <div className={classes['favorite-warning-co']}>
                    <img src={SourceEmptyImg} alt='resource_empty' />
                    <div className={classes['favorite-warning-co-desc']}>
                        {`${t('createRouteToolTip')},`}
                        <span className={classes['favorite-warning-co-desc-active']} onClick={createCommonRouting}>
                            {t('goToCreate')}
                        </span>
                    </div>
                </div>
            )}
        </CustomScroll>
    );
}

export default React.memo(RoutingEditingFunctionalFavorite);
