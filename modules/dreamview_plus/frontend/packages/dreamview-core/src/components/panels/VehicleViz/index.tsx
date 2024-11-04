import React, { useEffect, useState, useMemo, useRef, useCallback, useContext } from 'react';
import { Popover, IconPark } from '@dreamview/dreamview-ui';
import type { apollo } from '@dreamview/dreamview';
import { SimpleRouter, KeepAlive, Route, RouterContext } from '@dreamview/dreamview-core/src/util/SimpleRouter';
import { usePickHmiStore, HMIModeOperation } from '@dreamview/dreamview-core/src/store/HmiStore';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import { useTranslation } from 'react-i18next';
import { Subscription } from 'rxjs';
import throttle from 'lodash/throttle';
import useCarViz from '@dreamview/dreamview-core/src/hooks/useCarviz';
import { useLocalStorage, useLocalStorageState, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';
import useStyle from './useStyle';
import LayerMenu from './LayerMenu';
import ViewMenu from './ViewMenu';
import ViewBtn from './ViewBtn';
import { usePanelContext } from '../base/store/PanelStore';
import { formatLayerParams, getCurrentLayerParams, localLayerMenuParamsManager } from './LayerMenu/params';
import Panel from '../base/Panel';
import './index.less';
import { RoutingEditingOpArea } from './RoutingEditing';
import { Editor } from './RoutingEditing/Editor';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import { VizProvider, useVizStore } from './VizStore';
import { initRouteManager } from './VizStore/actions';
import { RouteManager } from './RoutingEditing/RouteManager';
import { RouteOrigin, RoutePoint } from './RoutingEditing/RouteManager/type';
import { VizCurMode } from './types';
import CountedSubject from '../../../util/CountedSubject';
import { FunctionalKey } from '../../../store/EventHandlersStore';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type IMap = apollo.hdmap.IMap;
type IPointCloud = apollo.dreamview.IPointCloud;

function Viz() {
    const { classes } = useStyle();
    const [hmi] = usePickHmiStore();
    const routingTimeRef = useRef(-1);
    const panelContext = usePanelContext();
    const routerContext = useContext(RouterContext);
    const [carviz, uid] = useCarViz();
    const adcPositionRef = useRef([0, 0]);
    const [currentView, setCurrentView] = useLocalStorageState(KEY_MANAGER.CurrentSwitchView, 'Default');
    const { mainApi, streamApi, isMainConnected } = useWebSocketServices();
    const { logger, panelId, subscribeToData, updateChannel, setKeyDownHandlers, removeKeyDownHandlers } = panelContext;
    const { t } = useTranslation('panels');
    const [pointCloudVisible, setPointCloudVisible] = useState(
        getCurrentLayerParams().Perception.pointCloud.currentVisible,
    );
    const [curChannel, setCurChannel] = useState(null);
    const [boudingBoxVisible, setBoudingBoxVisible] = useState(
        getCurrentLayerParams().Map.boudingBox?.currentVisible ?? false,
    );
    const [boundaryLineVisible, setBoundaryLineVisible] = useState(
        getCurrentLayerParams().Planning.planningBoundaryLine?.currentVisible ?? false,
    );
    const [referenceLineVisible, setReferenceLineVisible] = useState(
        getCurrentLayerParams().Planning.planningReferenceLine?.currentVisible ?? false,
    );
    const [trajectoryLineVisible, setTrajectoryLineVisible] = useState(
        getCurrentLayerParams().Planning.planningTrajectoryLine?.currentVisible ?? false,
    );

    const getVizCurMode = useMemo(() => {
        if (hmi.currentOperation === HMIModeOperation.WAYPOINT_FOLLOW) {
            return VizCurMode.FOLLOW;
        }
        return VizCurMode.DEFAULT;
    }, [hmi.currentOperation]);

    const [vizCurMode, setVizCurMode] = useState(getVizCurMode);

    const animationFrameIdRef = useRef<number>();
    const subscriptionRef = useRef<{
        name: string;
        subscription: Subscription;
    }>(null);
    const pointCloudSubscriptionRef = useRef<Subscription>(null);

    const [fps, setFps] = useState<number>(0);
    const [triangles, setTriangles] = useState<number>(0);
    let frameCount = 0;
    let lastTime = performance.now();

    const [fpsTextClickCount, setFpsTextClickCount] = useState(0);
    const [isFpsTextVisible, setIsFpsTextVisible] = useState(true);
    const customToolbarTitleClick = () => {
        setFpsTextClickCount((prevCount) => {
            const newCount = prevCount + 1;
            if (newCount === 5) {
                setIsFpsTextVisible(!isFpsTextVisible);
                console.log(`change fps text visible : ${isFpsTextVisible}`);
                return 0;
            }
            return newCount;
        });
    };

    const rendFps = () => {
        if (!isFpsTextVisible) return;

        const currentTime = performance.now();
        // eslint-disable-next-line no-plusplus
        frameCount++;
        if (currentTime - lastTime >= 1000) {
            setFps(frameCount);
            frameCount = 0;
            lastTime = currentTime;
        }
        setTriangles(carviz?.renderer.info.render.triangles);
    };

    const render = () => {
        rendFps();
        carviz?.render();
        animationFrameIdRef.current = requestIdleCallback(
            () => {
                render();
            },
            {
                timeout: 1000,
            },
        );
    };

    const closeChannel = () => {
        carviz?.updateData({
            object: [],
            autoDrivingCar: {},
        });
        carviz?.render();

        if (pointCloudSubscriptionRef?.current) {
            pointCloudSubscriptionRef.current.unsubscribe();
        }

        if (subscriptionRef.current && subscriptionRef.current.subscription) {
            subscriptionRef.current.subscription.unsubscribe();
            subscriptionRef.current = null;
        }
    };

    const updatePointcloudChannel = (v: string) => {
        closeChannel();
        const newConnectedSubj: CountedSubject<unknown> = streamApi.subscribeToDataWithChannel(
            StreamDataNames.POINT_CLOUD,
            v,
        );
        const newSubscription = newConnectedSubj.subscribe((pointCloudData: IPointCloud) => {
            if (!pointCloudData) return;
            carviz?.updatePointCloud(pointCloudData);
        });

        subscriptionRef.current = {
            name: StreamDataNames.POINT_CLOUD,
            subscription: newSubscription,
        };
    };

    useEffect(() => {
        setVizCurMode(getVizCurMode);
    }, [getVizCurMode]);

    useEffect(() => {
        carviz.init();
        const curLayerMenuParams = getCurrentLayerParams();
        localLayerMenuParamsManager.set(curLayerMenuParams);
        carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'vehicle');
        // 如果勾选了点云按钮，则订阅点云通道
        if (curLayerMenuParams.Perception.pointCloud.currentVisible) {
            setTimeout(() => {
                updateChannel({
                    name: StreamDataNames.POINT_CLOUD,
                    needChannel: false,
                });
            }, 0);
        }

        // 监听键盘事件
        // document.addEventListener('keydown', (e: KeyboardEvent) => {
        //     if (((e.metaKey && isMac()) || (e.ctrlKey && isWin())) && e.code === 'Equal') {
        //         e.preventDefault();
        //         carviz.view?.updateViewDistance(-10);
        //     }
        //     if (((e.metaKey && isMac()) || (e.ctrlKey && isWin())) && e.code === 'Minus') {
        //         e.preventDefault();
        //         carviz.view?.updateViewDistance(10);
        //     }
        // });
    }, []);

    useEffect(() => {
        const handlers = [
            {
                keys: ['='],
                functionalKey: 'ctrlKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(-10);
                },
                discriptor: t('zoomIn'),
            },
            {
                keys: ['='],
                functionalKey: 'metaKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(-10);
                },
                discriptor: t('zoomIn'),
            },
            {
                keys: ['-'],
                functionalKey: 'ctrlKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(10);
                },
                discriptor: t('zoomOut'),
            },
            {
                keys: ['-'],
                functionalKey: 'metaKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(10);
                },
                discriptor: t('zoomOut'),
            },
        ];
        setKeyDownHandlers(handlers);

        return () => {
            removeKeyDownHandlers(handlers);
        };
    }, [t]);

    const filterSimData = (simData: ISimulationWorld) => {
        const simDataCopy = { ...simData, boudingBox: !!boudingBoxVisible };
        let filterBoundaryLineKey: string[] = null;
        let planningDataPath = simDataCopy?.planningData?.path || [];
        if (!Array.isArray(simDataCopy?.planningData?.path)) return simDataCopy;
        if (!boundaryLineVisible) {
            filterBoundaryLineKey = [
                'planning_path_boundary_1_regular/self',
                'candidate_path_regular/self',
                'planning_path_boundary_2_regular/self',
                'planning_path_boundary_1_fallback/self',
                'candidate_path_fallback/self',
                'planning_path_boundary_2_fallback/self',
            ];
            simDataCopy.planningData.path = planningDataPath.filter(
                (item) => !filterBoundaryLineKey.includes(item.name),
            );
        }
        planningDataPath = simDataCopy?.planningData?.path || [];
        if (!referenceLineVisible) {
            filterBoundaryLineKey = ['planning_reference_line'];
            simDataCopy.planningData.path = planningDataPath.filter(
                (item) => !filterBoundaryLineKey.includes(item.name),
            );
        }

        if (!trajectoryLineVisible && simDataCopy.planningTrajectory) {
            simDataCopy.planningTrajectory = [];
        }

        return simDataCopy;
    };

    useEffect(() => {
        if (isMainConnected) {
            if (routerContext.currentPath !== '/') return () => null;

            let mapConnectedSubscription: Subscription = null;
            let simWorldConnectedSubscription: Subscription = null;

            if (vizCurMode === VizCurMode.FOLLOW) {
                carviz.removeAll();
                carviz.view.setViewType('Overhead');
                const simWorldConnectedSubj = subscribeToData({ name: StreamDataNames.SIM_WORLD, needChannel: false });
                if (simWorldConnectedSubj) {
                    simWorldConnectedSubscription = simWorldConnectedSubj.subscribe((simData: ISimulationWorld) => {
                        if (!simData) return;
                        if (Object.keys(simData).length !== 0) {
                            const followData = {
                                autoDrivingCar: simData.autoDrivingCar,
                                followPlanningData: simData.planningTrajectory,
                            };
                            carviz.updateData(followData);
                        }
                    });
                }
            }

            if (vizCurMode === VizCurMode.DEFAULT) {
                carviz.follow.dispose();
                const throttleFunc = throttle(
                    (simData: ISimulationWorld) => {
                        mainApi.getRoutePath().then((res) => {
                            routingTimeRef.current = simData.routingTime;
                            if (Object.keys(simData).length !== 0) {
                                const simDataCopy = { ...simData };
                                simDataCopy.routePath = res.routePath;
                                carviz.updateData(filterSimData(simDataCopy));
                                carviz?.pointCloud.updateOffsetPosition();
                            }
                        });
                    },
                    500,
                    { leading: true },
                );
                const simWorldConnectedSubj = subscribeToData({ name: StreamDataNames.SIM_WORLD, needChannel: false });
                if (simWorldConnectedSubj) {
                    simWorldConnectedSubscription = simWorldConnectedSubj.subscribe((simData: ISimulationWorld) => {
                        if (!simData) return;
                        // @ts-ignore
                        const autoDrivingCar = simData.autoDrivingCar;

                        if (autoDrivingCar) {
                            const positionX = autoDrivingCar.positionX ?? 0;
                            const positionY = autoDrivingCar.positionY ?? 0;

                            // 曼哈顿距离
                            const distance =
                                Math.abs(adcPositionRef.current[0] - positionX) +
                                Math.abs(adcPositionRef.current[1] - positionY);
                            logger.debug(`车辆偏移距离：${distance}, 阈值为100`);
                            if (distance > 100 && adcPositionRef.current[0] !== 0 && adcPositionRef.current[1] !== 0) {
                                if (carviz && carviz.initialized) {
                                    logger.debug('车辆偏移距离超过阈值，重置场景');
                                    carviz.resetScence();
                                }
                            }
                            adcPositionRef.current = [positionX, positionY];
                        }

                        if (Object.keys(simData).length !== 0) {
                            if (simData.routingTime && simData.routingTime !== routingTimeRef.current) {
                                throttleFunc(simData);
                            } else {
                                carviz.updateData(filterSimData(simData));
                                carviz?.pointCloud.updateOffsetPosition();
                            }
                        }
                    });
                }
                const mapConnectedSubj = subscribeToData({ name: StreamDataNames.Map, needChannel: false });
                if (mapConnectedSubj) {
                    mapConnectedSubscription = mapConnectedSubj.subscribe((mapData: IMap) => {
                        if (!mapData) return;
                        carviz?.updateMap(mapData);
                    });
                }
            }

            return () => {
                if (vizCurMode === VizCurMode.FOLLOW) {
                    carviz.view.setViewType('Default');
                    if (simWorldConnectedSubscription) {
                        simWorldConnectedSubscription.unsubscribe();
                    }
                }
                if (vizCurMode === VizCurMode.DEFAULT) {
                    if (mapConnectedSubscription) {
                        mapConnectedSubscription.unsubscribe();
                    }
                    if (simWorldConnectedSubscription) {
                        simWorldConnectedSubscription.unsubscribe();
                    }
                }
            };
        }
    }, [
        vizCurMode,
        isMainConnected,
        referenceLineVisible,
        boundaryLineVisible,
        trajectoryLineVisible,
        boudingBoxVisible,
        routerContext.currentPath,
    ]);

    useEffect(() => {
        if (routerContext.currentPath === '/') {
            render();
        }

        return () => {
            const animationFrameId = animationFrameIdRef.current;
            if (animationFrameId) {
                cancelIdleCallback(animationFrameId);
            }
        };
    }, [routerContext.currentPath]);

    const { metadata } = useWebSocketServices();
    const curMeta = useMemo(
        () => metadata.find((meata) => meata.dataName === StreamDataNames.POINT_CLOUD),
        [metadata, isMainConnected],
    );
    const channels = useMemo(() => {
        if (!curMeta) {
            return [];
        }

        return curMeta.channels.map((channel) => ({
            label: channel?.channelName,
            value: channel?.channelName,
        }));
    }, [curMeta]);

    const pointCloudFusionChannel = useMemo(() => {
        // 优先使用channels字符串数组中含有'compensator'的channel，然后如果没有使用含有'fusion'的channel
        const fusionPointCloudChannel = metadata
            .find((item) => item.dataName === StreamDataNames.POINT_CLOUD)
            ?.channels?.filter(
                (item) => item?.channelName.includes('compensator') || item?.channelName.includes('fusion'),
            )
            .sort(
                // compensator排在前面，fusion排在后面
                (a) => (a?.channelName.includes('compensator') ? -1 : 1),
            );

        if (Array.isArray(fusionPointCloudChannel)) {
            return fusionPointCloudChannel[0];
        }
        return '';
    }, [metadata]);

    const localVizPointCloueChannelManager = useLocalStorage(`${panelId}-viz-pointcloud-channel`);
    useEffect(() => {
        let pointCloudConnectedSubj: any = null;
        if (isMainConnected) {
            const prevSelectedChannel = localVizPointCloueChannelManager.get();

            if (pointCloudVisible && prevSelectedChannel) {
                pointCloudConnectedSubj = subscribeToData({
                    name: StreamDataNames.POINT_CLOUD,
                    channel: prevSelectedChannel,
                    needChannel: true,
                });
                if (pointCloudConnectedSubj) {
                    pointCloudSubscriptionRef.current = pointCloudConnectedSubj.subscribe(
                        (pointCloudData: IPointCloud) => {
                            if (!pointCloudData) return;
                            carviz?.updatePointCloud(pointCloudData);
                        },
                    );
                    setCurChannel(prevSelectedChannel);
                }
            }
        }
        return () => {
            if (pointCloudSubscriptionRef.current) {
                pointCloudSubscriptionRef.current.unsubscribe();
            }
            carviz.pointCloud.disposeLastFrame();
        };
    }, [metadata, pointCloudVisible, isMainConnected]);

    useEffect(
        () => () => {
            subscriptionRef.current?.subscription?.unsubscribe();
        },
        [],
    );

    const layerMenu = (
        <LayerMenu
            carviz={carviz}
            pointCloudFusionChannel={pointCloudFusionChannel}
            handlePointCloudVisible={setPointCloudVisible}
            curChannel={curChannel}
            setCurChannel={setCurChannel}
            pointcloudChannels={channels}
            updatePointcloudChannel={updatePointcloudChannel}
            closeChannel={closeChannel}
            handleReferenceLineVisible={setReferenceLineVisible}
            handleBoundaryLineVisible={setBoundaryLineVisible}
            handleTrajectoryLineVisible={setTrajectoryLineVisible}
            handleBoudingBoxVisible={setBoudingBoxVisible}
        />
    );

    return (
        <div className={classes['viz-container']}>
            <div id={uid} className={classes['web-gl']} />
            <div className={classes['viz-rend-fps-item-hide']} onClick={customToolbarTitleClick}>
                {}
            </div>
            {!isFpsTextVisible && (
                <div className={classes['viz-rend-fps-item']}>
                    <header className='FPS-display'>
                        <p>
                            fps: {fps} &nbsp;
                            triangles: {triangles}
                        </p>
                    </header>
                </div>
            )}
            <div className={classes['viz-btn-container']}>
                <ViewBtn carviz={carviz}>
                    <Popover placement='leftTop' content={layerMenu} trigger='click'>
                        <span className={classes['viz-btn-item']}>
                            <IconPark name='IcCoverageHover' />
                        </span>
                    </Popover>
                    <Popover
                        overlayClassName={classes['layer-menu-popover']}
                        placement='leftBottom'
                        content={<ViewMenu carviz={carviz} setCurrentView={setCurrentView} />}
                        trigger='click'
                        style={{ padding: '0 !importent' }}
                    >
                        <span className={classes['viz-btn-item']}>{currentView?.charAt(0)}</span>
                    </Popover>
                </ViewBtn>
            </div>
            <RoutingEditingOpArea />
        </div>
    );
}

function IndexInit() {
    const [, dispatch] = useVizStore();

    const routeManager = {
        routeOrigin: RouteOrigin.EDITING_ROUTE,
        routePoint: {
            routeInitialPoint: null as RoutePoint,
            routeWayPoint: [] as RoutePoint[],
        },
    };

    const routeManagerMix = {
        currentRouteLoop: {
            currentRouteLoopState: false,
        },
    };

    useEffect(() => {
        dispatch(initRouteManager({ routeManager: new RouteManager(routeManager, routeManagerMix) }));
    }, []);

    return (
        <SimpleRouter initialPath='/'>
            <KeepAlive path='/' style={{ minWidth: '244px', height: '100%', position: 'relative' }}>
                <Viz />
            </KeepAlive>
            <Route path='/routing' style={{ width: '100%', height: '100%' }}>
                <Editor />
            </Route>
        </SimpleRouter>
    );
}

function Index() {
    return (
        <VizProvider>
            <IndexInit />
        </VizProvider>
    );
}

function VehicleViz(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: Index,
                panelId: props.panelId,
            }),
        [],
    );

    return <Component {...props} />;
}

Index.displayName = 'VehicleViz';

export default React.memo(VehicleViz);
