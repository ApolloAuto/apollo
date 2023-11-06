import React, { useEffect, useState, useMemo, useRef } from 'react';
import { Carviz } from '@dreamview/dreamview-carviz/src';
import { Popover, IconIcCoverageHover } from '@dreamview/dreamview-ui';
import shortUUID from 'short-uuid';
import type { apollo } from '@dreamview/dreamview';
import { SimpleRouter, KeepAlive, Route } from '@dreamview/dreamview-core/src/util/SimpleRouter';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import useStyle from './useStyle';
import LayerMenu from './LayerMenu';
import ViewMenu from './ViewMenu';
import ViewBtn from './ViewBtn';
import { usePanelContext } from '../base/store/PanelStore';
import { formatLayerParams, getCurrentLayerParams } from './LayerMenu/params';
import Panel from '../base/Panel';
import { isMac, isWin } from './util';
import './index.less';
import { RoutingEditingOpArea } from './RoutingEditing';
import { Editor } from './RoutingEditing/Editor';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import { VizProvider, useVizStore } from './VizStore';
import { initRouteManager } from './VizStore/actions';
import { RouteManager } from './RoutingEditing/RouteManager';
import { RouteOrigin, RoutePoint } from './RoutingEditing/RouteManager/type';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type IMap = apollo.hdmap.IMap;
type IPointCloud = apollo.dreamview.IPointCloud;

function Viz() {
    const panelContext = usePanelContext();
    const [hmi] = usePickHmiStore();
    const [uid] = useState(shortUUID.generate);
    const { initSubscription, updateChannel } = panelContext;
    const { classes } = useStyle();
    const [currentView, setCurrentView] = useState('D');
    const [carviz] = useState(() => new Carviz(uid));

    const animationFrameIdRef = useRef<number>();

    const render = () => {
        carviz.render();
        animationFrameIdRef.current = requestAnimationFrame(render);
    };

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simData: ISimulationWorld) => {
                    if (!simData) return;
                    if (Object.keys(simData).length !== 0) {
                        carviz.updateData(simData);
                        carviz?.pointCloud.updateOffsetPosition();
                    }
                },
            },
            [StreamDataNames.Map]: {
                consumer: (mapData: IMap) => {
                    if (!mapData) return;
                    carviz?.updateMap(mapData);
                },
            },
            [StreamDataNames.POINT_CLOUD]: {
                consumer: (pointData: IPointCloud) => {
                    if (!pointData) return;
                    carviz?.updatePointCloud(pointData);
                },
            },
        });

        render();

        return () => {
            const animationFrameId = animationFrameIdRef.current;
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
            }
        };
    }, []);

    useEffect(() => {
        carviz.init();
        const curLayerMenuParams = getCurrentLayerParams();
        localStorage.setItem('layerMenuParams', JSON.stringify(curLayerMenuParams));
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
        document.addEventListener('keydown', (e: KeyboardEvent) => {
            if (((e.metaKey && isMac()) || (e.ctrlKey && isWin())) && e.code === 'Equal') {
                e.preventDefault();
                carviz.view?.updateViewDistance(-10);
            }
            if (((e.metaKey && isMac()) || (e.ctrlKey && isWin())) && e.code === 'Minus') {
                e.preventDefault();
                carviz.view?.updateViewDistance(10);
            }
        });
    }, []);

    const { metadata } = useWebSocketServices();

    const pointCloudFusionChannel = useMemo(() => {
        // 优先使用channels字符串数组中含有'compensator'的channel，然后如果没有使用含有'fusion'的channel
        const fusionPointCloudChannel = metadata
            .find((item) => item.dataName === StreamDataNames.POINT_CLOUD)
            ?.channels?.filter((item) => item.includes('compensator') || item.includes('fusion'))
            .sort(
                // compensator排在前面，fusion排在后面
                (a) => (a.includes('compensator') ? -1 : 1),
            );

        if (Array.isArray(fusionPointCloudChannel)) {
            return fusionPointCloudChannel[0];
        }
        return '';
    }, [metadata]);

    const pointCloudCurrentVisible = useMemo(() => {
        const curLayerMenuParams = getCurrentLayerParams();
        return curLayerMenuParams.Perception.pointCloud.currentVisible;
    }, []);

    useEffect(() => {
        if (pointCloudCurrentVisible) {
            // 如果勾选了点云按钮，则订阅点云通道
            updateChannel({
                name: StreamDataNames.POINT_CLOUD,
                needChannel: true,
                channel: pointCloudFusionChannel,
            });
        }
    }, [metadata, pointCloudCurrentVisible, pointCloudFusionChannel]);

    return (
        <div className={classes['viz-container']}>
            <div id={uid} className={classes['web-gl']} />
            <div className={classes['viz-btn-container']}>
                <ViewBtn carviz={carviz}>
                    <Popover
                        placement='leftTop'
                        content={<LayerMenu carviz={carviz} pointCloudFusionChannel={pointCloudFusionChannel} />}
                        trigger='click'
                    >
                        <span className={classes['viz-btn-item']}>
                            <IconIcCoverageHover />
                        </span>
                    </Popover>
                    <Popover
                        overlayClassName={classes['layer-menu-popover']}
                        placement='leftBottom'
                        content={<ViewMenu carviz={carviz} setCurrentView={setCurrentView} />}
                        trigger='click'
                        style={{ padding: '0 !importent' }}
                    >
                        <span className={classes['viz-btn-item']}>{currentView}</span>
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
                subscribeInfo: [
                    { name: StreamDataNames.SIM_WORLD, needChannel: false },
                    { name: StreamDataNames.Map, needChannel: false },
                ],
            }),
        [],
    );

    return <Component {...props} />;
}

Index.displayName = 'VehicleViz';

export default React.memo(VehicleViz);
