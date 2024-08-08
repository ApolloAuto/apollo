import React, { useEffect, useState, useMemo, useCallback, useRef } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { apollo } from '@dreamview/dreamview';
import { Subscription } from 'rxjs';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import CountedSubject from '@dreamview/dreamview-core/src/util/CountedSubject';
import useCarViz from '@dreamview/dreamview-core/src/hooks/useCarviz';
import { FunctionalKey } from '@dreamview/dreamview-core/src/store/EventHandlersStore';

import useStyle from '../VehicleViz/useStyle';
import ViewMenu from '../VehicleViz/ViewMenu';
import ViewBtn from '../VehicleViz/ViewBtn';
import { usePanelContext } from '../base/store/PanelStore';
import Panel from '../base/Panel';
import LayerMenu from './LayerMenu';
import { getCurrentLayerParams, formatLayerParams, localPointCloudManager } from './LayerMenu/params';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type IPointCloud = apollo.dreamview.IPointCloud;
function Cloud() {
    const [showPerception, setShowPerception] = useState(true);
    const [curChannel, setCurChannel] = useState(null);
    const panelContext = usePanelContext();
    const [hmi] = usePickHmiStore();
    const { initSubscription, setKeyDownHandlers, removeKeyDownHandlers } = panelContext;
    const { classes } = useStyle();
    const [currentView, setCurrentView] = useState('D');
    const [carviz, uid] = useCarViz();
    const { t: tPanels } = useTranslation('panels');
    const [data, setData] = useState<IPointCloud>(null);
    const { metadata, isMainConnected, streamApi } = useWebSocketServices();
    const subscriptionRef = useRef<{
        name: string;
        subscription: Subscription;
    }>(null);
    const curMeta = useMemo(() => metadata.find((meata) => meata.dataName === 'obstacle'), [metadata, isMainConnected]);
    const channels = useMemo(() => {
        if (!curMeta) {
            return [];
        }

        return curMeta.channels.map((channel) => ({
            label: channel?.channelName,
            value: channel?.channelName,
        }));
    }, [curMeta]);

    const closeChannel = () => {
        carviz?.updateData({
            object: [],
            autoDrivingCar: {},
        });
        // carviz?.render();
        if (subscriptionRef.current && subscriptionRef.current.subscription) {
            subscriptionRef.current.subscription.unsubscribe();
            subscriptionRef.current = null;
        }
    };

    const updateChannel = (v: any) => {
        closeChannel();
        const newConnectedSubj: CountedSubject<unknown> = streamApi.subscribeToDataWithChannel('obstacle', v);
        const newSubscription = newConnectedSubj.subscribe((val: any) => {
            const socketData = {
                CopyAutoDrivingCar: val.autoDrivingCar,
                object: val.obstacle || [],
            };
            carviz?.updateData(socketData);
            // carviz?.render();
        });

        subscriptionRef.current = {
            name: 'obstacle',
            subscription: newSubscription,
        };
    };

    useEffect(() => {
        carviz.init();
        const curLayerMenuParams = getCurrentLayerParams();
        localPointCloudManager.set(curLayerMenuParams);
        carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'pointCloud');

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
                discriptor: tPanels('zoomIn'),
            },
            {
                keys: ['='],
                functionalKey: 'metaKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(-10);
                },
                discriptor: tPanels('zoomIn'),
            },
            {
                keys: ['-'],
                functionalKey: 'ctrlKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(10);
                },
                discriptor: tPanels('zoomOut'),
            },
            {
                keys: ['-'],
                functionalKey: 'metaKey' as FunctionalKey,
                handler: (e) => {
                    e.preventDefault();
                    carviz.view?.updateViewDistance(10);
                },
                discriptor: tPanels('zoomOut'),
            },
        ];
        setKeyDownHandlers(handlers);

        return () => {
            removeKeyDownHandlers(handlers);
        };
    }, [tPanels]);

    useEffect(() => {
        if (channels && channels.length !== 0 && streamApi) {
            const firstChannel = channels[0];
            setCurChannel(firstChannel.value);
            if (showPerception) {
                updateChannel(firstChannel.value);
            }
        }
    }, [channels, streamApi]);

    useEffect(
        () => () => {
            subscriptionRef.current?.subscription?.unsubscribe();
        },
        [],
    );

    useEffect(() => {
        initSubscription({
            // fix: 解决点云抖动问题临时注释
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simworldData: ISimulationWorld) => {
                    carviz.adc.updateOffset(simworldData.autoDrivingCar, 'adc');
                },
            },
            [StreamDataNames.POINT_CLOUD]: {
                consumer: (pointCloudData: IPointCloud) => {
                    setData(pointCloudData);
                },
            },
        });
    }, []);

    useEffect(() => {
        if (!data) return;
        carviz?.updatePointCloud(data);
    }, [data]);

    const animationFrameIdRef = useRef<number>(0);
    const render = () => {
        carviz?.render();
        animationFrameIdRef.current = requestAnimationFrame(render);
    };

    useEffect(() => {
        render();

        return () => {
            const animationFrameId = animationFrameIdRef.current;
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
            }
        };
    }, []);

    useEffect(() => {
        setData(null);
        carviz?.removeAll();
    }, [hmi.currentRecordId]);

    const layerMenuNode = (
        <LayerMenu
            curChannel={curChannel}
            setCurChannel={setCurChannel}
            carviz={carviz}
            obstacleChannels={channels}
            updateObstacleChannel={updateChannel}
            closeChannel={closeChannel}
            showPerception={showPerception}
            setShowPerception={setShowPerception}
        />
    );
    return (
        <div className={classes['viz-container']}>
            <div id={uid} className={classes['web-gl']} />
            <div className={classes['viz-btn-container']}>
                <ViewBtn carviz={carviz}>
                    <Popover placement='leftTop' trigger='click' content={layerMenuNode}>
                        <span className={classes['viz-btn-item']}>
                            <IconPark name='IcCoverageHover' />
                        </span>
                    </Popover>
                    <Popover
                        placement='leftTop'
                        content={<ViewMenu carviz={carviz} setCurrentView={setCurrentView} />}
                        trigger='click'
                    >
                        <span className={classes['viz-btn-item']}>{currentView?.charAt(0)}</span>
                    </Popover>
                </ViewBtn>
            </div>
        </div>
    );
}

function PointCloud(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: Cloud,
                panelId: props.panelId,
                subscribeInfo: [
                    { name: StreamDataNames.POINT_CLOUD, needChannel: true },
                    { name: StreamDataNames.SIM_WORLD, needChannel: false },
                ],
            }),
        [],
    );

    return <Component {...props} />;
}

Cloud.displayName = 'PointCloud';

export default React.memo(PointCloud);
