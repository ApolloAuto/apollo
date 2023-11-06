import React, { useEffect, useState, useMemo, useCallback, useRef } from 'react';
import { Carviz } from '@dreamview/dreamview-carviz/src/index';
import { IconIcCoverageHover, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import shortUUID from 'short-uuid';
import { apollo } from '@dreamview/dreamview';
import { Subscription } from 'rxjs';
import useStyle from '../VehicleViz/useStyle';
import ViewMenu from '../VehicleViz/ViewMenu';
import ViewBtn from '../VehicleViz/ViewBtn';
import { usePanelContext } from '../base/store/PanelStore';
import { usePickHmiStore } from '../../../store/HmiStore';
import Panel from '../base/Panel';
import { StreamDataNames } from '../../../services/api/types';
import { isMac, isWin } from '../VehicleViz/util';
import LayerMenu from './LayerMenu';
import { getCurrentLayerParams, formatLayerParams } from './LayerMenu/params';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import CountedSubject from '../../../util/CountedSubject';

type IPointCloud = apollo.dreamview.IPointCloud;

function Cloud() {
    const [showPerception, setShowPerception] = useState(true);
    const [curChannel, setCurChannel] = useState(null);
    const panelContext = usePanelContext();
    const [uid] = useState(shortUUID.generate);
    const [hmi] = usePickHmiStore();
    const { initSubscription, logger, data: subcribedData } = panelContext;
    const { classes } = useStyle();
    const [currentView, setCurrentView] = useState('D');
    const [carviz] = useState(() => new Carviz(uid));
    const { t } = useTranslation('viewMenu');
    const [data, setData] = useState<IPointCloud>();

    useEffect(() => {
        initSubscription({
            [StreamDataNames.POINT_CLOUD]: {
                consumer: (pointCloudData) => {
                    setData(pointCloudData);
                },
            },
        });
    }, []);

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
            label: channel,
            value: channel,
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
                object: val.obstacle || [],
                autoDrivingCar: val.autoDrivingCar,
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
        if (channels && channels.length !== 0 && streamApi) {
            const firstChannel = channels[0];
            setCurChannel(firstChannel.value);
            if (showPerception) {
                updateChannel(firstChannel.value);
            }
        }
    }, [channels, streamApi]);

    useEffect(() => {
        carviz.init();
        const curLayerMenuParams = getCurrentLayerParams();
        localStorage.setItem('pointCloudLayerMenuParams', JSON.stringify(curLayerMenuParams));
        carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'pointCloud');

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

    useEffect(
        () => () => {
            subscriptionRef.current?.subscription?.unsubscribe();
        },
        [],
    );

    useEffect(() => {
        if (!data) return;
        carviz?.updatePointCloud(data);
        // carviz.render();
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
                <Popover placement='leftTop' trigger='click' content={layerMenuNode}>
                    <span className={classes['viz-btn-item']}>
                        <IconIcCoverageHover />
                    </span>
                </Popover>
                <Popover
                    placement='leftTop'
                    content={<ViewMenu carviz={carviz} setCurrentView={setCurrentView} />}
                    trigger='click'
                >
                    <span className={classes['viz-btn-item']}>{currentView}</span>
                </Popover>
                <ViewBtn carviz={carviz} />
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
                subscribeInfo: [{ name: StreamDataNames.POINT_CLOUD, needChannel: true }],
            }),
        [],
    );

    return <Component {...props} />;
}

Cloud.displayName = 'PointCloud';

export default React.memo(PointCloud);
