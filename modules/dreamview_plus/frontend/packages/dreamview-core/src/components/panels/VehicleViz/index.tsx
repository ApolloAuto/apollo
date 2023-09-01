import React, { useEffect, useState, useMemo } from 'react';
import Carviz from '@dreamview/dreamview-carviz/src/index';
import { Popover, IconIcCoverageHover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import shortUUID from 'short-uuid';
import type { apollo } from '@dreamview/dreamview';
import useStyle from './useStyle';
import LayerMenu from './LayerMenu';
import ViewMenu from './ViewMenu';
import ViewBtn from './ViewBtn';
import { usePanelContext } from '../base/store/PanelStore';
import { usePickHmiStore } from '../../../store/HmiStore';
import { formatLayerParams, getCurrentLayerParams } from './LayerMenu/params';
import Panel from '../base/Panel';
import { StreamDataNames } from '../../../services/api/types';
import { isMac, isWin } from './util';
import './index.less';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;
type IMap = apollo.hdmap.IMap;
type IPointCloud = apollo.dreamview.IPointCloud;

function Viz() {
    const panelContext = usePanelContext();
    const [hmi] = usePickHmiStore();
    const [uid] = useState(shortUUID.generate);
    const { initSubscription, logger, updateChannel } = panelContext;
    const { classes } = useStyle();
    const [currentView, setCurrentView] = useState('D');
    const [carviz] = useState(() => new Carviz(uid));
    const { t } = useTranslation('viewMenu');

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simDdata) => {
                    if (!simDdata) return;
                    if (Object.keys(simDdata).length !== 0) {
                        carviz.updateData(simDdata);
                        carviz.render();
                    }
                },
            },
            [StreamDataNames.Map]: {
                consumer: (mapData) => {
                    if (!mapData) return;
                    const data = {
                        map: mapData,
                    };
                    carviz?.updateData(data);
                    carviz.render();
                },
            },
            [StreamDataNames.POINT_CLOUD]: {
                consumer: (pointData) => {
                    if (!pointData) return;
                    const data = {
                        pointCloud: pointData,
                    };
                    carviz?.updateData(data);
                    carviz.render();
                },
            },
        });
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

    useEffect(() => {
        carviz?.removeAll();
    }, [hmi.currentRecordId]);
    return (
        <div className={classes['viz-container']}>
            <div id={uid} className={classes['web-gl']} />
            <div className={classes['viz-btn-container']}>
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
                <ViewBtn carviz={carviz} />
            </div>
        </div>
    );
}

export default function VehicleViz(props: any) {
    const Component = useMemo(
        () =>
            Panel(Viz, props.panelId, [
                { name: StreamDataNames.SIM_WORLD, needChannel: false },
                { name: StreamDataNames.Map, needChannel: false },
            ]),
        [],
    );

    return <Component {...props} />;
}

Viz.displayName = 'VehicleViz';
