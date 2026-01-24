import React, { useCallback, useMemo, useRef, useState } from 'react';
import { Checkbox, Radio } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { useLocalStorage } from '@dreamview/dreamview-core/src/util/storageManager';
import useStyle from '../useStyle';
import { layerMenuParams, formatLayerParams, getCurrentLayerParams, localLayerMenuParamsManager } from './params';
import { usePanelContext } from '../../base/store/PanelStore';
import { StreamDataNames } from '../../../../services/api/types';
import useWebSocketServices from '../../../../services/hooks/useWebSocketServices';
import ChannelSelect from '../../base/ChannelSelect';

function LayerMenu(props: any) {
    const { classes, cx } = useStyle();
    const {
        carviz,
        pointCloudFusionChannel,
        handlePointCloudVisible,
        pointcloudChannels,
        updatePointcloudChannel,
        curChannel,
        setCurChannel,
        handleReferenceLineVisible,
        handleBoundaryLineVisible,
        handleTrajectoryLineVisible,
        handleBoudingBoxVisible,
    } = props;
    const { t } = useTranslation('layerMenu');
    const panelContext = usePanelContext();
    const { panelId } = panelContext;

    const { metadata } = useWebSocketServices();

    const checkBoxDisabled = useCallback(
        (key: string) => {
            // todo：支持包括融合点云在内的多个点云
            // if (key === 'pointCloud') {
            //     // 优先判断channels字符串数组中是否有'compensator'，然后判断是否有'fusion'
            //     const pointCloudChannels = metadata.find(
            //         (item) => item.dataName === StreamDataNames.POINT_CLOUD,
            //     )?.channels;
            //
            //     if (Array.isArray(pointCloudChannels)) {
            //         const hasCompensator = pointCloudChannels.some((item) => item.channelName.includes('compensator'));
            //         const hasFusion = pointCloudChannels.some((item) => item.channelName.includes('fusion'));
            //         return !(hasCompensator || hasFusion);
            //     }
            // }
            return false;
        },
        [metadata],
    );

    const curLayerMenuParams = getCurrentLayerParams();
    localLayerMenuParamsManager.set(curLayerMenuParams);
    carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'vehicle');

    const [layerMenu, setLayerMenu] = useState(curLayerMenuParams);
    const [menu] = useState(Object.keys(layerMenu));
    const [currentMenu, setCurrentMenu] = useState(menu[0]);
    const [subMenu, setSubMenu] = useState(layerMenu[currentMenu]);
    const pointCloudCheckedRef = useRef<boolean>();

    const resetLayerMenu = () => {
        setLayerMenu(() => layerMenuParams);
        carviz.option.updateLayerOption(formatLayerParams(layerMenuParams), 'vehicle');
        localLayerMenuParamsManager.set(layerMenuParams);
    };

    const localVizPointCloudChannel = useLocalStorage(`${panelId}-viz-pointcloud-channel`);
    const onChange = (v: any) => {
        setCurChannel(v);
        updatePointcloudChannel(v);
        localVizPointCloudChannel.set(v);
    };

    const pointCloudInfoBox = useMemo(() => {
        const { currentVisible } = layerMenu[currentMenu].pointCloud || {};
        pointCloudCheckedRef.current = currentVisible;
        return (
            <li className={classes['layer-menu-content-right-li']} key='pointCloud'>
                <span className={classes['layer-menu-content-right-switch']}>
                    <Checkbox
                        checked={currentVisible}
                        // disabled={checkBoxDisabled('pointCloud')}
                        defaultChecked={currentVisible}
                        onChange={(e) => {
                            const checked = e.target.checked;
                            pointCloudCheckedRef.current = checked;
                            const newMenu = {
                                ...layerMenu[currentMenu],
                                pointCloud: {
                                    ...layerMenu[currentMenu].pointCloud,
                                    currentVisible: checked,
                                },
                            };

                            const newLayerMenu = {
                                ...layerMenu,
                                [currentMenu]: newMenu,
                            };
                            setSubMenu(() => newMenu);
                            setLayerMenu(() => newLayerMenu);
                            carviz.option.updateLayerOption(formatLayerParams(newLayerMenu), 'vehicle');
                            localLayerMenuParamsManager.set(newLayerMenu);
                            // if (checked && !checkBoxDisabled('pointCloud') && pointCloudFusionChannel) {
                            if (checked) {
                                if (handlePointCloudVisible) {
                                    handlePointCloudVisible(e.target.checked);
                                }
                            }

                            if (!checked) {
                                if (handlePointCloudVisible) {
                                    handlePointCloudVisible(e.target.checked);
                                }
                            }
                        }}
                    />
                </span>
                <span className={classes['layer-menu-content-right-label']}>{t('pointCloud')}</span>
            </li>
        );
    }, [
        carviz.option,
        checkBoxDisabled,
        classes,
        currentMenu,
        handlePointCloudVisible,
        layerMenu,
        pointCloudFusionChannel,
        t,
    ]);

    return (
        <div className={classes['layer-menu-container']}>
            <div className={classes['layer-menu-header']}>
                <div className={classes['layer-menu-header-left']}>{t('layerMenu')}</div>
                <div className={classes['layer-menu-header-right']}>
                    <div className={classes['layer-menu-header-reset-btn']} onClick={resetLayerMenu}>
                        {t('restoreDefaultSettings')}
                    </div>
                </div>
            </div>
            <div className={classes['layer-menu-content']}>
                <div className={classes['layer-menu-content-left']}>
                    {menu.map((item) => (
                        <li
                            key={item}
                            className={cx(classes['layer-menu-content-left-li'], {
                                [classes['li-active']]: currentMenu === item,
                            })}
                            onClick={() => {
                                setCurrentMenu(item);
                                setSubMenu(layerMenu[item]);
                            }}
                        >
                            <span>{t(item)}</span>
                        </li>
                    ))}
                </div>
                <div className={classes['layer-menu-content-right']}>
                    {currentMenu === 'Perception' && (
                        <>
                            <li className={classes['layer-menu-content-right-li']}>
                                <span className={classes['layer-menu-content-right-switch']}>
                                    <Radio
                                        checked={layerMenu?.Perception?.polygon?.currentVisible}
                                        onChange={(e: any) => {
                                            const value = e.target.checked;
                                            if (value) {
                                                const newMenu = {
                                                    ...layerMenu.Perception,
                                                    polygon: {
                                                        ...layerMenu.Perception.polygon,
                                                        currentVisible: true,
                                                    },
                                                    boundingbox: {
                                                        ...layerMenu.Perception.boundingbox,
                                                        currentVisible: false,
                                                    },
                                                };
                                                const newLayerMenu = {
                                                    ...layerMenu,
                                                    [currentMenu]: newMenu,
                                                };
                                                setSubMenu(() => newMenu);
                                                setLayerMenu(() => newLayerMenu);
                                                carviz.option.updateLayerOption(
                                                    formatLayerParams(newLayerMenu),
                                                    'vehicle',
                                                );
                                                localLayerMenuParamsManager.set(newLayerMenu);
                                            }
                                        }}
                                    />
                                </span>
                                <span className={classes['layer-menu-content-right-label']}>{t('polygon')}</span>
                            </li>
                            <li className={classes['layer-menu-content-right-li']}>
                                <span className={classes['layer-menu-content-right-switch']}>
                                    <Radio
                                        checked={layerMenu?.Perception?.boundingbox?.currentVisible}
                                        onChange={(e) => {
                                            const value = e.target.checked;
                                            if (value) {
                                                const newMenu = {
                                                    ...layerMenu.Perception,
                                                    polygon: {
                                                        ...layerMenu.Perception.polygon,
                                                        currentVisible: false,
                                                    },
                                                    boundingbox: {
                                                        ...layerMenu.Perception.boundingbox,
                                                        currentVisible: true,
                                                    },
                                                };
                                                const newLayerMenu = {
                                                    ...layerMenu,
                                                    [currentMenu]: newMenu,
                                                };
                                                setSubMenu(() => newMenu);
                                                setLayerMenu(() => newLayerMenu);
                                                carviz.option.updateLayerOption(
                                                    formatLayerParams(newLayerMenu),
                                                    'vehicle',
                                                );
                                                localLayerMenuParamsManager.set(newLayerMenu);
                                            }
                                        }}
                                    />
                                </span>
                                <span className={classes['layer-menu-content-right-label']}>{t('boundingbox')}</span>
                            </li>
                            <div className={classes['layer-menu-horizontal-line']} />
                        </>
                    )}

                    {pointCloudInfoBox}
                    <ChannelSelect
                        disabled={!pointCloudCheckedRef.current}
                        value={curChannel}
                        options={pointcloudChannels}
                        onChange={(v) => onChange(v)}
                    />
                    <div className={classes['layer-menu-horizontal-line']} />

                    {Object.keys(subMenu).map((key) => {
                        if (key === 'pointCloud') {
                            return null;
                        }

                        const { currentVisible } = layerMenu[currentMenu][key] || {};
                        return (
                            key !== 'polygon' &&
                            key !== 'boundingbox' && (
                                <li className={classes['layer-menu-content-right-li']} key={key}>
                                    <span className={classes['layer-menu-content-right-switch']}>
                                        <Checkbox
                                            checked={currentVisible}
                                            disabled={checkBoxDisabled(key)}
                                            defaultChecked={currentVisible}
                                            onChange={(e) => {
                                                const checked = e.target.checked;
                                                const newMenu = {
                                                    ...layerMenu[currentMenu],
                                                    [key]: {
                                                        ...layerMenu[currentMenu][key],
                                                        currentVisible: checked,
                                                    },
                                                };
                                                const newLayerMenu = {
                                                    ...layerMenu,
                                                    [currentMenu]: newMenu,
                                                };
                                                setSubMenu(() => newMenu);
                                                setLayerMenu(() => newLayerMenu);
                                                carviz.option.updateLayerOption(
                                                    formatLayerParams(newLayerMenu),
                                                    'vehicle',
                                                );
                                                localLayerMenuParamsManager.set(newLayerMenu);

                                                if (
                                                    checked &&
                                                    key === 'pointCloud' &&
                                                    !checkBoxDisabled(key) &&
                                                    pointCloudFusionChannel
                                                ) {
                                                    if (handlePointCloudVisible) {
                                                        handlePointCloudVisible(e.target.checked);
                                                    }
                                                }

                                                if (!checked && key === 'pointCloud') {
                                                    if (handlePointCloudVisible) {
                                                        handlePointCloudVisible(e.target.checked);
                                                    }
                                                }
                                                if (key === 'planningReferenceLine') {
                                                    handleReferenceLineVisible(checked);
                                                }

                                                if (key === 'planningBoundaryLine') {
                                                    handleBoundaryLineVisible(checked);
                                                }

                                                if (key === 'planningTrajectoryLine') {
                                                    handleTrajectoryLineVisible(checked);
                                                }

                                                if (key === 'egoBoudingBox') {
                                                    handleBoudingBoxVisible(checked);
                                                }
                                            }}
                                        />
                                    </span>
                                    <span className={classes['layer-menu-content-right-label']}>{t(key)}</span>
                                </li>
                            )
                        );
                    })}
                </div>
            </div>
        </div>
    );
}

export default React.memo(LayerMenu);
