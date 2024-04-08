import React, { useState } from 'react';
import { Checkbox, Radio } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from '../../VehicleViz/useStyle';
import { localPointCloudManager, formatLayerParams, getCurrentLayerParams } from './params';
import ChannelSelect from '../../base/ChannelSelect';

export default function LayerMenu(props: any) {
    const { classes } = useStyle();
    const {
        carviz,
        obstacleChannels,
        updateObstacleChannel,
        closeChannel,
        showPerception,
        setShowPerception,
        curChannel,
        setCurChannel,
    } = props;
    const { t } = useTranslation('layerMenu');
    const { t: tPanels } = useTranslation('panels');

    const curLayerMenuParams = getCurrentLayerParams();

    localPointCloudManager.set(curLayerMenuParams);
    carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'pointCloud');

    const [layerMenu, setLayerMenu] = useState(curLayerMenuParams);
    const [menu] = useState(Object.keys(layerMenu));
    const [currentMenu] = useState(menu[0]);
    const [, setSubMenu] = useState(layerMenu[currentMenu]);

    const onChange = (v: any) => {
        setCurChannel(v);
        updateObstacleChannel(v);
    };

    return (
        <div className={classes['layer-menu-container']}>
            <div className={classes['layer-menu-header']}>
                <div className={classes['layer-menu-header-left']}>{tPanels('layerMenu')}</div>
            </div>
            <div className={classes['layer-menu-content']}>
                <div className={classes['layer-menu-content-left']}>
                    <span
                        className={classes['layer-menu-content-right-switch']}
                        style={{ display: 'inline-block', marginTop: '6px' }}
                    >
                        <Checkbox
                            checked={showPerception}
                            defaultChecked
                            onChange={(e) => {
                                const checked = e.target.checked;
                                if (checked && obstacleChannels.length) {
                                    setShowPerception(true);
                                    updateObstacleChannel(curChannel);
                                } else {
                                    setShowPerception(false);
                                    closeChannel();
                                }
                            }}
                        />
                    </span>
                    {t('perception')}
                </div>
                <div className={classes['layer-menu-content-right']}>
                    <ChannelSelect value={curChannel} options={obstacleChannels} onChange={(v) => onChange(v)} />
                    <div className={classes['layer-menu-horizontal-line']} />
                    <li className={classes['layer-menu-content-right-li']}>
                        <span className={classes['layer-menu-content-right-switch']}>
                            <Radio
                                checked={layerMenu.Perception.polygon.currentVisible}
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
                                        carviz.option.updateLayerOption(formatLayerParams(newLayerMenu), 'vehicle');
                                        localPointCloudManager.set(newLayerMenu);
                                    }
                                }}
                            />
                        </span>
                        <span className={classes['layer-menu-content-right-label']}>{t('polygon')}</span>
                    </li>
                    <li className={classes['layer-menu-content-right-li']}>
                        <span className={classes['layer-menu-content-right-switch']}>
                            <Radio
                                checked={layerMenu.Perception.boundingbox.currentVisible}
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
                                        carviz.option.updateLayerOption(formatLayerParams(newLayerMenu), 'vehicle');
                                        localPointCloudManager.set(newLayerMenu);
                                    }
                                }}
                            />
                        </span>
                        <span className={classes['layer-menu-content-right-label']}>{t('boundingbox')}</span>
                    </li>
                </div>
            </div>
        </div>
    );
}
