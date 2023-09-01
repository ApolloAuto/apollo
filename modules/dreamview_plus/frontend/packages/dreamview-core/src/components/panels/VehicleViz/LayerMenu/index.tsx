import React, { useCallback, useState } from 'react';
import { Checkbox, Radio } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from '../useStyle';
import { layerMenuParams, formatLayerParams, getCurrentLayerParams } from './params';
import { usePanelContext } from '../../base/store/PanelStore';
import { StreamDataNames } from '../../../../services/api/types';
import useWebSocketServices from '../../../../services/hooks/useWebSocketServices';

export default function LayerMenu(props: any) {
    const { classes, cx } = useStyle();
    const { carviz, pointCloudFusionChannel } = props;
    const { t } = useTranslation('layerMenu');
    const panelContext = usePanelContext();
    const { updateChannel, closeSubcription } = panelContext;

    const { metadata } = useWebSocketServices();

    const checkBoxDisabled = useCallback(
        (key: string) => {
            if (key === 'pointCloud') {
                // 优先判断channels字符串数组中是否有'compensator'，然后判断是否有'fusion'
                const pointCloudChannels = metadata.find(
                    (item) => item.dataName === StreamDataNames.POINT_CLOUD,
                )?.channels;

                if (Array.isArray(pointCloudChannels)) {
                    const hasCompensator = pointCloudChannels.some((item) => item.includes('compensator'));
                    const hasFusion = pointCloudChannels.some((item) => item.includes('fusion'));
                    return !(hasCompensator || hasFusion);
                }
            }
            return false;
        },
        [metadata],
    );

    const curLayerMenuParams = getCurrentLayerParams();
    localStorage.setItem('layerMenuParams', JSON.stringify(curLayerMenuParams));
    carviz.option.updateLayerOption(formatLayerParams(curLayerMenuParams), 'vehicle');

    const [layerMenu, setLayerMenu] = useState(curLayerMenuParams);
    const [menu] = useState(Object.keys(layerMenu));
    const [currentMenu, setCurrentMenu] = useState(menu[0]);
    const [subMenu, setSubMenu] = useState(layerMenu[currentMenu]);

    const resetLayerMenu = () => {
        setLayerMenu(() => layerMenuParams);
        carviz.option.updateLayerOption(formatLayerParams(layerMenuParams), 'vehicle');
        localStorage.setItem('layerMenuParams', JSON.stringify(layerMenuParams));
    };
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
                                                localStorage.setItem('layerMenuParams', JSON.stringify(newLayerMenu));
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
                                                localStorage.setItem('layerMenuParams', JSON.stringify(newLayerMenu));
                                            }
                                        }}
                                    />
                                </span>
                                <span className={classes['layer-menu-content-right-label']}>{t('boundingbox')}</span>
                            </li>
                            <div className={classes['layer-menu-horizontal-line']} />
                        </>
                    )}

                    {Object.keys(subMenu).map((key) => {
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
                                                localStorage.setItem('layerMenuParams', JSON.stringify(newLayerMenu));
                                                if (
                                                    checked &&
                                                    key === 'pointCloud' &&
                                                    !checkBoxDisabled(key) &&
                                                    pointCloudFusionChannel
                                                ) {
                                                    updateChannel({
                                                        name: StreamDataNames.POINT_CLOUD,
                                                        needChannel: true,
                                                        channel: pointCloudFusionChannel,
                                                    });
                                                }
                                                if (!checked && key === 'pointCloud') {
                                                    carviz.removePointCloud();
                                                    closeSubcription(StreamDataNames.POINT_CLOUD);
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
