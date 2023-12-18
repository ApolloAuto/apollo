import React, { useEffect, useState } from 'react';
import { Checkbox } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { useLocalStorage, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';
import useStyle from '../useStyle';
import { layerMenuParams } from './params';

function LayerMenu(props: { setShowBoundingBox: any }) {
    const panelContext = usePanelContext();
    const { panelId } = panelContext;
    const setShowBoundingBox = props.setShowBoundingBox;

    const { classes, cx } = useStyle();

    const { t } = useTranslation('layerMenu');

    const [layerMenu, setLayerMenu] = useState(layerMenuParams);
    const [menu] = useState(() => Object.keys(layerMenu));
    const [currentMenu, setCurrentMenu] = useState(menu[0]);
    const [subMenu, setSubMenu] = useState(layerMenu[currentMenu]);

    const localLayerMenuManager = useLocalStorage(`${KEY_MANAGER.layerMenu}${panelId}`);
    const localBoundingBoxManager = useLocalStorage(`${KEY_MANAGER.BBox}${panelId}`);

    useEffect(() => {
        const localLayerMenu = localLayerMenuManager.get();
        if (localLayerMenu) {
            setLayerMenu(localLayerMenu);
            setCurrentMenu(menu[0]);
            setSubMenu(localLayerMenu[menu[0]]);
        }
    }, []);

    return (
        <div className={classes['layer-menu-container']}>
            <div className={classes['layer-menu-header']}>
                <div className={classes['layer-menu-header-left']}>{t('layerMenu')}</div>
                <div className={classes['layer-menu-header-right']} />
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
                            <span>{item}</span>
                        </li>
                    ))}
                </div>
                <div className={classes['layer-menu-content-right']}>
                    {Object.keys(subMenu).map((key) => {
                        const { currentVisible } = layerMenu[currentMenu][key];
                        return (
                            <li key={key}>
                                <span>
                                    <Checkbox
                                        checked={currentVisible}
                                        defaultChecked={localBoundingBoxManager.get() === '1'}
                                        onChange={(e) => {
                                            const checked = e.target.checked;
                                            setShowBoundingBox(checked);
                                            localBoundingBoxManager.set(checked ? 1 : 0);
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
                                            localLayerMenuManager.set(newLayerMenu);
                                        }}
                                    >
                                        {key}
                                    </Checkbox>
                                </span>
                            </li>
                        );
                    })}
                </div>
            </div>
        </div>
    );
}

export default React.memo(LayerMenu);
