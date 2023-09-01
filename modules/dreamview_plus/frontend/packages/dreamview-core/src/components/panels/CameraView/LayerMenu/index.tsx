import React, { useState } from 'react';
import { Switch, Checkbox } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from '../useStyle';
import { layerMenuParams } from './params';

export default function LayerMenu(props: { setShowBoundingBox: any }) {
    const setShowBoundingBox = props.setShowBoundingBox;

    const { classes, cx } = useStyle();

    const { t } = useTranslation('layerMenu');

    const [layerMenu, setLayerMenu] = useState(layerMenuParams);
    const [menu] = useState(Object.keys(layerMenu));
    const [currentMenu, setCurrentMenu] = useState(menu[0]);
    const [subMenu, setSubMenu] = useState(layerMenu[currentMenu]);

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
                                        defaultChecked={currentVisible}
                                        onChange={(e) => {
                                            const checked = e.target.checked;
                                            setShowBoundingBox(checked);
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
