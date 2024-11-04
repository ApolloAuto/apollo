import React, { useMemo, useRef } from 'react';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore/actionTypes';
import { useThemeContext } from '@dreamview/dreamview-theme';
import AddPanel from './AddPanel';
import ModeSetting from './ModeSetting';
import ResourceManager from './ResourceManager';
import useStyle from './useStyle';
import useComponentDisplay from '../../hooks/useComponentDisplay';

const DrawerCatalog: any = {
    [ENUM_MENU_KEY.ADD_PANEL]: AddPanel,
    [ENUM_MENU_KEY.MODE_SETTING]: ModeSetting,
    [ENUM_MENU_KEY.PROFILE_MANAGEER]: ResourceManager,
    [ENUM_MENU_KEY.HIDDEN]: () => <></>,
};

const DrawerCatalogKeys = Object.keys(DrawerCatalog);

function MenuDrawer() {
    const [{ activeMenu }] = useMenuStore();
    const { tokens } = useThemeContext();
    const [, { menuDrawerWidthString }] = useComponentDisplay();

    const style = useMemo(() => {
        const backgroundColor =
            activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER ? tokens.components.meneDrawer.backgroundColor : '';
        return {
            width: menuDrawerWidthString,
            backgroundColor,
        };
    }, [activeMenu, menuDrawerWidthString, tokens]);

    const { classes, cx } = useStyle(style);

    const memo = useRef<Record<string, boolean>>({});

    memo.current[activeMenu] = true;

    const menuHidden = activeMenu === ENUM_MENU_KEY.HIDDEN;

    return (
        <div
            className={cx(classes['dv-layout-menudrawer'], classes['dv-layout-menudrawer-border'], {
                [classes.hidden]: menuHidden,
            })}
        >
            {DrawerCatalogKeys.map((item: string) => {
                const Component = DrawerCatalog[item];
                if (memo.current[item]) {
                    return (
                        <div
                            className={cx(classes['dv-layout-menudrawer-item'], {
                                [classes.hidden]: Number(item as any) !== activeMenu,
                            })}
                            key={item}
                        >
                            <Component />
                        </div>
                    );
                }
                return null;
            })}
        </div>
    );
}

export default React.memo(MenuDrawer);
