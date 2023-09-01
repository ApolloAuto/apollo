import React, { useEffect, useMemo, useState } from 'react';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore/actionTypes';
import AddPanel from './AddPanel';
import ModeSetting from './ModeSetting';
import ProfileManager from './ProfileManager';
import useStyle from './useStyle';

interface IMenuDrawer {}

const DrawerCatalog: any = {
    [ENUM_MENU_KEY.ADD_PANEL]: AddPanel,
    [ENUM_MENU_KEY.MODE_SETTING]: ModeSetting,
    [ENUM_MENU_KEY.PROFILE_MANAGEER]: ProfileManager,
    // eslint-disable-next-line react/jsx-no-useless-fragment
    [ENUM_MENU_KEY.HIDDEN]: () => <></>,
};

const DrawerCatalogKeys = Object.keys(DrawerCatalog);

function MenuDrawer(props: IMenuDrawer) {
    const [{ activeMenu }] = useMenuStore();

    const style = useMemo(() => {
        const width = (() => {
            if (activeMenu === ENUM_MENU_KEY.HIDDEN) {
                return '0px';
            }
            if (activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER) {
                return 'calc(100vw - 64px)';
            }
            return '370px';
        })();
        const backgroundColor = activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER ? '#16181E' : '';
        return {
            width,
            backgroundColor,
        };
    }, [activeMenu]);

    const { classes, cx } = useStyle()(style);

    const memo = React.useRef<Record<string, boolean>>({});

    memo.current[activeMenu] = true;

    const menuHidden = activeMenu === ENUM_MENU_KEY.HIDDEN;

    return (
        <div
            className={cx(classes['dv-layout-menudrawer'], classes['dv-layout-menudrawer-border'], {
                [classes.hidden]: menuHidden,
            })}
        >
            {DrawerCatalogKeys.map((item: string) => {
                const Component = React.memo(DrawerCatalog[item]);
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
