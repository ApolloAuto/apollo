import React, { useEffect } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { WebSocketManager } from '@dreamview/dreamview-core/src/services/WebSocketManager';
import Logger from '@dreamview/log';
import usePageLayoutStyles from '@dreamview/dreamview-core/src/components/PageLayout/style';

import Menu from '.';
import MenuDrawer from '../MenuDrawer';

import { PageLayoutStoreProvider } from '../../store/PageLayoutStore';
import { ENUM_MENU_KEY } from '../../store/MenuStore/actionTypes';
import { UpdateMenuAction } from '../../store/MenuStore/actions';
import { useMenuStore } from '../../store/MenuStore';

const logger = Logger.getInstance('WebSocketManagerStory');

type StoryObjComponentProps = { menu: ENUM_MENU_KEY };

function PageLayout(props: StoryObjComponentProps) {
    const { classes: c, cx } = usePageLayoutStyles();

    const [state, dispatch] = useMenuStore();

    const menu = props.menu;

    useEffect(() => {
        dispatch(UpdateMenuAction(menu));
    }, [dispatch, menu]);

    return (
        <div className={c['dv-root']}>
            <div className={c['dv-layout-menu']}>
                <Menu />
            </div>
            <div className={c['dv-layout-content']}>
                <div className={c['dv-layout-window']}>
                    <MenuDrawer />
                    <div className={c['dv-layout-panellayout']}>{/* <PanelLayout /> */}</div>
                </div>
                <div
                    className={cx(c['dv-layout-playercontrol'], {
                        [c['dv-layout-playercontrol-active']]: false,
                    })}
                >
                    {/* <PlayerControlBar /> */}
                </div>
            </div>
        </div>
    );
}

function PageLayoutWrapper(props: StoryObjComponentProps) {
    const menu = props.menu;
    return (
        <PageLayoutStoreProvider>
            <PageLayout menu={menu} />
        </PageLayoutStoreProvider>
    );
}

const meta: Meta<typeof PageLayoutWrapper> = {
    title: 'Dreamview/Menu',
    component: PageLayoutWrapper,
};

export const AddPanel: StoryObj<typeof PageLayoutWrapper> = {
    args: { menu: ENUM_MENU_KEY.ADD_PANEL },
};

export const ModeSetting: StoryObj<typeof PageLayoutWrapper> = {
    args: { menu: ENUM_MENU_KEY.MODE_SETTING },
};

export const ProfileManageer: StoryObj<typeof PageLayoutWrapper> = {
    args: { menu: ENUM_MENU_KEY.PROFILE_MANAGEER },
};

export const Hidden: StoryObj<typeof PageLayoutWrapper> = {
    args: { menu: ENUM_MENU_KEY.HIDDEN },
};

export default meta;
