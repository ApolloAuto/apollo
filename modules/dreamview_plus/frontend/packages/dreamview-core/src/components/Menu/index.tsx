import React, { PropsWithChildren, useCallback, useEffect } from 'react';
import { OperatePopover, Popover, useImagePrak, IconPark } from '@dreamview/dreamview-ui';
import {
    ENUM_MENU_KEY,
    ENUM_CERT_STATUS,
    useMenuStore,
    menuStoreUtils,
} from '@dreamview/dreamview-core/src/store/MenuStore';
import { ChangeCertStatusAction, UpdateMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { useThemeContext } from '@dreamview/dreamview-theme';
import { useTranslation } from 'react-i18next';
import Logger from '@dreamview/log';
import useStyle from './useStyle';
import useWebSocketServices from '../../services/hooks/useWebSocketServices';
import User from './User';
import { useUserInfoStore } from '../../store/UserInfoStore';
import { CURRENT_MODE } from '../../store/HmiStore';
import IcMoon from './moon.svg';
import IcSun from './sun.svg';
import { useChangeTheme } from '../../store/ThemeProviderStore';

const logger = Logger.getInstance('Menu');

interface IActivatableMenu {
    onClick?: () => void;
    onMenuChange: (menu: ENUM_MENU_KEY) => void;
    menuKey: ENUM_MENU_KEY;
    activeMenuKey: ENUM_MENU_KEY;
    popover: React.JSX.Element;
}

function ActivatableMenu(props: PropsWithChildren<IActivatableMenu>) {
    const { popover, menuKey, activeMenuKey, onClick: porpsOnClick, onMenuChange } = props;
    const { classes, cx } = useStyle();

    const onClick = () => {
        onMenuChange(menuKey);
        if (porpsOnClick) {
            porpsOnClick();
        }
    };

    const popoverContent = <div className={classes['menu-item-popover']}>{popover}</div>;

    return (
        <Popover placement='right' trigger='hover' content={popoverContent}>
            <li
                onClick={onClick}
                className={cx(classes['menu-item'], {
                    [classes.active]: activeMenuKey === menuKey,
                })}
            >
                {props.children}
            </li>
        </Popover>
    );
}

const ActivatableMenuMemo = React.memo(ActivatableMenu);

type MenuProps = {
    setEnterGuideStateMemo: (currentMode: CURRENT_MODE) => void;
};

function Menu(props: MenuProps) {
    const IconPersonalCenterDefault = useImagePrak('ic_personal_center_default');
    const { setEnterGuideStateMemo } = props;
    const { classes, cx } = useStyle();
    const [{ activeMenu, certStatus }, dispatch] = useMenuStore();
    const [{ userInfo }] = useUserInfoStore();
    const hasLogin = !!userInfo?.id;
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const { t: tMode } = useTranslation('modeSettings');
    const { t: tAdd } = useTranslation('addPanel');
    const { theme } = useThemeContext();
    const setTheme = useChangeTheme();
    // 插件安装成功
    const isCertSuccess = menuStoreUtils.isCertSuccess(certStatus);

    const onMenuChange = useCallback(
        (key: ENUM_MENU_KEY) => {
            if (activeMenu === key) {
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.HIDDEN));
            } else {
                dispatch(UpdateMenuAction(key));
            }
        },
        [activeMenu],
    );

    const changeTheme = () => {
        setTheme((prev: string) => (prev === 'drak' ? 'light' : 'drak'));
    };

    useEffect(() => {
        if (isPluginConnected) {
            pluginApi
                .checkCertStatus()
                .then(() => {
                    logger.info('checkCertStatus success');
                    dispatch(ChangeCertStatusAction(ENUM_CERT_STATUS.SUCCESS));
                })
                .catch(() => {
                    dispatch(ChangeCertStatusAction(ENUM_CERT_STATUS.FAIL));
                });
        }
    }, [isPluginConnected]);

    const UserContent = <User setEnterGuideStateMemo={setEnterGuideStateMemo} />;

    return (
        <div className={classes['menu-container']}>
            <ul>
                <li className={cx(classes['menu-item'], classes.logo)} />
                <ActivatableMenuMemo
                    popover={<>{tMode('modeSettings')}</>}
                    activeMenuKey={activeMenu}
                    menuKey={ENUM_MENU_KEY.MODE_SETTING}
                    onMenuChange={onMenuChange}
                >
                    <IconPark name='IcGlobalSettingsNormal' />
                </ActivatableMenuMemo>
            </ul>
            <ul>
                {/* fixme: 本地无法使用pwa接口，暂时注释 */}
                {/* {__PLATFORM__ === 'web' && ( */}
                {/*    <li className={cx(classes['menu-item'])}> */}
                {/*        <div className={cx(classes['add-desktop'], 'add-desktop-hover')} /> */}
                {/*    </li> */}
                {/* )} */}
                <li onClick={changeTheme} className={cx(classes['menu-item'], classes['change-theme'])}>
                    <div className={classes['theme-btn']}>{theme === 'drak' ? <IcSun /> : <IcMoon />}</div>
                </li>
                <ActivatableMenuMemo
                    popover={<>{tAdd('addPanel')}</>}
                    activeMenuKey={activeMenu}
                    menuKey={ENUM_MENU_KEY.ADD_PANEL}
                    onMenuChange={onMenuChange}
                >
                    <IconPark name='IcAddPanel' />
                </ActivatableMenuMemo>
                {/* <ActivatableMenuMemo
                    popover={<>Save Panel 快捷键</>}
                    activeMenuKey={activeMenu}
                    menuKey={ENUM_MENU_KEY.SAVE_LAYOUT}
                    onMenuChange={onMenuChange}
                >
                    <IconIcSaveIcHelpNormal />
                </ActivatableMenuMemo> */}
                {isCertSuccess && (
                    <ActivatableMenuMemo
                        popover={<>{tMode('resourceManager')}</>}
                        activeMenuKey={activeMenu}
                        menuKey={ENUM_MENU_KEY.PROFILE_MANAGEER}
                        onMenuChange={onMenuChange}
                    >
                        <IconPark name='IcProfileMangerNormal' />
                    </ActivatableMenuMemo>
                )}
                <li className={cx(classes['menu-item'])}>
                    <OperatePopover
                        content={UserContent}
                        rootClassName={classes.popover}
                        placement='right'
                        trigger='hover'
                    >
                        <div className={cx(classes['menu-item-image'])}>
                            {/* eslint-disable-next-line no-nested-ternary */}
                            {hasLogin ? (
                                userInfo.avatar_url ? (
                                    <div
                                        className={classes['menu-item-avatar-image']}
                                        style={{ backgroundImage: `url(${userInfo.avatar_url})` }}
                                    />
                                ) : (
                                    <div
                                        className={classes['menu-item-avatar-image']}
                                        style={{ backgroundImage: `url(${IconPersonalCenterDefault})` }}
                                    />
                                )
                            ) : (
                                <IconPark name='IcDefaultAvatar' />
                            )}
                        </div>
                    </OperatePopover>
                </li>
            </ul>
        </div>
    );
}

export default React.memo(Menu);
