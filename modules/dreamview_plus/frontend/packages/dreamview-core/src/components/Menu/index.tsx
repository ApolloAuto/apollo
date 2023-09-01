import React, { PropsWithChildren, useCallback, useEffect } from 'react';
import {
    IconIcGlobalSettingsNormal,
    IconIcAddPanel,
    IconIcProfileMangerNormal,
    IconIcDefaultAvatar,
    Popover,
} from '@dreamview/dreamview-ui';
import {
    ENUM_MENU_KEY,
    ENUM_CERT_STATUS,
    useMenuStore,
    menuStoreUtils,
} from '@dreamview/dreamview-core/src/store/MenuStore';
import { ChangeCertStatusAction, UpdateMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { useTranslation } from 'react-i18next';
import IconPersonalCenterDefault from '@dreamview/dreamview-core/src/assets/ic_personal_center_default.png';
import useStyle from './useStyle';
import useWebSocketServices from '../../services/hooks/useWebSocketServices';
import User from './User';
import { useUserInfoStore } from '../../store/UserInfoStore';
import { CURRENT_MODE } from '../../store/HmiStore';

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
        <Popover rootClassName={classes.popoverBg} placement='right' trigger='hover' content={popoverContent}>
            <li
                onClick={onClick}
                className={cx(classes['menu-item'], classes['menu-item-icon'], {
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
    const { setEnterGuideStateMemo } = props;
    const { classes, cx } = useStyle();
    const [{ activeMenu, certStatus }, dispatch] = useMenuStore();
    const [{ userInfo }] = useUserInfoStore();
    const hasLogin = !!userInfo?.id;
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const { t: tMode } = useTranslation('modeSettings');
    const { t: tAdd } = useTranslation('addPanel');

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

    useEffect(() => {
        if (isPluginConnected) {
            pluginApi
                .checkCertStatus()
                .then(() => {
                    console.log('checkCertStatus success');
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
                    <IconIcGlobalSettingsNormal />
                </ActivatableMenuMemo>
            </ul>
            <ul>
                {/* fixme: 本地无法使用pwa接口，暂时注释 */}
                {/* {__PLATFORM__ === 'web' && ( */}
                {/*    <li className={cx(classes['menu-item'])}> */}
                {/*        <div className={cx(classes['add-desktop'], 'add-desktop-hover')} /> */}
                {/*    </li> */}
                {/* )} */}
                <ActivatableMenuMemo
                    popover={<>{tAdd('addPanel')}</>}
                    activeMenuKey={activeMenu}
                    menuKey={ENUM_MENU_KEY.ADD_PANEL}
                    onMenuChange={onMenuChange}
                >
                    <IconIcAddPanel />
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
                        popover={<>{tMode('profileManager')}</>}
                        activeMenuKey={activeMenu}
                        menuKey={ENUM_MENU_KEY.PROFILE_MANAGEER}
                        onMenuChange={onMenuChange}
                    >
                        <IconIcProfileMangerNormal />
                    </ActivatableMenuMemo>
                )}
                <li className={cx(classes['menu-item'])}>
                    <Popover content={UserContent} rootClassName={classes.popover} placement='right' trigger='hover'>
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
                                <IconIcDefaultAvatar />
                            )}
                        </div>
                    </Popover>
                </li>
            </ul>
        </div>
    );
}

export default React.memo(Menu);
