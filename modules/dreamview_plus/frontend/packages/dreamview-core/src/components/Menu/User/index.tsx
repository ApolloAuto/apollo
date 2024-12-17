import React, { useEffect, useMemo, useState } from 'react';
import { IconPark, useImagePrak } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import packageJson from '@dreamview/dreamview-core/package.json';
import { makeStyles, useThemeContext } from '@dreamview/dreamview-theme';
import useStyle, { useMenuItemStyle } from './useStyle';
import showSettingModal from './SettingModal';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import { CURRENT_MODE, usePickHmiStore } from '../../../store/HmiStore';
import showViewLoginModal from '../../WelcomeGuide/showViewLoginModal';
import { useUserInfoStore } from '../../../store/UserInfoStore';
import { updateSubscribe } from '../../../store/UserInfoStore/actions';

interface IMenuItem {
    icon: React.ReactNode;
    text: string;
    onClick: () => void;
}

function MenuItem(props: { menus: IMenuItem[] }) {
    const { classes } = useMenuItemStyle();
    const { menus } = props;
    return (
        <div className={classes.menu}>
            {menus.map((item) => (
                <div onClick={item.onClick} className={classes['menu-item']} key={item.text}>
                    {item.icon}
                    {item.text}
                </div>
            ))}
        </div>
    );
}

const MenuItemMemo = React.memo(MenuItem);

type UserProps = {
    setEnterGuideStateMemo: (currentMode: CURRENT_MODE) => void;
};

const useDividerStyle = makeStyles((theme) => ({
    'gray-divider': {
        height: '1px',
        backgroundColor: theme.tokens.colors.divider2,
    },
}));

function Divider() {
    const { classes } = useDividerStyle();
    return <div className={classes['gray-divider']} />;
}

function User(props: UserProps) {
    const { setEnterGuideStateMemo } = props;
    const [hmi] = usePickHmiStore();
    const [{ isLogin, userInfo, account }, dispatch] = useUserInfoStore();
    const hasLogin = !!userInfo?.id;
    const { theme } = useThemeContext();
    const { classes } = useStyle(theme);
    const { t } = useTranslation('personal');
    const IconPersonalCenterDefault = useImagePrak('ic_personal_center_default');
    const { isPluginConnected, pluginApi, mainApi } = useWebSocketServices();

    const updateSubscribeAccount = () => {
        dispatch(updateSubscribe(pluginApi));
    };

    const { menusSetting, menusProfile } = useMemo(
        () => ({
            menusSetting: [
                {
                    icon: <IconPark name='IcSettingClick' />,
                    text: t('setting'),
                    onClick: () => {
                        //
                        showSettingModal({
                            dreamviewVersion: packageJson.version,
                            dockerVersion: hmi?.dockerImage,
                            updateSubscribeAccount,
                            pluginApi,
                            isLogin,
                            userInfo,
                        });
                    },
                },
                {
                    icon: <IconPark name='IcCloudProfile' />,
                    text: t('cloud'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/workspace');
                    },
                },
            ],
            menusProfile: [
                {
                    icon: <IconPark name='IcFaq' />,
                    text: t('faq'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/community/article/1223');
                    },
                },
                {
                    icon: <IconPark name='IcHelpHover' />,
                    text: t('guide'),
                    onClick: () => {
                        setEnterGuideStateMemo(hmi.currentMode);
                    },
                },
                {
                    icon: <IconPark name='IcProductDocumentation' />,
                    text: t('document'),
                    onClick: () => {
                        //
                        window.open(
                            'https://apollo.baidu.com/community/Apollo-Homepage-Document/Apollo_alpha_doc/dreamview',
                        );
                    },
                },
                {
                    icon: <IconPark name='IcApolloDeveloperCommunity' />,
                    text: t('community'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com');
                    },
                },
                {
                    icon: <IconPark name='IcSuggest' />,
                    text: t('technicalSupport'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/workspace?modal=feedback');
                    },
                },
            ],
        }),
        [hmi.dockerImage, hmi.currentMode, t, pluginApi, userInfo, isLogin],
    );

    return (
        <>
            {hasLogin ? (
                <div className={classes.userinfo}>
                    <div className={classes.avatar}>
                        {userInfo.avatar_url ? (
                            <img alt='头像' src={userInfo?.avatar_url} />
                        ) : (
                            <div
                                style={{ backgroundImage: `url(${IconPersonalCenterDefault})` }}
                                className={classes['avatar-default']}
                            />
                        )}
                    </div>
                    <div className={classes.displayname}>{userInfo?.displayname}</div>
                </div>
            ) : (
                <>
                    <div className={classes.title}>{t('loginTip')}</div>
                    <div onClick={showViewLoginModal} className={classes.login}>
                        {t('loginGuide')}
                    </div>
                </>
            )}
            <Divider />
            <MenuItemMemo menus={menusSetting} />
            <Divider />
            <MenuItemMemo menus={menusProfile} />
        </>
    );
}

export default React.memo(User);
