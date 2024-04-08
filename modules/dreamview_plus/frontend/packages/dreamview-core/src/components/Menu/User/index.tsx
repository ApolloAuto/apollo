import React, { useEffect, useMemo, useState } from 'react';
import {
    IconIcSettingClick,
    IconIcCloudProfile,
    IconIcHelpHover,
    IconIcProductDocumentation,
    IconIcApolloDeveloperCommunity,
    IconIcSuggest,
    IconIcFaq,
    useImagePrak,
} from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import packageJson from '@dreamview/dreamview-core/package.json';
import { useMakeStyle, useThemeContext } from '@dreamview/dreamview-theme';
import useStyle, { useMenuItemStyle } from './useStyle';
import showSettingModal from './SettingModal';
import { CURRENT_MODE, usePickHmiStore } from '../../../store/HmiStore';
import showViewLoginModal from '../../WelcomeGuide/showViewLoginModal';
import { useUserInfoStore } from '../../../store/UserInfoStore';

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

function Divider() {
    const { classes } = useMakeStyle((theme) => ({
        'gray-divider': {
            height: '1px',
            backgroundColor: theme.tokens.colors.divider2,
        },
    }))();
    return <div className={classes['gray-divider']} />;
}

function User(props: UserProps) {
    const { setEnterGuideStateMemo } = props;
    const [hmi] = usePickHmiStore();
    const [{ userInfo }] = useUserInfoStore();
    const hasLogin = !!userInfo?.id;
    const { theme } = useThemeContext();
    const { classes } = useStyle(theme);
    const { t } = useTranslation('personal');
    const IconPersonalCenterDefault = useImagePrak('ic_personal_center_default');

    const { menusSetting, menusProfile } = useMemo(
        () => ({
            menusSetting: [
                {
                    icon: <IconIcSettingClick />,
                    text: t('setting'),
                    onClick: () => {
                        //
                        showSettingModal({
                            dreamviewVersion: packageJson.version,
                            dockerVersion: hmi?.dockerImage,
                        });
                    },
                },
                {
                    icon: <IconIcCloudProfile />,
                    text: t('cloud'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/workspace');
                    },
                },
            ],
            menusProfile: [
                {
                    icon: <IconIcFaq />,
                    text: t('faq'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/community/article/1223');
                    },
                },
                {
                    icon: <IconIcHelpHover />,
                    text: t('guide'),
                    onClick: () => {
                        setEnterGuideStateMemo(hmi.currentMode);
                    },
                },
                {
                    icon: <IconIcProductDocumentation />,
                    text: t('document'),
                    onClick: () => {
                        //
                        window.open(
                            'https://apollo.baidu.com/community/Apollo-Homepage-Document/Apollo_alpha_doc/dreamview',
                        );
                    },
                },
                {
                    icon: <IconIcApolloDeveloperCommunity />,
                    text: t('community'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com');
                    },
                },
                {
                    icon: <IconIcSuggest />,
                    text: t('technicalSupport'),
                    onClick: () => {
                        window.open('https://apollo.baidu.com/workspace?modal=feedback');
                    },
                },
            ],
        }),
        [hmi.dockerImage, hmi.currentMode, t],
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
