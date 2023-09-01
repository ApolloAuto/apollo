import React, { useEffect, useMemo, useState } from 'react';
import {
    IconIcSettingClick,
    IconIcCloudProfile,
    IconIcHelpHover,
    IconIcProductDocumentation,
    IconIcApolloDeveloperCommunity,
    IconIcSuggest,
} from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import packageJson from '@dreamview/dreamview-core/package.json';
import IconPersonalCenterDefault from '@dreamview/dreamview-core/src/assets/ic_personal_center_default.png';
import popupFeedbackForm from '@dreamview/dreamview-core/src/util/popupFeedbackForm';
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

function User(props: UserProps) {
    const { setEnterGuideStateMemo } = props;
    const [hmi] = usePickHmiStore();
    const [{ userInfo }] = useUserInfoStore();
    const hasLogin = !!userInfo?.id;
    const { classes } = useStyle();
    const { t } = useTranslation('personal');

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
                        window.open('https://apollo.baidu.com/community/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/dreamview');
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

            <MenuItemMemo menus={menusSetting} />
            <MenuItemMemo menus={menusProfile} />
        </>
    );
}

export default React.memo(User);
