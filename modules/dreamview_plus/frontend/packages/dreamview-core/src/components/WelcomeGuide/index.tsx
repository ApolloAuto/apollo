import React, { useEffect, useState } from 'react';
import { Tabs, Button, useImagePrak } from '@dreamview/dreamview-ui';
import { usePickHmiStore, CURRENT_MODE, changeMode } from '@dreamview/dreamview-core/src/store/HmiStore';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { useUserInfoStore } from '../../store/UserInfoStore';
import showViewLoginModal from './showViewLoginModal';
import BackgroundEffect from './BackgroundEffect';
import { StreamDataNames } from '../../services/api/types';

type WelcomeGuideTabsContentProps = {
    contentImageUrl: string;
    clickEnterThisMode: () => void;
    currentMode: CURRENT_MODE;
};

function WelcomeGuideTabsContent(props: WelcomeGuideTabsContentProps) {
    const { contentImageUrl, clickEnterThisMode, currentMode } = props;
    const { t } = useTranslation('guide');
    const { classes } = useStyle();

    return (
        <div className={classes['welcome-guide-content-tab']}>
            <div
                className={classes['welcome-guide-content-tab-image']}
                style={{ backgroundImage: `url(${contentImageUrl})` }}
            />
            <div className={classes['welcome-guide-content-tab-text']}>
                <div className={classes['welcome-guide-content-tab-text-desc']}>{t(`${currentMode}Desc`)}</div>
                <div className={classes['welcome-guide-content-tab-text-modules-panel']}>
                    {`${t('modules')}:`}
                    <div className={classes['welcome-guide-content-tab-text-modules-panel-desc']}>
                        {t(`${currentMode}Modules`)}
                    </div>
                </div>
                <div className={classes['welcome-guide-content-tab-text-modules-panel']}>
                    {`${t('panel')}:`}
                    <div className={classes['welcome-guide-content-tab-text-modules-panel-desc']}>
                        {t(`${currentMode}Panel`)}
                    </div>
                </div>
                <div className={classes['enter-this-mode']}>
                    <Button className={classes['enter-this-mode-btn']} onClick={() => clickEnterThisMode()}>
                        {t('enterThisMode')}
                    </Button>
                </div>
            </div>
        </div>
    );
}

type WelcomeGuideProps = {
    clickEnterThisModeToGuide: (useGuideMode: CURRENT_MODE) => void;
    setLocalFirstUseGuideMode: (useGuideMode: CURRENT_MODE) => void;
};

function WelcomeGuide(props: WelcomeGuideProps) {
    const { clickEnterThisModeToGuide, setLocalFirstUseGuideMode } = props;

    const [
        welcomeGuideTabsImageDefault,
        welcomeGuideTabsImagePerception,
        welcomeGuideTabsImageVehicle,
        welcomeGuideDecorateEle,
        welcomeGuideTabsImagePNC,
    ] = useImagePrak([
        'welcome_guide_tabs_image_default',
        'welcome_guide_tabs_image_perception',
        'welcome_guide_tabs_image_vehicle',
        'welcome_guide_decorate_ele',
        'welcome_guide_tabs_image_pnc',
    ]);

    const [hmi, dispatch] = usePickHmiStore();

    const { classes } = useStyle();

    const { t } = useTranslation('guide');

    const { mainApi, streamApi } = useWebSocketServices();

    const [userMixInfo] = useUserInfoStore();

    const [activeWelcomeGuideItemKey, setActiveWelcomeGuideItemKey] = useState(CURRENT_MODE.DEFAULT);

    const clickViewLoginSteps = () => {
        showViewLoginModal();
    };

    const clickWelcomeGuideTabs = (activeKey: string) => {
        setActiveWelcomeGuideItemKey(activeKey as CURRENT_MODE);
    };

    const clickEnterThisMode = () => {
        dispatch(changeMode(mainApi, activeWelcomeGuideItemKey as CURRENT_MODE));
        setLocalFirstUseGuideMode(activeWelcomeGuideItemKey);
        const subscription = streamApi?.subscribeToData(StreamDataNames.HMI_STATUS).subscribe((data: any) => {
            if (data?.currentMode === activeWelcomeGuideItemKey) {
                clickEnterThisModeToGuide(activeWelcomeGuideItemKey);
                subscription.unsubscribe();
            }
        });
    };

    const WelcomeGuideTabsItem = [
        {
            key: CURRENT_MODE.DEFAULT,
            label: 'Default Mode',
            children: (
                <WelcomeGuideTabsContent
                    contentImageUrl={welcomeGuideTabsImageDefault}
                    clickEnterThisMode={clickEnterThisMode}
                    currentMode={CURRENT_MODE.DEFAULT}
                />
            ),
        },
        {
            key: CURRENT_MODE.PERCEPTION,
            label: 'Perception Mode',
            children: (
                <WelcomeGuideTabsContent
                    contentImageUrl={welcomeGuideTabsImagePerception}
                    clickEnterThisMode={clickEnterThisMode}
                    currentMode={CURRENT_MODE.PERCEPTION}
                />
            ),
        },
        {
            key: CURRENT_MODE.PNC,
            label: 'PNC Mode',
            children: (
                <WelcomeGuideTabsContent
                    contentImageUrl={welcomeGuideTabsImagePNC}
                    clickEnterThisMode={clickEnterThisMode}
                    currentMode={CURRENT_MODE.PNC}
                />
            ),
        },
        {
            key: CURRENT_MODE.VEHICLE_TEST,
            label: 'Vehicle Test Mode',
            children: (
                <WelcomeGuideTabsContent
                    contentImageUrl={welcomeGuideTabsImageVehicle}
                    clickEnterThisMode={clickEnterThisMode}
                    currentMode={CURRENT_MODE.VEHICLE_TEST}
                />
            ),
        },
    ];

    return (
        <div className={classes['welcome-guide-container']}>
            <BackgroundEffect />
            <div className={classes['welcome-guide-content']}>
                <div className={classes['welcome-guide-head']}>
                    <div className={classes['welcome-guide-head-text']}>
                        <div
                            className={classes['welcome-guide-head-text-name']}
                            style={{ backgroundImage: `url(${welcomeGuideDecorateEle})` }}
                        >
                            <span>{t('welcome')}</span>
                            <span>{userMixInfo.isLogin ? '~ ' : ', '}</span>
                            <span>
                                {userMixInfo.isLogin ? (
                                    `${userMixInfo.userInfo.displayname}`
                                ) : (
                                    <span
                                        className={classes['welcome-guide-head-text-name-no-login']}
                                        onClick={clickViewLoginSteps}
                                    >
                                        {t('viewLoginSteps')}
                                    </span>
                                )}
                            </span>
                        </div>
                        <div className={classes['welcome-guide-head-text-desc']}>{t('modeSelectDesc')}</div>
                    </div>
                    <div className={classes['welcome-guide-head-logo']} />
                </div>
                <div className={classes['welcome-guide-tabs']}>
                    <Tabs
                        items={WelcomeGuideTabsItem}
                        onTabClick={clickWelcomeGuideTabs}
                        defaultActiveKey={activeWelcomeGuideItemKey}
                        rootClassName={classes['welcome-guide-tabs-class']}
                    />
                </div>
            </div>
        </div>
    );
}

export default React.memo(WelcomeGuide);
