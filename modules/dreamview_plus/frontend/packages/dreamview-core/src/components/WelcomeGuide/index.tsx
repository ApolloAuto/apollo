import React, { useState } from 'react';
import { Tabs, Button } from '@dreamview/dreamview-ui';
import welcomeGuideTabsImageDefault from '@dreamview/dreamview-core/src/assets/welcome_guide_tabs_image_default.png';
import welcomeGuideTabsImagePerception from '@dreamview/dreamview-core/src/assets/welcome_guide_tabs_image_perception.png';
import welcomeGuideDecorateEle from '@dreamview/dreamview-core/src/assets/welcome_guide_decorate_ele.png';
import welcomeGuideTabsImagePNC from '@dreamview/dreamview-core/src/assets/welcome_guide_tabs_image_pnc.png';
import useStyle from './useStyle';
import { useUserInfoStore } from '../../store/UserInfoStore';
import { CURRENT_MODE } from '../../store/HmiStore';
import showViewLoginModal from './showViewLoginModal';
import BackgroundEffect from './BackgroundEffect';

type WelcomeGuideTabsContentProps = {
    contentImageUrl: string;
    clickEnterThisMode: () => void;
};

function WelcomeGuideTabsContent(props: WelcomeGuideTabsContentProps) {
    const { contentImageUrl, clickEnterThisMode } = props;
    const { classes } = useStyle();

    return (
        <div className={classes['welcome-guide-content-tab']}>
            <div
                className={classes['welcome-guide-content-tab-image']}
                style={{ backgroundImage: `url(${contentImageUrl})` }}
            />
            <div className={classes['welcome-guide-content-tab-text']}>
                <div className={classes['welcome-guide-content-tab-text-desc']}>
                    The default mode follows the old version of Dreamview layout and is applicable to all scenarios
                    where debugging begins.
                </div>
                <div className={classes['welcome-guide-content-tab-text-modules-panel']}>
                    Modules:
                    <div className={classes['welcome-guide-content-tab-text-modules-panel-desc']}>
                        Include all modules
                    </div>
                </div>
                <div className={classes['welcome-guide-content-tab-text-modules-panel']}>
                    Panel：
                    <div className={classes['welcome-guide-content-tab-text-modules-panel-desc']}>
                        Vehicle Visualization、Vehicle Dashboard、Module delay、Console
                    </div>
                </div>
                <div className={classes['enter-this-mode']}>
                    <Button className={classes['enter-this-mode-btn']} onClick={() => clickEnterThisMode()}>
                        Enter this mode
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

    const { classes, theme } = useStyle();

    const [userMixInfo] = useUserInfoStore();

    const [activeWelcomeGuideItemKey, setActiveWelcomeGuideItemKey] = useState(CURRENT_MODE.DEFAULT);

    const clickViewLoginSteps = () => {
        showViewLoginModal();
    };

    const clickWelcomeGuideTabs = (activeKey: string) => {
        setActiveWelcomeGuideItemKey(activeKey as CURRENT_MODE);
    };

    const clickEnterThisMode = () => {
        setLocalFirstUseGuideMode(activeWelcomeGuideItemKey);
        clickEnterThisModeToGuide(activeWelcomeGuideItemKey);
    };

    const WelcomeGuideTabsItem = [
        {
            key: CURRENT_MODE.DEFAULT,
            label: 'Default Mode',
            children: (
                <WelcomeGuideTabsContent
                    contentImageUrl={welcomeGuideTabsImageDefault}
                    clickEnterThisMode={clickEnterThisMode}
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
                            <span>Welcome</span>
                            <span>{userMixInfo.isLogin ? '~ ' : ', '}</span>
                            <span>
                                {userMixInfo.isLogin ? (
                                    `${userMixInfo.userInfo.displayname}`
                                ) : (
                                    <span
                                        className={classes['welcome-guide-head-text-name-no-login']}
                                        onClick={clickViewLoginSteps}
                                    >
                                        View login steps
                                    </span>
                                )}
                            </span>
                        </div>
                        <div className={classes['welcome-guide-head-text-desc']}>
                            We provide you with the following visualization template. Choose one as the default
                            interface to open.
                        </div>
                    </div>
                    <div className={classes['welcome-guide-head-logo']} />
                </div>
                <div className={classes['welcome-guide-tabs']}>
                    <Tabs
                        items={WelcomeGuideTabsItem}
                        onTabClick={clickWelcomeGuideTabs}
                        defaultActiveKey={activeWelcomeGuideItemKey}
                        className={classes['welcome-guide-tabs-class']}
                    />
                </div>
            </div>
        </div>
    );
}

export default React.memo(WelcomeGuide);
