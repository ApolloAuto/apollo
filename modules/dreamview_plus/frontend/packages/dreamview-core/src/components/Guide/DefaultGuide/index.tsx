import React, { Fragment, useEffect, useState } from 'react';
import Joyride, { Placement } from 'react-joyride';
import { Button } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { TFunction } from 'i18next';
import { useMenuStore, ENUM_MENU_KEY } from '../../../store/MenuStore';
import { UpdateMenuAction } from '../../../store/MenuStore/actions';
import useStyle from './useStyle';
import { useUserInfoStore } from '../../../store/UserInfoStore';

function tooltip(props: { isLogin: boolean; theme: any; currentIndex: number }) {
    const { isLogin, theme, currentIndex } = props;
    const tooltipValue = {
        tooltip: {
            width: '494px',
            padding: '0px',
            borderRadius: '10px',
            backgroundColor: theme.tokens.colors.background2,
        },
        tooltipTitle: {
            ...theme.tokens.typography.title,
            color: '#FFFFFF',
            height: '50px',
            lineHeight: '50px',
            textAlign: 'left',
            paddingLeft: '24px',
            borderBottom: '1px solid #383C4D',
        },
        tooltipContent: {
            ...theme.tokens.typography.content,
            color: theme.tokens.colors.fontColor5,
            padding: '16px 24px 24px 24px',
            textAlign: 'left',
        },
        tooltipFooter: {
            height: '72px',
            margin: '0px',
            display: 'flex',
            padding: '0px 24px 30px 24px',
        },
        options: {
            arrowColor: theme.tokens.colors.background2,
        },
    };
    return tooltipValue;
}
// const modeSettingsGuideStartValue = {
//     width: '0px',
//     height: '0px',
// };
// const modeSettingsGuideActiveValue = {
//     width: '370px',
//     height: 'auto',
//     position: 'fixed',
//     top: '20px',
//     left: '63px',
//     bottom: '63px',
//     borderRadius: '12px',
// };
const profileManageGuideStartValue = {
    width: '0px',
    height: '0px',
};
const profileManageGuideActiveValue = {
    width: '589px',
    height: '146px',
    position: 'fixed',
    top: '15px',
    left: '75px',
};

export function SkipNode(props: { currentStep: number }) {
    const { currentStep } = props;
    const { classes } = useStyle();
    const { t } = useTranslation('guide');
    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    return (
        <span>
            <span className={classes['dv-default-guide-skip']}>{t('skip')}</span>
            <span className={classes['dv-default-guide-skip']}>(</span>
            <span className={classes['dv-default-guide-current-value']}>{currentStep + 1}</span>
            <span className={classes['dv-default-guide-skip']}>/</span>
            <span className={classes['dv-default-guide-all-value']}>{isLogin ? 6 : 5}</span>
            <span className={classes['dv-default-guide-skip']}>)</span>
        </span>
    );
}

export function BackNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-default-guide-back-node']}>{t('back')}</Button>;
}

export function NextNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-default-guide-next-node']}>{t('next')}</Button>;
}

export function CloseNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-default-guide-close-node']}>{t('close')}</Button>;
}

const steps = (isLogin: boolean, currentStep: number, translation: TFunction) =>
    [
        {
            title: translation('defaultSelectMode'),
            target: '#guide-modesettings-mode',
            content: translation('defaultSelectModeDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('defaultSelectModule'),
            target: '#guide-modesettings-modules',
            content: translation('defaultSelectModuleDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('defaultSelectOperations'),
            target: '#guide-modesettings-operations',
            content: translation('defaultSelectOperationsDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('defaultResourceManager'),
            target: '.default-guide-profile-manager-start',
            content: translation('defaultResourceManagerDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('defaultSelectVariableRes'),
            target: '#guide-modesettings-variable',
            content: translation('defaultSelectVariableResDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('defaultSelectFixedRes'),
            target: '#guide-modesettings-fixed',
            content: translation('defaultSelectFixedResDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                last: CloseNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
    ].filter((item, index) => {
        if (!isLogin && index === 3) {
            return false;
        }
        return true;
    });

function DefaultGuide() {
    const { t } = useTranslation('guide');

    const [{ activeMenu }, dispatch] = useMenuStore();

    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    const { classes, cx, css, theme } = useStyle();

    const [run, setRun] = useState(false);

    const [stepIndexState, setStepIndexState] = useState(0);

    const [toolTipValue, setToolTipValue] = useState(() => tooltip({ isLogin, theme, currentIndex: stepIndexState }));

    // const [modeSettingsGuideStatus, setModeSettingsGuideStatus] = useState(modeSettingsGuideStartValue);

    const [profileManageGuideStatus, setProfileManageGuideStatus] = useState(profileManageGuideStartValue);

    const operationGuide = (operationGuideProps: any) => {
        // eslint-disable-next-line react/prop-types
        const { action, index, lifecycle, status } = operationGuideProps;

        // 判断modules引导逻辑
        // if (
        //     (action === 'start' && index === 0 && lifecycle === 'init') ||
        //     (action === 'prev' && index === 1 && lifecycle === 'complete')
        // ) {
        //     setModeSettingsGuideStatus(modeSettingsGuideActiveValue);
        // } else if (action === 'next' && index === 0 && lifecycle === 'complete') {
        //     setModeSettingsGuideStatus(modeSettingsGuideStartValue);
        // }

        if (isLogin) {
            // 登陆状态下 MenuDrawer变更和Resource引导逻辑
            if (
                (action === 'next' && index === 2 && lifecycle === 'complete') ||
                (action === 'prev' && index === 4 && lifecycle === 'complete')
            ) {
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
                setProfileManageGuideStatus(profileManageGuideActiveValue);
            } else if (
                (action === 'next' && index === 3 && lifecycle === 'complete') ||
                (action === 'prev' && index === 3 && lifecycle === 'complete')
            ) {
                setRun(false);
                setProfileManageGuideStatus(profileManageGuideStartValue);
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.MODE_SETTING));
                setTimeout(() => {
                    setRun(true);
                }, 500);
            }
        }

        // 同步当前所处步骤
        if (action === 'update' && lifecycle === 'tooltip' && status === 'running') {
            setStepIndexState(index);
        }
    };

    useEffect(() => {
        setToolTipValue(() => tooltip({ isLogin, theme, currentIndex: stepIndexState }));
    }, [stepIndexState]);

    useEffect(() => {
        dispatch(UpdateMenuAction(ENUM_MENU_KEY.MODE_SETTING));
        setRun(true);
    }, []);

    const styles = {
        tooltip: toolTipValue.tooltip,
        tooltipTitle: toolTipValue.tooltipTitle,
        tooltipContent: toolTipValue.tooltipContent,
        tooltipFooter: toolTipValue.tooltipFooter,
        options: toolTipValue.options,
    };

    return (
        <>
            {/* <div className={cx('default-guide-modesettings', css(modeSettingsGuideStatus))} /> */}
            <div className={cx('default-guide-profile-manager-start', css(profileManageGuideStatus))} />
            <Joyride
                run={run}
                steps={steps(isLogin, stepIndexState, t)}
                continuous
                showSkipButton
                callback={operationGuide}
                styles={styles}
                scrollOffset={100}
                disableOverlayClose
                floaterProps={{
                    styles: {
                        arrow: {
                            length: 6,
                            spread: 12,
                        },
                    },
                }}
            />
        </>
    );
}

export default React.memo(DefaultGuide);
