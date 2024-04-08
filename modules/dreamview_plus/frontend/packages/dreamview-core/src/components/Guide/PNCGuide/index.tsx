import React, { Fragment, useEffect, useState } from 'react';
import Joyride, { Placement } from 'react-joyride';
import { Button } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { TFunction } from 'i18next';
import { ChangeEnviormentResourcesAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { ENUM_ENVIORMENT_MANAGER_TAB_KEY } from '@dreamview/dreamview-core/src/store/MenuStore';
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
            <span className={classes['dv-pnc-guide-skip']}>{t('skip')}</span>
            <span className={classes['dv-pnc-guide-skip']}>(</span>
            <span className={classes['dv-pnc-guide-current-value']}>{currentStep + 1}</span>
            <span className={classes['dv-pnc-guide-skip']}>/</span>
            <span className={classes['dv-pnc-guide-all-value']}>{isLogin ? 8 : 7}</span>
            <span className={classes['dv-pnc-guide-skip']}>)</span>
        </span>
    );
}

export function BackNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-pnc-guide-back-node']}>{t('back')}</Button>;
}

export function NextNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-pnc-guide-next-node']}>{t('next')}</Button>;
}

export function CloseNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-pnc-guide-close-node']}>{t('close')}</Button>;
}

const steps = (isLogin: boolean, currentStep: number, translation: TFunction) =>
    [
        {
            title: translation('PNCSelectOperations'),
            target: '#guide-modesettings-operations',
            content: translation('PNCSelectOperationsDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSelectModules'),
            target: '#guide-modesettings-modules',
            content: translation('PNCSelectModulesDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCResourceManager'),
            target: '.pnc-guide-profile-manager-start',
            content: translation('PNCResourceManagerDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSelectScenario'),
            target: '#guide-modesettings-variable',
            content: translation('PNCSelectScenarioDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSelectMap'),
            target: '#guide-modesettings-variable',
            content: translation('PNCSelectMapDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSelectADS'),
            target: '#guide-modesettings-fixed',
            content: translation('PNCSelectADSDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSetRoute'),
            target: '#routing-editing-btn',
            content: translation('PNCSetRouteDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
        {
            title: translation('PNCSimulationRecord'),
            target: '#guide-simulation-record',
            content: translation('PNCSimulationRecordDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                last: CloseNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 5,
        },
    ].filter((item, index) => {
        if (!isLogin && index === 2) {
            return false;
        }
        return true;
    });

function PNCGuide() {
    const { t } = useTranslation('guide');

    const [{ activeMenu }, dispatch] = useMenuStore();

    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    const { classes, cx, css, theme } = useStyle();

    const [run, setRun] = useState(false);

    const [stepIndexState, setStepIndexState] = useState(0);

    const [toolTipValue, setToolTipValue] = useState(() => tooltip({ isLogin, theme, currentIndex: stepIndexState }));

    const [profileManageGuideStatus, setProfileManageGuideStatus] = useState(profileManageGuideStartValue);

    const operationGuide = (operationGuideProps: any) => {
        // eslint-disable-next-line react/prop-types
        const { action, index, lifecycle, status } = operationGuideProps;

        if (isLogin) {
            // 登陆状态下 MenuDrawer变更和Resource引导逻辑
            if (
                (action === 'next' && index === 1 && lifecycle === 'complete') ||
                (action === 'prev' && index === 3 && lifecycle === 'complete')
            ) {
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
                setProfileManageGuideStatus(profileManageGuideActiveValue);
            } else if (
                (action === 'next' && index === 2 && lifecycle === 'complete') ||
                (action === 'prev' && index === 2 && lifecycle === 'complete')
            ) {
                setRun(false);
                setProfileManageGuideStatus(profileManageGuideStartValue);
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.MODE_SETTING));
                setTimeout(() => {
                    setRun(true);
                }, 500);
            }

            if (action === 'next' && index === 2 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.SCENARIO));
            }
            if (action === 'next' && index === 3 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP));
            }
            if (action === 'prev' && index === 4 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.SCENARIO));
            }
        }

        if (!isLogin) {
            if (action === 'next' && index === 1 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.SCENARIO));
            }
            if (action === 'next' && index === 2 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP));
            }
            if (action === 'prev' && index === 3 && lifecycle === 'complete') {
                dispatch(ChangeEnviormentResourcesAction(ENUM_ENVIORMENT_MANAGER_TAB_KEY.SCENARIO));
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
            <div className={cx('pnc-guide-profile-manager-start', css(profileManageGuideStatus))} />
            <Joyride
                run={run}
                steps={steps(isLogin, stepIndexState, t)}
                continuous
                showSkipButton
                hideCloseButton
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

export default React.memo(PNCGuide);
