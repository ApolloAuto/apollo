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
            width: (isLogin && currentIndex === 4) || (!isLogin && currentIndex === 3) ? '370px' : '494px',
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
    border: '1.5px solid red',
    borderRadius: '12px',
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
            <span className={classes['dv-perception-guide-skip']}>{t('skip')}</span>
            <span className={classes['dv-perception-guide-skip']}>(</span>
            <span className={classes['dv-perception-guide-current-value']}>{currentStep + 1}</span>
            <span className={classes['dv-perception-guide-skip']}>/</span>
            <span className={classes['dv-perception-guide-all-value']}>{isLogin ? 5 : 4}</span>
            <span className={classes['dv-perception-guide-skip']}>)</span>
        </span>
    );
}

export function BackNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-perception-guide-back-node']}>{t('back')}</Button>;
}

export function NextNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-perception-guide-next-node']}>{t('next')}</Button>;
}

export function CloseNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-perception-guide-close-node']}>{t('close')}</Button>;
}

const steps = (isLogin: boolean, currentStep: number, translation: TFunction) =>
    [
        {
            title: translation('perceptionSelectModule'),
            target: '#guide-modesettings-modules',
            content: translation('perceptionSelectModuleDesc'),
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
            title: translation('perceptionSelectOperations'),
            target: '#guide-modesettings-operations',
            content: translation('perceptionSelectOperationsDesc'),
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
            title: translation('perceptionResourceManager'),
            target: '.perception-guide-profile-manager-start',
            content: translation('perceptionResourceManagerDesc'),
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
            title: translation('perceptionSelectResources'),
            target: '#guide-modesettings-variable',
            content: translation('perceptionSelectResourcesDesc'),
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
            title: translation('perceptionPerceivedEffects'),
            target: '#perception-guide-perceived-effects',
            content: translation('perceptionPerceivedEffectsDesc'),
            placement: 'left-start' as const,
            disableBeacon: true,
            hideCloseButton: true,
            locale: {
                back: BackNode(),
                last: CloseNode(),
                skip: SkipNode({ currentStep }),
            },
            offset: 8,
        },
    ].filter((item, index) => {
        if (!isLogin && index === 2) {
            return false;
        }
        return true;
    });

type PerceptiontGuideProps = {
    controlPerceptionGuideLastStep: (perceptionGuideLastStepStyle: object) => void;
};

function PerceptiontGuide(props: PerceptiontGuideProps) {
    const { controlPerceptionGuideLastStep } = props;

    const { t } = useTranslation('guide');

    const [{ activeMenu }, dispatch] = useMenuStore();

    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    const { classes, cx, css, theme } = useStyle();

    const [run, setRun] = useState(false);

    const [disableScrollState, setDisableScrollState] = useState(true);

    const [stepIndexState, setStepIndexState] = useState(0);

    const [toolTipValue, setToolTipValue] = useState(() => tooltip({ isLogin, theme, currentIndex: stepIndexState }));

    const [profileManageGuideStatus, setProfileManageGuideStatus] = useState(profileManageGuideStartValue);

    const operationGuide = (operationGuideProps: any) => {
        // eslint-disable-next-line react/prop-types
        const { action, index, lifecycle, status } = operationGuideProps;

        // 防止页面抖动，刚进入页面时禁用元素滚动，待用户点击后启用页面滚动保证元素处于视图内
        if (action === 'next' && index === 0 && lifecycle === 'complete') {
            setDisableScrollState(false);
        }

        if (isLogin) {
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

            if (action === 'next' && index === 3 && lifecycle === 'complete') {
                controlPerceptionGuideLastStep({
                    position: 'absolute',
                    left: 10,
                    right: 20,
                    top: 15,
                    bottom: 20,
                });
            } else if (
                (action === 'next' && index === 4 && lifecycle === 'complete') ||
                (action === 'prev' && index === 4 && lifecycle === 'complete')
            ) {
                controlPerceptionGuideLastStep({});
            }
        }

        if (!isLogin) {
            if (action === 'next' && index === 2 && lifecycle === 'complete') {
                controlPerceptionGuideLastStep({
                    position: 'absolute',
                    left: 10,
                    right: 20,
                    top: 15,
                    bottom: 20,
                });
            } else if (
                (action === 'next' && index === 3 && lifecycle === 'complete') ||
                (action === 'prev' && index === 3 && lifecycle === 'complete')
            ) {
                controlPerceptionGuideLastStep({});
            }
        }

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
            <div className={cx('perception-guide-profile-manager-start', css(profileManageGuideStatus))} />
            <Joyride
                run={run}
                steps={steps(isLogin, stepIndexState, t)}
                continuous
                showSkipButton
                callback={operationGuide}
                styles={styles}
                disableOverlayClose
                scrollOffset={100}
                floaterProps={{
                    styles: {
                        arrow: {
                            length: 6,
                            spread: 12,
                        },
                    },
                }}
                disableScrolling={disableScrollState}
            />
        </>
    );
}

export default React.memo(PerceptiontGuide);
