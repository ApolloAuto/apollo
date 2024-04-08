import React, { useEffect, useState } from 'react';
import Joyride, { Placement } from 'react-joyride';
import { Button } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { TFunction } from 'i18next';
import { useMenuStore, ENUM_MENU_KEY } from '../../../store/MenuStore';
import { UpdateMenuAction } from '../../../store/MenuStore/actions';
import useStyle from './useStyle';
import { useUserInfoStore } from '../../../store/UserInfoStore';
import { HMIModeOperation } from '../../../store/HmiStore';

export enum VehicleGuideStepName {
    MODULES = 'MODULES',
    OPERATIONS = 'OPERATIONS',
    RE_MANAGER = 'RE_MANAGER',
    ADS_RESOURCES = 'ADS_RESOURCES',
    ENV_RESOURCES = 'ENV_RESOURCES',
    ROUTING_EDITING = 'ROUTING_EDITING',
    START_AUTODRIVE = 'START_AUTODRIVE',
}

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

export function SkipNode(props: { currentStep: number; currentOperation: string }) {
    const { currentStep, currentOperation } = props;
    const { classes } = useStyle();
    const { t } = useTranslation('guide');
    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    const completeStepsNum = () => {
        if (isLogin) {
            if (currentOperation === HMIModeOperation.PLAY_RECORDER) {
                return 6;
            }
            return 7;
        }
        if (currentOperation === HMIModeOperation.PLAY_RECORDER) {
            return 5;
        }
        return 6;
    };

    return (
        <span>
            <span className={classes['dv-vehicle-guide-skip']}>{t('skip')}</span>
            <span className={classes['dv-vehicle-guide-skip']}>(</span>
            <span className={classes['dv-vehicle-guide-current-value']}>{currentStep + 1}</span>
            <span className={classes['dv-vehicle-guide-skip']}>/</span>
            <span className={classes['dv-vehicle-guide-all-value']}>{completeStepsNum()}</span>
            <span className={classes['dv-vehicle-guide-skip']}>)</span>
        </span>
    );
}

export function BackNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-vehicle-guide-back-node']}>{t('back')}</Button>;
}

export function NextNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-vehicle-guide-next-node']}>{t('next')}</Button>;
}

export function CloseNode() {
    const { t } = useTranslation('guide');
    const { classes, theme } = useStyle();
    return <Button className={classes['dv-vehicle-guide-close-node']}>{t('close')}</Button>;
}

const steps = (isLogin: boolean, currentStep: number, translation: TFunction, currentOperation: string) =>
    [
        {
            name: VehicleGuideStepName.MODULES,
            title: translation('VehicleSelectModules'),
            target: '#guide-modesettings-modules',
            content: translation('VehicleSelectModulesDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                next: NextNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.OPERATIONS,
            title: translation('VehicleSelectOperations'),
            target: '#guide-modesettings-operations',
            content: translation('VehicleSelectOperationsDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.RE_MANAGER,
            title: translation('VehicleResourceManager'),
            target: '.vehicle-guide-profile-manager-start',
            content: translation('VehicleResourceManagerDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.ADS_RESOURCES,
            title: translation('VehicleSelectVehicle'),
            target: '#guide-modesettings-fixed',
            content: translation('VehicleSelectVehicleDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.ENV_RESOURCES,
            title: translation('VehicleSelectMap'),
            target: '#guide-modesettings-variable',
            content: translation('VehicleSelectMapDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                next: NextNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.ROUTING_EDITING,
            title: translation('VehicleRoutingEditing'),
            target: '#routing-editing-btn',
            content: translation('VehicleRoutingEditingDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                [currentOperation !== HMIModeOperation.PLAY_RECORDER ? 'next' : 'last']:
                    currentOperation !== HMIModeOperation.PLAY_RECORDER ? NextNode() : CloseNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
        {
            name: VehicleGuideStepName.START_AUTODRIVE,
            title: translation('VehicleStartAutoDrive'),
            target: '#guide-auto-drive-bar',
            content: translation('VehicleStartAutoDriveDesc'),
            placement: 'right' as const,
            disableBeacon: true,
            locale: {
                back: BackNode(),
                last: CloseNode(),
                skip: SkipNode({ currentStep, currentOperation }),
            },
            offset: 5,
        },
    ].filter((item) => {
        if (!isLogin && item.name === VehicleGuideStepName.RE_MANAGER) {
            return false;
        }
        if (currentOperation === HMIModeOperation.PLAY_RECORDER && item.name === VehicleGuideStepName.START_AUTODRIVE) {
            return false;
        }
        return true;
    });

function VehicleGuide(props: { currentOperation: string }) {
    const { currentOperation } = props;

    const { t } = useTranslation('guide');

    const [{ activeMenu }, dispatch] = useMenuStore();

    const [{ isLogin }, dispatchUserInfo] = useUserInfoStore();

    const { classes, cx, css, theme } = useStyle();

    const [run, setRun] = useState(false);

    const [stepIndexState, setStepIndexState] = useState(0);

    const [profileManageGuideStatus, setProfileManageGuideStatus] = useState(profileManageGuideStartValue);

    const [toolTipValue, setToolTipValue] = useState(() => tooltip({ isLogin, theme, currentIndex: stepIndexState }));

    const operationGuide = (operationGuideProps: any) => {
        // eslint-disable-next-line react/prop-types
        const { action, index, lifecycle, status } = operationGuideProps;
        const { name } = operationGuideProps.step;

        if (isLogin) {
            // 登陆状态下 MenuDrawer变更和Resource引导逻辑
            if (
                (action === 'next' && name === VehicleGuideStepName.OPERATIONS && lifecycle === 'complete') ||
                (action === 'prev' && name === VehicleGuideStepName.ADS_RESOURCES && lifecycle === 'complete')
            ) {
                dispatch(UpdateMenuAction(ENUM_MENU_KEY.PROFILE_MANAGEER));
                setProfileManageGuideStatus(profileManageGuideActiveValue);
            } else if (
                (action === 'next' && name === VehicleGuideStepName.RE_MANAGER && lifecycle === 'complete') ||
                (action === 'prev' && name === VehicleGuideStepName.RE_MANAGER && lifecycle === 'complete')
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
            <div className={cx('vehicle-guide-profile-manager-start', css(profileManageGuideStatus))} />
            <Joyride
                run={run}
                steps={steps(isLogin, stepIndexState, t, currentOperation)}
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

export default React.memo(VehicleGuide);
