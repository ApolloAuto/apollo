import React, { useEffect, useRef } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useStyle, DynamicEffectButtonStatus } from './useStyle';

export * from './useStyle';

interface ButtonStatusType {
    [DynamicEffectButtonStatus.DISABLE]?: {
        text: string;
        icon?: React.ReactNode;
        disabledMsg: string;
        clickHandler?: () => void;
    };
    [DynamicEffectButtonStatus.RUNNING]?: {
        text: string;
        icon?: React.ReactNode;
        clickHandler?: () => void;
    };
    [DynamicEffectButtonStatus.START]?: {
        text: string;
        icon?: React.ReactNode;
        clickHandler?: () => void;
    };
    [DynamicEffectButtonStatus.STOP]?: {
        text: string;
        icon?: React.ReactNode;
        clickHandler?: () => void;
    };
}

export interface IDynamicEffectButton {
    behavior: ButtonStatusType;
    status: DynamicEffectButtonStatus;
}

const ua = navigator.userAgent.toLocaleLowerCase();
const needLowerPerformance = /linux/.test(ua) || /unbuntu/.test(ua);

function DynamicEffectButton(props: IDynamicEffectButton) {
    const { behavior, status } = props;

    const { classes, cx } = useStyle();

    const buttonBehavior = behavior[status];

    const icon =
        buttonBehavior?.icon ||
        {
            [DynamicEffectButtonStatus.DISABLE]: <IconPark name='IcNotApplicable' />,
            [DynamicEffectButtonStatus.START]: <IconPark name='IcNotApplicable' />,
            [DynamicEffectButtonStatus.RUNNING]: <IconPark name='IcNotApplicable' />,
            [DynamicEffectButtonStatus.STOP]: <IconPark name='IcOverUsable' />,
        }[status];

    const onClick = () => {
        if (buttonBehavior.clickHandler) {
            buttonBehavior.clickHandler();
        }
    };

    const dom = useRef<any>();
    const timer = useRef<number>(0);
    useEffect(() => {
        clearInterval(timer.current);
        if (status === DynamicEffectButtonStatus.RUNNING && !needLowerPerformance) {
            let deg = 0;
            timer.current = window.setInterval(() => {
                deg += 3;
                if (deg > 360) {
                    deg -= 360;
                }
                if (dom.current) {
                    dom.current.style = `background: linear-gradient(${deg}deg, #8dd0ff,#3288FA)`;
                }
            }, 17);
        }
        return () => {
            clearInterval(timer.current);
        };
    }, [status]);

    if (!buttonBehavior) {
        return null;
    }

    if (status === DynamicEffectButtonStatus.DISABLE) {
        return (
            <Popover
                trigger='hover'
                content={(buttonBehavior as ButtonStatusType[DynamicEffectButtonStatus.DISABLE]).disabledMsg}
            >
                <div className={cx(classes['btn-container'], classes['btn-disabled'])}>
                    <span>{icon}</span>
                    <span>{buttonBehavior.text}</span>
                </div>
            </Popover>
        );
    }

    if (status === DynamicEffectButtonStatus.RUNNING) {
        return (
            <div
                onClick={onClick}
                className={cx(classes['btn-container'], classes['btn-doing'])}
                id='guide-auto-drive-bar'
            >
                <div
                    ref={dom}
                    className={cx({
                        [classes['btn-border']]: !needLowerPerformance,
                    })}
                />
                <div className={classes['btn-ripple']} />
                <span>{icon}</span>
                <span>{buttonBehavior.text}</span>
                <div className={classes['btn-running-image']} />
            </div>
        );
    }

    if (status === DynamicEffectButtonStatus.START) {
        return (
            <div
                onClick={onClick}
                className={cx(classes['btn-container'], classes['btn-reactive'], classes['btn-start'])}
                id='guide-auto-drive-bar'
            >
                <span>{icon}</span>
                <span>{buttonBehavior.text}</span>
            </div>
        );
    }

    if (status === DynamicEffectButtonStatus.STOP) {
        return (
            <div
                onClick={onClick}
                className={cx(classes['btn-container'], classes['btn-stop'])}
                id='guide-auto-drive-bar'
            >
                <span>{icon}</span>
                <span>{buttonBehavior.text}</span>
            </div>
        );
    }

    return null;
}

export const DynamicEffectButtonMemo = React.memo(DynamicEffectButton);
