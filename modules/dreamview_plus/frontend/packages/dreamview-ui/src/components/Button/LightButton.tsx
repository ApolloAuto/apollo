import React, { PropsWithChildren } from 'react';
import { makeStyles, makeStylesWithProps } from '@dreamview/dreamview-theme';
import { OperatePopover } from '@dreamview/dreamview-ui/src/components/Popover/OperatePopover';

interface IButton {
    width?: number;
    onClick?: () => void;
    disabled?: boolean;
    className?: string;
    disabledText?: string;
}

const useStyle = makeStylesWithProps<{ width: number }>()((theme, props) => ({
    'light-button': {
        outline: 'none',
        border: 'none',
        margin: '0',
        width: props.width ? `${props.width}px` : 'auto',
        height: '32px',
        lineHeight: '32px',
        textAlign: 'center',
        borderRadius: '6px',
        padding: '0 20px',
        cursor: 'pointer',
        background: theme.components.lightButton.background,
        ...theme.tokens.typography.sideText,
        color: theme.components.lightButton.color,
        '&:hover': {
            color: theme.components.lightButton.colorHover,
            background: theme.components.lightButton.backgroundHover,
        },
        '&:active': {
            color: theme.components.lightButton.colorActive,
            background: theme.components.lightButton.backgroundActive,
        },
    },
    'light-button-disabled': {
        cursor: 'not-allowed',
        color: theme.components.lightButton.colorDisabled,
        background: theme.components.lightButton.backgroundDisabled,
        '&:hover': {
            color: theme.components.lightButton.colorDisabled,
            background: theme.components.lightButton.backgroundDisabled,
        },
        '&:active': {
            color: theme.components.lightButton.colorDisabled,
            background: theme.components.lightButton.backgroundDisabled,
        },
    },
}));
function Button(props: PropsWithChildren<IButton>) {
    const { disabled, width, onClick: propOnClick = () => true, className, disabledText } = props;
    const { classes, cx } = useStyle({ width });

    const onClick = () => {
        if (disabled) {
            return;
        }
        propOnClick();
    };

    if (disabled && disabledText) {
        <OperatePopover trigger='hover' content={disabledText}>
            <button
                onClick={onClick}
                type='button'
                className={cx(classes['light-button'], className, { [classes['light-button-disabled']]: disabled })}
            >
                {props.children}
            </button>
        </OperatePopover>;
    }

    return (
        <button
            onClick={onClick}
            type='button'
            className={cx(classes['light-button'], className, { [classes['light-button-disabled']]: disabled })}
        >
            {props.children}
        </button>
    );
}

export const LightButton = React.memo(Button);
