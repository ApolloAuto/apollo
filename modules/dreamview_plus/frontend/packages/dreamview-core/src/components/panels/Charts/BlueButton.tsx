import React, { PropsWithChildren } from 'react';
import { makeStyles } from '@dreamview/dreamview-theme';
import noop from 'lodash/noop';
import tinycolor from 'tinycolor2';

const useStyle = makeStyles((theme) => ({
    'color-button': {
        cursor: 'pointer',
        display: 'inline-flex',
        outline: 'none',
        backgroundColor: theme.tokens.backgroundColor.transparent,
        color: theme.tokens.font.color.primary,
        border: `1px solid ${theme.tokens.font.color.primary}`,
        borderRadius: '8px',
        padding: '6px 16px',
        height: '32px',

        alignItems: 'center',

        ...theme.tokens.typography.content,
        '&:hover': {
            borderColor: '#307BE0',
        },
        '&:active': {
            borderColor: tinycolor(theme.tokens.font.color.primary).setAlpha(0.7).toRgbString(),
            color: tinycolor(theme.tokens.font.color.primary).setAlpha(0.7).toRgbString(),
        },
        '& > .anticon': {
            marginRight: '6px',
            fontSize: '16px',
        },
    },
    small: {
        padding: '4px 16px',
        height: '28px',
        fontSize: theme.tokens.font.size.sm,
        borderRadius: '6px',
    },
}));
interface BlueButtonProps {
    className?: string;
    onClick?: () => void;
    size?: 'small' | 'normal' | 'large';
}

function BlueButton(props: PropsWithChildren<BlueButtonProps>) {
    const { className, onClick = noop, size = 'normal' } = props;
    const { cx, classes } = useStyle();

    return (
        <button
            onClick={onClick}
            className={cx(classes['color-button'], { [classes.small]: size === 'small' }, className)}
            type='button'
        >
            {props.children}
        </button>
    );
}

export default React.memo(BlueButton);
