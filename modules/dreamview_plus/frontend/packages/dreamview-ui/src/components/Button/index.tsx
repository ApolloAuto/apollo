/* eslint-disable react/function-component-definition */
/* eslint-disable jsx-a11y/no-static-element-interactions */
/* eslint-disable jsx-a11y/click-events-have-key-events */
/* eslint-disable react/display-name */
import React, { createRef, forwardRef, useEffect, useMemo, useState } from 'react';
import PropTypes from 'prop-types';
import classNames from 'classnames';
import { composeRef } from 'rc-util/lib/ref';
import omit from 'rc-util/lib/omit';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconWrapper from './IconWrapper';
import IconPark from '../../IconPark';
import showWaveEffect from '../Wave/WaveEffect';

export type SizeType = 'small' | 'middle' | 'large' | undefined;

export type DirectionType = 'ltr' | 'rtl' | undefined;

export type ButtonTypes = 'default' | 'primary' | 'dashed' | 'link' | 'text' | undefined;

export type ButtonShapes = 'default' | 'circle' | 'round' | undefined;

export interface BaseButtonProps {
    type?: ButtonTypes;
    icon?: React.ReactNode;
    shape?: ButtonShapes;
    size?: SizeType;
    disabled?: boolean;
    loading?: boolean | { delay?: number };
    prefixCls?: string;
    className?: string;
    rootClassName?: string;
    ghost?: boolean;
    danger?: boolean;
    block?: boolean;
    children?: React.ReactNode;
    [key: `data-${string}`]: string;
    classNames?: { icon: string };
    styles?: { icon: React.CSSProperties };
    direction?: DirectionType;
}

export type AnchorButtonProps = {
    href: string;
    target?: string;
    onClick?: React.MouseEventHandler<HTMLAnchorElement>;
} & BaseButtonProps &
    Omit<React.AnchorHTMLAttributes<HTMLAnchorElement | HTMLButtonElement>, 'type' | 'onClick'>;

export type NativeButtonProps = {
    htmlType?: 'submit' | 'reset' | 'button' | undefined;
    onClick?: React.MouseEventHandler<HTMLButtonElement>;
} & BaseButtonProps &
    Omit<React.ButtonHTMLAttributes<HTMLButtonElement>, 'type' | 'onClick'>;

export type ButtonProps = Partial<AnchorButtonProps & NativeButtonProps>;

type LoadingType = {
    loading: boolean;
    delay: number;
};

export const getLoadingOrDelay = (loading: BaseButtonProps['loading']): LoadingType => {
    if (typeof loading === 'object' && loading) {
        const delay = loading?.delay;
        const isDelay = !Number.isNaN(delay) && typeof delay === 'number';
        return {
            loading: false,
            delay: isDelay ? delay : 0,
        };
    }

    return {
        loading: !!loading,
        delay: 0,
    };
};

const InternalButton: React.ForwardRefRenderFunction<HTMLButtonElement | HTMLAnchorElement, ButtonProps> = (
    props,
    ref,
) => {
    const {
        loading = false,
        prefixCls: customizePrefixCls,
        type = 'default',
        danger,
        shape = 'default',
        size: customizeSize,
        styles,
        disabled: customDisabled,
        className,
        rootClassName,
        children,
        icon,
        ghost = false,
        block = false,
        classNames: customClassNames,
        htmlType = 'button',
        direction = 'ltr',
        ...rest
    } = props;

    const prefixCls = getPrefixCls('btn', customizePrefixCls);

    const [disabled, setDisabled] = useState(false);
    const mergedDisabled = customDisabled ?? disabled;

    const loadingOrDelay = useMemo(() => getLoadingOrDelay(loading), [loading]);

    const [innerLoading, setLoading] = useState(loadingOrDelay.loading);

    useEffect(() => {
        let delayTimer: any = null;
        if (loadingOrDelay.delay > 0) {
            delayTimer = setTimeout(() => {
                delayTimer = null;
                setLoading(true);
            }, loadingOrDelay.delay);
        } else {
            setLoading(loadingOrDelay.loading);
        }

        function cleanupTimer() {
            if (delayTimer) {
                clearTimeout(delayTimer);
                delayTimer = null;
            }
        }

        return cleanupTimer;
    }, [loadingOrDelay]);

    const internalRef = createRef<HTMLButtonElement | HTMLAnchorElement>();

    const buttonRef = composeRef(ref, internalRef);

    const sizeCls = customizeSize || 'middle';

    const linkButtonRestProps = omit(rest as ButtonProps & { navigate: any }, ['navigate']);

    const classes = classNames(
        prefixCls,
        {
            [`${prefixCls}-${shape}`]: shape !== 'default' && shape,
            [`${prefixCls}-${type}`]: type,
            [`${prefixCls}-${sizeCls}`]: sizeCls,
            [`${prefixCls}-loading`]: innerLoading,
            [`${prefixCls}-block`]: block,
            [`${prefixCls}-dangerous`]: !!danger,
            [`${prefixCls}-rtl`]: direction === 'rtl',
            [`${prefixCls}-disabled`]: mergedDisabled,
        },
        className,
        rootClassName,
    );

    const internalIconLoading = innerLoading ? <IconPark name='IcLoading' spin /> : undefined;

    const iconNode =
        icon && !innerLoading ? (
            <IconWrapper prefixCls={prefixCls} className={customClassNames?.icon} style={styles?.icon}>
                {icon}
            </IconWrapper>
        ) : (
            internalIconLoading
        );

    const handleClick = (e: React.MouseEvent<HTMLButtonElement | HTMLAnchorElement, MouseEvent>) => {
        const { onClick } = props;
        // showWaveEffect((buttonRef as any).current, `${prefixCls}-wave`);
        if (innerLoading || mergedDisabled) {
            e.preventDefault();
            return;
        }
        (onClick as React.MouseEventHandler<HTMLButtonElement | HTMLAnchorElement>)?.(e);
    };

    if (linkButtonRestProps.href !== undefined) {
        return (
            <a
                {...linkButtonRestProps}
                className={classes}
                onClick={handleClick}
                ref={buttonRef as React.Ref<HTMLAnchorElement>}
            >
                {iconNode}
                {children}
            </a>
        );
    }

    const buttonNode = (
        <button
            {...(rest as NativeButtonProps)}
            // eslint-disable-next-line react/button-has-type
            type={htmlType}
            className={classes}
            onClick={handleClick}
            disabled={mergedDisabled}
            ref={buttonRef as React.Ref<HTMLButtonElement>}
        >
            {iconNode}
            {children}
        </button>
    );

    return buttonNode;
};

export const Button = forwardRef<HTMLButtonElement | HTMLAnchorElement, ButtonProps>(InternalButton);

Button.propTypes = {
    /**
     * Is this the principal call to action on the page?
     */
    type: PropTypes.oneOf(['default', 'primary', 'link']),
    /**
     * How large should the button be?
     */
    size: PropTypes.oneOf(['small', 'middle', 'large']),
    /**
     * Optional click handler
     */
    onClick: PropTypes.func,
};

Button.defaultProps = {
    type: 'primary',
    size: 'middle',
    onClick: () => {
        console.log('clicked');
    },
    children: '点击',
    shape: 'default',
    loading: false,
    disabled: false,
    danger: false,
};

Button.displayName = 'Button';
