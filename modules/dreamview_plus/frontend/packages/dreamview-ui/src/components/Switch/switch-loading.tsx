import classNames from 'classnames';
import RcSwitch from 'rc-switch';
import React, { useState } from 'react';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import PropTypes from 'prop-types';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconPark from '../../IconPark';
// import './switch-loading.less';

export type SwitchSize = 'small' | 'default';
export type SwitchChangeEventHandler = (
    checked: boolean,
    event: React.MouseEvent<HTMLButtonElement> | React.KeyboardEvent<HTMLButtonElement>,
) => void;
export type SwitchClickEventHandler = SwitchChangeEventHandler;

export interface SwitchProps {
    prefixCls?: string;
    size?: SwitchSize;
    className?: string;
    rootClassName?: string;
    checked?: boolean;
    defaultChecked?: boolean;
    onChange?: SwitchChangeEventHandler;
    onClick?: SwitchClickEventHandler;
    checkedChildren?: React.ReactNode;
    unCheckedChildren?: React.ReactNode;
    disabled?: boolean;
    loading?: boolean;
    autoFocus?: boolean;
    style?: React.CSSProperties;
    title?: string;
    tabIndex?: number;
    id?: string;
}

const useStyle = makeStylesWithProps<{ classname: string }>()((theme, props) => ({
    [props.classname]: {
        '&.dreamview-switch-loading': {
            boxSizing: 'border-box' as any,
            margin: 0,
            padding: 0,
            color: 'rgba(0, 0, 0, 0.88)',
            fontSize: '14px',
            lineHeight: '12px',
            listStyle: 'none',
            fontFamily:
                "-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans',sans-serif,'Apple Color Emoji','Segoe UI Emoji','Segoe UI Symbol','Noto Color Emoji'",
            position: 'relative',
            display: 'block',
            minWidth: '26px',
            height: '12px',
            verticalAlign: 'middle',
            background: '#A0A3A7',
            border: 0,
            borderRadius: '3px',
            cursor: 'pointer',
            transition: 'all 0.2s',
            userSelect: 'none' as any,

            '&.dreamview-switch-loading.dreamview-switch-loading-checked': {
                background: '#055FE7',

                '& .dreamview-switch-loading-inner': {
                    '& .dreamview-switch-loading-inner-checked': {
                        marginInlineStart: 0,
                        marginInlineEnd: 0,
                    },

                    '& .dreamview-switch-loading-inner-unchecked': {
                        marginInlineStart: 'calc(100% - 22px + 48px)',
                        marginInlineEnd: 'calc(-100% + 22px - 48px)',
                    },
                },

                '& .dreamview-switch-loading-handle': {
                    insetInlineStart: 'calc(100% - 12px)',
                },

                '& .dreamview-switch-loading-handle::before': {
                    backgroundColor: '#fff',
                },
            },

            '& .dreamview-switch-loading-handle': {
                position: 'absolute',
                top: '1px',
                insetInlineStart: '2px',
                width: '10px',
                height: '10px',
                transition: 'all 0.2s ease-in-out',
            },

            '& .dreamview-switch-loading-handle::before': {
                position: 'absolute',
                top: 0,
                insetInlineEnd: 0,
                bottom: 0,
                insetInlineStart: 0,
                backgroundColor: 'white',
                borderRadius: '3px',
                boxShadow: '0 2px 4px 0 rgb(0 35 11 / 20%)',
                transition: 'all 0.2s ease-in-out',
                content: '""',
            },

            '& .dreamview-switch-loading-inner': {
                display: 'block',
                overflow: 'hidden',
                borderRadius: '100px',
                height: '100%',
                transition: 'padding-inline-start 0.2s ease-in-out,padding-inline-end 0.2s ease-in-out',

                '& .dreamview-switch-loading-inner-checked': {
                    display: 'block',
                    color: '#fff',
                    fontSize: '10px',
                    transition: 'margin-inline-start 0.2s ease-in-out,margin-inline-end 0.2s ease-in-out',
                    pointerEvents: 'none' as any,
                    marginInlineStart: 'calc(-100% + 22px - 48px)',
                    marginInlineEnd: 'calc(100% - 22px + 48px)',
                    paddingRight: '13px',
                    marginTop: '-1px',
                },

                '& .dreamview-switch-loading-inner-unchecked': {
                    display: 'block',
                    color: '#fff',
                    fontSize: '10px',
                    transition: 'margin-inline-start 0.2s ease-in-out,margin-inline-end 0.2s ease-in-out',
                    pointerEvents: 'none' as any,
                    marginTop: '-11px',
                    marginInlineStart: 0,
                    marginInlineEnd: 0,
                    paddingLeft: '14px',
                },
            },

            '&.dreamview-switch-loading-disabled': {
                cursor: 'not-allowed',
                opacity: 0.65,

                '& .dreamview-switch-loading-handle::before': {
                    display: 'none',
                },
            },
            '&.dreamview-switch-loading-loading': {
                cursor: 'not-allowed',
                opacity: 0.65,

                '& .dreamview-switch-loading-handle::before': {
                    display: 'none',
                },
            },

            '& .dreamview-switch-loading-loading-icon': {
                color: '#BDC3CD',
                fontSize: '12px',
            },
        },
    },
}));

export const SwitchLoading = React.forwardRef<HTMLButtonElement, SwitchProps>(
    (
        {
            prefixCls: customizePrefixCls,
            size: customizeSize,
            disabled: customDisabled,
            loading,
            className,
            rootClassName,
            ...props
        },
        ref,
    ) => {
        const [disabled, setDisabled] = useState(false);
        const mergedDisabled = (customDisabled ?? disabled) || loading;
        const prefixCls = getPrefixCls('switch-loading', customizePrefixCls);
        const loadingIcon = (
            <div className={`${prefixCls}-handle`}>
                {loading && <IconPark name='IcAmplification' spin className={`${prefixCls}-loading-icon`} />}
            </div>
        );

        const mergedSize = customizeSize || 'default';
        const { classes, cx } = useStyle({ classname: prefixCls });

        const cls = cx(
            {
                [`${prefixCls}-small`]: mergedSize === 'small',
                [`${prefixCls}-loading`]: loading,
            },
            classes[prefixCls],
            className,
            rootClassName,
        );

        return (
            <RcSwitch
                {...props}
                prefixCls={prefixCls}
                className={cls}
                disabled={mergedDisabled}
                ref={ref}
                loadingIcon={loadingIcon}
            />
        );
    },
);

SwitchLoading.propTypes = {
    /**
     * 决定Switch开关是否打开
     */
    checked: PropTypes.bool,
    /**
     * Switch开关默认是否打开
     */
    defaultChecked: PropTypes.bool,
    /**
     * 开关打开后显示的元素
     */
    checkedChildren: PropTypes.node,
    /**
     * 开关关闭后显示的元素
     */
    unCheckedChildren: PropTypes.node,
    /**
     * 是否禁用Switch开关
     */
    disabled: PropTypes.bool,
    /**
     * 点击回调
     */
    onClick: PropTypes.func,
    /**
     * 开关状态变更回调
     */
    onChange: PropTypes.func,
};

SwitchLoading.defaultProps = {
    // checkedChildren: '开启',
    // unCheckedChildren: '关闭',
    defaultChecked: false,
    // loading: true,
    // onChange: (checked, event) => {
    //     console.log('checked', checked);
    // },
    // onClick: (checked, event) => {
    //     console.log('clicked', checked);
    // },
    // checked: true,
};

SwitchLoading.displayName = 'SwitchLoading';
