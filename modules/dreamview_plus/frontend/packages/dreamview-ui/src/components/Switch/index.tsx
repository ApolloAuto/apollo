import classNames from 'classnames';
import RcSwitch from 'rc-switch';
import React, { useState } from 'react';
import PropTypes from 'prop-types';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconPark from '../../IconPark';
import './index.less';

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

export const Switch = React.forwardRef<HTMLButtonElement, SwitchProps>(
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
        const prefixCls = getPrefixCls('switch', customizePrefixCls);
        const loadingIcon = (
            <div className={`${prefixCls}-handle`}>
                {loading && <IconPark name='IcBackTheStartingPoint' spin className={`${prefixCls}-loading-icon`} />}
            </div>
        );

        const mergedSize = customizeSize || 'default';

        const classes = classNames(
            {
                [`${prefixCls}-small`]: mergedSize === 'small',
                [`${prefixCls}-loading`]: loading,
            },
            className,
            rootClassName,
        );

        return (
            <RcSwitch
                {...props}
                prefixCls={prefixCls}
                className={classes}
                disabled={mergedDisabled}
                ref={ref}
                loadingIcon={loadingIcon}
            />
        );
    },
);

Switch.propTypes = {
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

Switch.defaultProps = {
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

Switch.displayName = 'Switch';
