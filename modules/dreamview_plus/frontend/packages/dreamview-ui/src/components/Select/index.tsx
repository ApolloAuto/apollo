import React from 'react';
import { Select as InternalSelect, SelectProps as AntdSelectProps } from 'antd';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconPark from '../../IconPark';
import NoDataPlaceHolder from './NoData';

export type SelectProps = AntdSelectProps;
const useStyle = makeStylesWithProps<{ classname: string }>()((theme, props) => ({
    [props.classname]: {
        '&:hover .dreamview-select-selector': {
            border: `${theme.components.select.borderHover}`,
            boxShadow: theme.components.select.boxShadowHover,
            backgroundColor: theme.components.select.bgColorHover,
        },
        '&.dreamview-select-single': {
            '& .dreamview-select-selector': {
                color: `${theme.components.select.color}`,
                fontFamily: 'PingFangSC-Regular',
                height: '36px !important',
                backgroundColor: theme.components.select.bgColor,
                border: theme.components.select.border,
                boxShadow: theme.components.select.boxShadow,
                borderRadius: theme.components.select.borderRadius,

                '& .dreamview-select-selection-item:hover': {
                    color: `${theme.components.select.colorHover}`,
                },

                '& .dreamview-select-selection-item': {
                    overflow: 'hidden',
                    whiteSpace: 'nowrap',
                    textOverflow: 'ellipsis',
                    lineHeight: '36px',
                },

                '& .dreamview-select-selection-placeholder': {
                    color: '#A6B5CC',
                    fontFamily: 'PingFangSC-Regular',
                },

                '&:hover': {
                    backgroundColor: theme.components.select.bgColorHover,
                    boxShadow: theme.components.select.boxShadowHover,
                    border: theme.components.select.borderHover,
                },
            },
            '&.dreamview-select-open': {
                color: `${theme.components.select.optionColor} !important`,
                fontFamily: 'PingFangSC-Regular',

                '& .dreamview-select-selection-item': {
                    color: `${theme.components.select.optionColor} !important`,
                },

                '& .dreamview-select-arrow': {
                    transform: 'rotate(180deg)',
                },
            },

            '& .dreamview-select-arrow': {
                color: theme.components.select.iconColor,
                fontSize: '16px',
                transition: 'all 0.1s ease-in-out',
                '&::before': {
                    backgroundColor: 'transparent',
                },
                '&::after': {
                    backgroundColor: 'transparent',
                },
            },
        },
    },
    [`${props.classname}-dropdown`]: {
        fontFamily: 'PingFangSC-Regular',
        fontSize: '14px',
        fontWeight: 400,
        background: theme.components.select.optionBgColor,
        borderRadius: '4px',
        padding: '4px 0 8px 0',

        '& .dreamview-select-item': {
            color: theme.components.select.optionColor,
            borderRadius: 0,

            '&.dreamview-select-item-option-selected': {
                color: theme.components.select.optionSelectColor,
                backgroundColor: theme.components.select.optionSelectBgColor,
            },

            '&.dreamview-select-item-option-active': {
                backgroundColor: theme.components.select.optionSelectBgColor,
            },
        },
    },
}))
export function Select(props: AntdSelectProps) {
    const { prefixCls: customizePrefixCls, className, popupClassName, ...rest } = props;
    const prefixCls = getPrefixCls('select', customizePrefixCls);
    const { cx, classes } = useStyle({ classname: prefixCls });

    return (
        <InternalSelect
            notFoundContent={<NoDataPlaceHolder />}
            suffixIcon={<IconPark name='IcArrowsDown' />}
            prefixCls={prefixCls}
            className={cx(className, classes[prefixCls])}
            popupClassName={cx(popupClassName, classes[`${prefixCls}-dropdown`])}
            {...rest}
        />
    );
}

Select.propTypes = {};

Select.defaultProps = {
    style: {
        width: '322px',
    },
};

Select.displayName = 'Select';
