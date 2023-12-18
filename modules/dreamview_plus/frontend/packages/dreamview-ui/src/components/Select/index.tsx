import React from 'react';
import { Select as InternalSelect, SelectProps as AntdSelectProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import { IconIcArrowsDown } from '../../icons';
import NoDataPlaceHolder from './NoData';

export type SelectProps = AntdSelectProps;

export function Select(props: AntdSelectProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('select', customizePrefixCls);

    return (
        <InternalSelect
            notFoundContent={<NoDataPlaceHolder />}
            suffixIcon={<IconIcArrowsDown />}
            prefixCls={prefixCls}
            {...rest}
        />
    );
}

Select.propTypes = {};

Select.defaultProps = {
    style: {
        width: '322px',
    },
    options: [
        '1111111111111111111111111111111111111111111111111111111111111111111111111',
        '2',
        '3',
        '4',
        '5',
        '6',
        '7',
        '8',
        '9',
    ].map((value) => ({
        label: value,
        value,
    })),
};

Select.displayName = 'Select';
