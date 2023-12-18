import React from 'react';
import { InputNumber as InternalInputNumber, InputNumberProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function InputNumber(props: InputNumberProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('input-number', customizePrefixCls);

    return (
        <InternalInputNumber prefixCls={prefixCls} {...rest}>
            {children}
        </InternalInputNumber>
    );
}

InputNumber.propTypes = {};

InputNumber.defaultProps = {};

InputNumber.displayName = 'InputNumber';
