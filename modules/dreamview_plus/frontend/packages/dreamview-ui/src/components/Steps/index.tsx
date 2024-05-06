import React from 'react';
import { Steps as InternalSteps, StepsProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import './index.less';

export function Steps<T>(props: StepsProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('steps', customizePrefixCls);

    return (
        <InternalSteps prefixCls={prefixCls} {...rest}>
            {children}
        </InternalSteps>
    );
}

Steps.propTypes = {};

Steps.defaultProps = {};

Steps.displayName = 'Steps';
