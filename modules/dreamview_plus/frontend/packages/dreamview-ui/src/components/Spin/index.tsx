import React from 'react';
import { Spin as InternalSpin, SpinProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import LoaderApolloSrc from './loader_apollo.gif';

import './index.less';

export function Spin(props: SpinProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('Spin', customizePrefixCls);

    return <InternalSpin prefixCls={prefixCls} {...rest} />;
}

Spin.propTypes = {};

Spin.defaultProps = {
    indicator: (
        <div className='lds-dual-ring'>
            <img src={LoaderApolloSrc} alt='' />
        </div>
    ),
};

Spin.displayName = 'Spin';
