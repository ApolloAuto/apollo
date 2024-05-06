import React from 'react';
import { Spin as InternalSpin, SpinProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import LoaderApolloSrc from './loader_apollo.gif';

import './index.less';

function SpinWithMemo(props: SpinProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('Spin', customizePrefixCls);
    return <InternalSpin prefixCls={prefixCls} {...rest} />;
}

export const Spin = React.memo(SpinWithMemo);

SpinWithMemo.propTypes = {};

SpinWithMemo.defaultProps = {
    indicator: (
        <div className='lds-dual-ring'>
            <img src={LoaderApolloSrc} alt='' />
        </div>
    ),
};

Spin.displayName = 'Spin';
