import React from 'react';
import { Collapse as InternalCollapse, CollapseProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconPark from '../../IconPark';

export function Collapse(props: CollapseProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('collapse', customizePrefixCls);

    return (
        <InternalCollapse
            // eslint-disable-next-line react/no-unstable-nested-components
            expandIcon={({ isActive }) => (
                <IconPark
                    name='IcPullDownExpansion'
                    style={{
                        fontSize: 16,
                    }}
                    rotate={isActive ? 0 : -90}
                />
            )}
            ghost
            prefixCls={prefixCls}
            {...rest}
        />
    );
}

Collapse.propTypes = {};

Collapse.defaultProps = {};

Collapse.displayName = 'Collapse';
