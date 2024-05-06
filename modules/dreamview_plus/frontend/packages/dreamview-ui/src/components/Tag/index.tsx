import React from 'react';
import { Tag as InternalTag, TagProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import './index.less';

export function Tag(props: TagProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('tag', customizePrefixCls);

    return (
        <InternalTag prefixCls={prefixCls} {...rest}>
            {children}
        </InternalTag>
    );
}

Tag.propTypes = {};

Tag.defaultProps = {};

Tag.displayName = 'Tag';
