import React, { ReactNode, useMemo, useState } from 'react';
import { List } from 'antd';
import './index.less';
import { ListItemProps } from 'antd/es/list';
import classnames from 'classnames';
import { getPrefixCls } from '../../../tools/prefixCls/prefixCls';

export function ListItem(
    props: ListItemProps & {
        active?: boolean | undefined;
        activeIcon?: ReactNode;
        hoverIcon?: ReactNode;
    },
) {
    const { prefixCls: customizePrefixCls, children, active = false, activeIcon, hoverIcon, ...rest } = props;
    const prefixCls = getPrefixCls('list-item', customizePrefixCls);
    const className = classnames({
        'dreamview-list-item-active': active,
    });

    return (
        <List.Item
            className={className}
            style={{
                width: 350,
                height: 40,
            }}
            prefixCls={prefixCls}
            {...rest}
        >
            <span className='icon-use'>{active ? activeIcon : hoverIcon}</span>
            {children}
        </List.Item>
    );
}

ListItem.displayName = 'ListItem';
