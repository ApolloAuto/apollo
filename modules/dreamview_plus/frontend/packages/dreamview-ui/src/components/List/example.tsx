import { ListProps } from 'antd';
import React, { useState } from 'react';
import { List } from '.';
import { ListItem } from './ListItem';
import IconPark from '../../IconPark';

export function Template<T>(props: ListProps<T>) {
    const [data, setData] = useState([
        {
            key: 'ceshi1',
            val: '测试数据1',
        },
        {
            key: 'ceshi2',
            val: '测试数据2',
        },
        {
            key: 'ceshi3',
            val: '测试数据3',
        },
        {
            key: 'ceshi4',
            val: '测试数据4',
        },
        {
            key: 'ceshi5',
            val: '测试数据5',
        },
        {
            key: 'ceshi6',
            val: '测试数据6',
        },
        {
            key: 'ceshi7',
            val: '测试数据7',
        },
        {
            key: 'ceshi8',
            val: '测试数据8',
        },
    ]);

    const [selectedKey, setSelected] = useState<string>();

    const itemOnClick = (e: React.MouseEvent<HTMLElement>, key: string) => {
        setSelected(key);
    };

    return (
        <List>
            {data.map((item) => (
                <ListItem
                    activeIcon={<IconPark name='IcSucceed' />}
                    hoverIcon={<IconPark name='IcUse' />}
                    onClick={(e) => {
                        itemOnClick(e, item.key);
                    }}
                    key={item.key}
                    active={item.key === selectedKey}
                >
                    <div>{item.val}</div>
                </ListItem>
            ))}
        </List>
    );
}
