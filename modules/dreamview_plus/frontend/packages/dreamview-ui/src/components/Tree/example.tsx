import { TabsProps } from 'antd';
import React from 'react';
import { DataNode } from 'antd/es/tree';
import { Tree } from '.';

export function Template(props: TabsProps) {
    const dig = (path = '0', level = 3) => {
        const list = [];
        for (let i = 0; i < 3; i += 1) {
            const key = `${path}-${i}`;
            const treeNode: DataNode = {
                title: key,
                key,
            };

            if (level > 0) {
                treeNode.children = dig(key, level - 1);
            }

            list.push(treeNode);
        }
        return list;
    };

    const treeData = dig();
    console.log('treeData', treeData);
    const data = [
        {
            title: 'header',
            key: 'header',
            children: [
                {
                    title: 'timestamp_spec',
                    key: 'timestamp_spec',
                    children: [
                        {
                            title: 'module_name',
                            key: 'module_name',
                        },
                    ],
                },
            ],
        },
    ];

    return (
        <Tree
            checkable={false}
            blockNode={false}
            selectable={false}
            switcherIcon={null}
            defaultExpandAll
            treeData={data}
        />
    );
}
