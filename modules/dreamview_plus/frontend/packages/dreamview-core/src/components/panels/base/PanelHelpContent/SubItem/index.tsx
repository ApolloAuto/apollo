import React, { PropsWithChildren } from 'react';
import './index.less';

type SubItemProps = {
    border?: boolean;
} & PropsWithChildren;

export function SubItemOrigin(props: SubItemProps) {
    return (
        <div
            style={{
                borderBottom: props?.border ? '1px solid rgba(151,151,151,1)' : 'none',
            }}
            className='dreamview-panel-sub-item'
        >
            {props.children}
        </div>
    );
}

export const SubItem = React.memo(SubItemOrigin);
