import React, { PropsWithChildren } from 'react';
import './index.less';

export function SubContent(props: PropsWithChildren) {
    return <div className='dreamview-panel-sub-content'>{props.children}</div>;
}
