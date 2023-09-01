import React, { PropsWithChildren } from 'react';
import './index.less';

export function SubHeader(props: PropsWithChildren) {
    return <div className='dreamview-panel-sub-header'>{props.children}</div>;
}
