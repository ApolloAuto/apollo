import React, { PropsWithChildren } from 'react';
import './index.less';

export function SubHeaderOrigin(props: PropsWithChildren) {
    return <div className='dreamview-panel-sub-header'>{props.children}</div>;
}

export const SubHeader = React.memo(SubHeaderOrigin);
