import React, { PropsWithChildren } from 'react';
import './index.less';

export function SubContentOrigin(props: PropsWithChildren) {
    return <div className='dreamview-panel-sub-content'>{props.children}</div>;
}

export const SubContent = React.memo(SubContentOrigin);
