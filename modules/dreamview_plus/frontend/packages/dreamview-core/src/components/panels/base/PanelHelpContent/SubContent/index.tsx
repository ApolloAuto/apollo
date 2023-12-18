import React, { PropsWithChildren } from 'react';
import './index.less';

type SubContentProps = PropsWithChildren & {
    style?: React.CSSProperties;
};

export function SubContentOrigin(props: SubContentProps) {
    return (
        <div
            style={{
                ...props?.style,
            }}
            className='dreamview-panel-sub-content'
        >
            {props.children}
        </div>
    );
}

export const SubContent = React.memo(SubContentOrigin);
