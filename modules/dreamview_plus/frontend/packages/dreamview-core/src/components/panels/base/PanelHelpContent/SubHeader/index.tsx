import React, { PropsWithChildren, CSSProperties } from 'react';
import './index.less';

type Props = PropsWithChildren & {
    style?: CSSProperties;
};

export function SubHeaderOrigin(props: Props) {
    return (
        <div
            style={{
                ...props?.style,
            }}
            className='dreamview-panel-sub-header'
        >
            {props.children}
        </div>
    );
}

export const SubHeader = React.memo(SubHeaderOrigin);
