import React, { PropsWithChildren, CSSProperties } from 'react';
import { useMakeStyle } from '@dreamview/dreamview-theme';

type Props = PropsWithChildren & {
    style?: CSSProperties;
};

export function SubHeaderOrigin(props: Props) {
    const { classes, cx } = useMakeStyle((theme) => ({
        'dreamview-panel-sub-header': {
            fontFamily: 'PingFangSC-Medium',
            fontSize: '16px',
            color: theme.tokens.colors.fontColor5,
            fontWeight: 500,
            margin: '24px 0 6px 0',
        },
    }))();
    return (
        <div
            style={{
                ...props?.style,
            }}
            className={classes['dreamview-panel-sub-header']}
        >
            {props.children}
        </div>
    );
}

export const SubHeader = React.memo(SubHeaderOrigin);
