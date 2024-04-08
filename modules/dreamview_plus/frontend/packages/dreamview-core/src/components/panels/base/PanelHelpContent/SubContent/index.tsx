import React, { PropsWithChildren } from 'react';
import { useMakeStyle } from '@dreamview/dreamview-theme';

type SubContentProps = PropsWithChildren & {
    style?: React.CSSProperties;
};

export function SubContentOrigin(props: SubContentProps) {
    const { classes } = useMakeStyle((theme) => ({
        'dreamview-panel-sub-content': {
            fontFamily: 'PingFangSC-Regular',
            fontSize: theme.tokens.font.size.regular,
            color: theme.tokens.colors.fontColor4,
        },
    }))();
    return (
        <div
            style={{
                ...props?.style,
            }}
            className={classes['dreamview-panel-sub-content']}
        >
            {props.children}
        </div>
    );
}

export const SubContent = React.memo(SubContentOrigin);
