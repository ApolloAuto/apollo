import React from 'react';
import { MacScrollbar, MacScrollbarProps } from 'mac-scrollbar';

function CustomScroll(props: React.PropsWithChildren<MacScrollbarProps>) {
    const { children, ...childProps } = props;
    return <MacScrollbar {...childProps}>{children}</MacScrollbar>;
}

export default React.memo(CustomScroll);
