import React from 'react';
import { MacScrollbar, MacScrollbarProps } from 'mac-scrollbar';

function CustomScroll(props: React.PropsWithChildren<MacScrollbarProps>, ref?: any) {
    const { children, ...childProps } = props;
    return (
        <MacScrollbar {...childProps} ref={ref}>
            {children}
        </MacScrollbar>
    );
}

export default React.memo(React.forwardRef(CustomScroll));
