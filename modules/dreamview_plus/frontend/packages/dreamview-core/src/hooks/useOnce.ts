import React from 'react';

export default function useOnce(cb: () => void) {
    const once = React.useRef(false);
    if (!once.current) {
        once.current = true;
        cb();
    }
}
