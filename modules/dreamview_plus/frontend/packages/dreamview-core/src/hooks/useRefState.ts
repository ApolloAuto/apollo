import React, { useCallback, useState, MutableRefObject } from 'react';

export default function useRefState<T>(propValue?: T): [MutableRefObject<T>, (state: T) => void] {
    const value = React.useRef<T>(propValue);
    const [, triggerStateChange] = useState<number>(0);

    const changeState = useCallback((state: T) => {
        value.current = state;
        triggerStateChange((prev) => prev + 1);
    }, []);

    return [value, changeState];
}
