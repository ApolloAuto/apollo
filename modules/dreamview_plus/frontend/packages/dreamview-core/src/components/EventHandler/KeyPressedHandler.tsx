import React, { useEffect, useState } from 'react';
import { useEventHandlersContext } from '../../store/EventHandlersStore';

function KeyPressed(props) {
    const [isBlack, setIsBlack] = useState(false);
    const eventHandlers = useEventHandlersContext();
    const { setMultiPressedKey } = eventHandlers.keydown;
    useEffect(() => {
        setMultiPressedKey(
            (event) => {
                if (event instanceof KeyboardEvent) {
                    setIsBlack((prev) => !prev);
                }
            },
            ['a', 'x'],
            'ctrlKey',
        );
    }, []);
    const color = isBlack ? 'balck' : 'pink';
    return (
        <div
            style={{
                width: isBlack ? 100 : 200,
                height: 100,
                backgroundColor: color,
            }}
        >
            按下 Ctrl+a+x 切换背景色
        </div>
    );
}

export default React.memo(KeyPressed);
