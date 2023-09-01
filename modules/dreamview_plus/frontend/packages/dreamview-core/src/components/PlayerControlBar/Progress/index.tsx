import React, { useMemo, useRef } from 'react';
import useStyle from './useStyle';

interface IProgress {
    className?: string;
    duration: number;
    progress: number;
    onChange: (duration: number, progress: string) => void;
}

const Nill = () => true;

function Progress(props: IProgress) {
    const dragElem = useRef<any>();
    const { className, duration, progress, onChange = Nill } = props;
    const style = useMemo(() => {
        const precent = (progress / duration) * 100;
        return {
            width: `${precent || 0}%`,
        };
    }, [duration, progress]);

    const { classes, cx } = useStyle();

    const getPointerPositionX = (e: any) => {
        const clientLeft = e.clientX;
        const { left } = dragElem.current.getBoundingClientRect();
        const maxPointerPositionX = dragElem.current.offsetWidth;
        const pointerPositionX = Math.min(Math.max(clientLeft - left, 0), maxPointerPositionX);
        return pointerPositionX;
    };

    const bindMoveEvent = () => {
        const moveHandler = (e: any) => {
            const maxPointerPositionX = dragElem.current.offsetWidth;
            const pointerPositionX = getPointerPositionX(e);
            const nextProgress = (pointerPositionX / maxPointerPositionX) * duration;
            onChange(nextProgress, `${(pointerPositionX / maxPointerPositionX) * 100}%`);
        };
        const upHandler = () => {
            window.removeEventListener('mouseup', upHandler);
            window.removeEventListener('mousemove', moveHandler);
        };
        window.addEventListener('mousemove', moveHandler);
        window.addEventListener('mouseup', upHandler);
    };

    const onMouseDown = (e: any) => {
        const maxPointerPositionX = dragElem.current.offsetWidth;
        const pointerPositionX = getPointerPositionX(e);
        const nextProgress = (pointerPositionX / maxPointerPositionX) * duration;
        onChange(nextProgress, `${(pointerPositionX / maxPointerPositionX) * 100}%`);
    };

    return (
        <div ref={dragElem} onMouseDown={onMouseDown} className={cx(classes['progress-container'], className)}>
            <div className='progress-background'>
                <div style={{ width: style.width }} className='progress-inner' />
                <div style={{ left: `calc(${style.width} - 6px)` }} className='progress-pointer' />
            </div>
        </div>
    );
}

export default React.memo(Progress);
