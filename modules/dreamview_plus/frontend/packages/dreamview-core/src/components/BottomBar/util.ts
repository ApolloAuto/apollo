import { useRef, useState, useCallback, useEffect } from 'react';

export enum EMUN_OPERATE_STATUS {
    SUCCESS,
    PROGRESSING,
    FAILED,
}
export enum PlayRecordStatus {
    RUNNING = 'RUNNING',
    PAUSED = 'PAUSED',
    CLOSED = 'CLOSED',
}

export enum EMUN_TIMEDOWN_STATUS {
    INIT,
    PORGRESSING,
}

export const popoverStatus = (operateStatus: EMUN_OPERATE_STATUS, timedownStatus: EMUN_TIMEDOWN_STATUS) => {
    if (timedownStatus === EMUN_TIMEDOWN_STATUS.INIT) {
        return '';
    }
    if (operateStatus === EMUN_OPERATE_STATUS.SUCCESS) {
        return 'operate-success';
    }
    if (operateStatus === EMUN_OPERATE_STATUS.FAILED) {
        return 'operate-failed';
    }
    return '';
};

export function useTimeDown(
    defaultText: string,
    time: number,
): [string, (i: string, p: string) => void, EMUN_TIMEDOWN_STATUS] {
    const [text, setText] = useState<string>(defaultText);
    const running = useRef<EMUN_TIMEDOWN_STATUS>(EMUN_TIMEDOWN_STATUS.INIT);
    const timer = useRef<number>();

    useEffect(() => {
        setText(defaultText);
    }, [defaultText]);

    const changeIntoProgressing = () => {
        running.current = EMUN_TIMEDOWN_STATUS.PORGRESSING;
    };

    const changeIntoInit = () => {
        running.current = EMUN_TIMEDOWN_STATUS.INIT;
    };

    const clearTimer = () => {
        clearInterval(timer.current);
    };

    const trigger = useCallback((init: string, progressText: string) => {
        if (running.current === EMUN_TIMEDOWN_STATUS.PORGRESSING) {
            return;
        }

        let curTime = time;
        changeIntoProgressing();
        clearTimer();
        setText(`${progressText}(${time}s)`);
        timer.current = window.setInterval(() => {
            curTime -= 1;
            if (curTime === 0) {
                clearTimer();
                changeIntoInit();
                setText(init);
            } else {
                setText(`${progressText}(${curTime}s)`);
            }
        }, 1000);
    }, []);

    useEffect(
        () => () => {
            clearTimer();
        },
        [],
    );

    return [text, trigger, running.current];
}

export function useKeyDownEvent(onKeyDownPlaycallback: () => void, onKeyDownStopcallback: () => void) {
    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'p') {
                onKeyDownPlaycallback();
            }
            if (e.key === 's') {
                onKeyDownStopcallback();
            }
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [onKeyDownPlaycallback, onKeyDownStopcallback]);
}
