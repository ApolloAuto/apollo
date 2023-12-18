/* eslint-disable global-require */
import React, { useMemo } from 'react';
import useStyle from './useStyle';

export enum SignalEnum {
    NO_SIGNALS = 'NO SIGNALS',
    UNKNOWN = 'UNKNOWN',
    RED = 'RED',
    YELLOW = 'YELLOW',
    GREEN = 'GREEN',
}

export type SignalProps = {
    signal: SignalEnum;
};

function Signal(props: SignalProps) {
    const { classes } = useStyle();

    const child = useMemo(() => {
        switch (props.signal) {
            case SignalEnum.GREEN:
                return (
                    <>
                        <img
                            style={{
                                width: 18,
                                height: 18,
                            }}
                            src={require('./imgs/icon_green_light@3x.png')}
                            alt='green light'
                        />
                        &nbsp;GREEN
                    </>
                );
            case SignalEnum.YELLOW:
                return (
                    <>
                        <img
                            style={{
                                width: 18,
                                height: 18,
                            }}
                            src={require('./imgs/icon_yellow_light@3x.png')}
                            alt='yellow light'
                        />
                        &nbsp;YELLOW
                    </>
                );
            case SignalEnum.RED:
                return (
                    <>
                        <img
                            style={{
                                width: 18,
                                height: 18,
                            }}
                            src={require('./imgs/icon_red_light@3x.png')}
                            alt='green light'
                        />
                        &nbsp;RED
                    </>
                );
            case SignalEnum.UNKNOWN:
                return <>UNKNOWN</>;
            default:
                return <>NO SIGNALS</>;
        }
    }, [props]);

    return <div className={classes['dashboard-signal']}>{child}</div>;
}

export default React.memo(Signal);
