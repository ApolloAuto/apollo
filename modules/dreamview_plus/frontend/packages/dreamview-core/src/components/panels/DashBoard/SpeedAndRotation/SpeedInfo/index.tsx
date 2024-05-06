import React, { useMemo } from 'react';
import { Progress } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';
import { useThemeContext } from '@dreamview/dreamview-theme';

type SpeedInfoProps = {
    speed?: number;
    throttlePercent?: number;
    brakePercent?: number;
};

function fromMsToKmh(speed: number) {
    if (speed !== undefined && !Number.isNaN(speed)) {
        return Math.round(speed * 3.6);
    }

    return 0;
}

function toFix1(percent: number) {
    return parseFloat(percent.toFixed(1));
}

function SpeedInfo(props: SpeedInfoProps) {
    const { classes } = useStyle();
    const { tokens } = useThemeContext();

    return (
        <div className={classes['dashboard-speed-info']}>
            <div
                style={{
                    marginTop: 10,
                    marginBottom: 10,
                }}
            >
                <div className={classes['dashboard-speed-info-font']}>{fromMsToKmh(props?.speed)}</div>
                <span className={classes['dashboard-speed-info-font-small']}>Speed(km/h)</span>
            </div>
            <div
                style={{
                    height: '60px',
                    width: '100%',
                }}
            >
                <div
                    style={{
                        width: '100%',
                        padding: '0 16px 6px',
                        height: '30px',
                    }}
                >
                    <div className={classes['dashboard-speed-brake']}>
                        <span className={classes['dashboard-speed-brake-words']}>Brake</span>
                        <span className={classes['dashboard-speed-brake-number']}>
                            {`${props?.brakePercent ? toFix1(props?.brakePercent) : 0}%`}
                        </span>
                    </div>
                    <Progress
                        style={{
                            top: -6,
                        }}
                        strokeColor='#F75660'
                        trailColor={tokens.components.dashBoard.progressBgColor}
                        strokeLinecap='butt'
                        percent={props?.brakePercent ? toFix1(props?.brakePercent) : 0}
                        showInfo={false}
                    />
                    {/* <div className={classes['dashboard-speed-brake-percent']}>2</div> */}
                </div>
                <div
                    style={{
                        width: '100%',
                        padding: '0 16px 6px',
                        height: '30px',
                    }}
                >
                    <div className={classes['dashboard-speed-acc']}>
                        <span className={classes['dashboard-speed-acc-words']}>Accelerator</span>
                        <span className={classes['dashboard-speed-acc-number']}>
                            {`${props?.throttlePercent ? toFix1(props?.throttlePercent) : 0}%`}
                        </span>
                    </div>
                    <Progress
                        style={{
                            top: -6,
                        }}
                        strokeColor='#3288FA'
                        trailColor={tokens.components.dashBoard.progressBgColor}
                        strokeLinecap='butt'
                        percent={props?.throttlePercent ? toFix1(props?.throttlePercent) : 0}
                        showInfo={false}
                    />
                </div>
            </div>
        </div>
    );
}

export default React.memo(SpeedInfo);
