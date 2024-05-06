import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { WayFollowBtnMemo } from './WayFollowBtn';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import RecordBtn from '../Operate/Record';

interface ScenarioBarProps {}

function WayFollowBar(props: ScenarioBarProps) {
    const { cx, classes } = useStyle();

    return (
        <div className={cx(classes['record-controlbar-container'])}>
            <div className='ic-play-container'>
                <WayFollowBtnMemo />
            </div>
            <div className={classes['flex-center']}>
                <RecordBtn />
                <DumpBtn disabled={false} />
                <ResetBtn disabled={false} />
            </div>
        </div>
    );
}

export default React.memo(WayFollowBar);
