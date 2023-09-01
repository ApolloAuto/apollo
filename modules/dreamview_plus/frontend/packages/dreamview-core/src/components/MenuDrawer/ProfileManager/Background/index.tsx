import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { usePickHmiStore, hmiUtils } from '@dreamview/dreamview-core/src/store/HmiStore';
import useStyle from './useStyle';

export default function Background(props: React.PropsWithChildren) {
    const [hmi] = usePickHmiStore();

    const isPlayerControlBarShow = useMemo(() => hmiUtils.isPlayerControlShow(hmi), [hmi]);
    const { classes } = useStyle();

    return (
        <div
            style={{ height: `calc(100vh - 176px - ${isPlayerControlBarShow ? 63 : 0}px)` }}
            className={classes['table-background']}
        >
            {props.children}
        </div>
    );
}
