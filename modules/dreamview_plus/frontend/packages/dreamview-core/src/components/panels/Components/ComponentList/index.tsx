import React, { useMemo } from 'react';
import useStyle from '../useStyle';
import { OKStatus } from '../ComponentsStatus/OKStatus';
import { FatalStatus } from '../ComponentsStatus/FatalStatus';
import { WarnStatus } from '../ComponentsStatus/WarnStatus';
import { ErrorStatus } from '../ComponentsStatus/ErrorStatus';

export type ComponentListProps = {
    name: string;
    status: string;
};

export enum ComponentStatus {
    UNKNOWN = 'UNKNOWN',
    OK = 'OK',
    WARN = 'WARN',
    FATAL = 'FATAL',
    ERROR = 'ERROR',
}

export function ComponentListItem(props: ComponentListProps) {
    const { classes } = useStyle();

    const status = useMemo(() => {
        switch (props.status) {
            case ComponentStatus.OK:
                return <OKStatus />;
            case ComponentStatus.FATAL:
                return <FatalStatus />;
            case ComponentStatus.WARN:
                return <WarnStatus />;
            case ComponentStatus.ERROR:
                return <ErrorStatus />;
            default:
                return null;
        }
    }, [props.status]);

    return (
        <div className={classes['panel-components-list-item']}>
            <span>{props.name}</span>
            {status}
        </div>
    );
}
