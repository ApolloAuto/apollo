import React, { PropsWithChildren, useContext, useMemo, useState } from 'react';
import useStyle from './useStyle';

interface IButton {
    width?: number;
    onClick?: () => void;
}

export default function Button(props: PropsWithChildren<IButton>) {
    const { width, onClick = () => true } = props;
    const { classes } = useStyle()({ width });
    return (
        <button onClick={onClick} type='button' className={classes['mode-setting-button']}>
            {props.children}
        </button>
    );
}
