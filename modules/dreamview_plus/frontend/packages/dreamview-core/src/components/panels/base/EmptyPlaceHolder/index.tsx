/* eslint-disable global-require */
import React, { HTMLAttributes } from 'react';
import useStyle from './useStyle';

export type NoDataPlaceHolderProps = {
    text: string;
} & HTMLAttributes<HTMLDivElement>;

function EmptyPlaceHolder(props: NoDataPlaceHolderProps) {
    const { classes } = useStyle();

    return (
        <div className={classes['dreamview-nodata-placeholder']} {...props}>
            <img
                style={{
                    width: 160,
                    height: 100,
                }}
                src={require('../imgs/ic_empty_page_no_data@3x.png')}
                alt='no data'
            />
            <div>{props.text}</div>
        </div>
    );
}

export default React.memo(EmptyPlaceHolder);
