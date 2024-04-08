/* eslint-disable global-require */
import React, { HTMLAttributes } from 'react';
import { useImagePrak } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

export type NoDataPlaceHolderProps = {
    text: string;
} & HTMLAttributes<HTMLDivElement>;

function EmptyPlaceHolder(props: NoDataPlaceHolderProps) {
    const { classes } = useStyle();
    const src = useImagePrak('ic_empty_page_no_data');

    return (
        <div className={classes['dreamview-nodata-placeholder']} {...props}>
            <img
                style={{
                    width: 160,
                    height: 100,
                }}
                src={src}
                alt='no data'
            />
            <div>{props.text}</div>
        </div>
    );
}

export default React.memo(EmptyPlaceHolder);
